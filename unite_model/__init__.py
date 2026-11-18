# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTIBILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

"""Unite model

Joins every mesh of a collection (typically one imported COLLADA house) into
a single object and repairs what double-sided SketchUp exports break:

1. Every surface is exported twice (front + back face, same vertices).
   "Merge by Distance" then deletes a random face of each pair, so the visible
   front side sometimes disappears. We remove the back face of each pair
   *before* welding, keeping the face with the front material.
2. Vertices are not shared between triangles. They are welded afterwards.
3. Some faces still point inside. Each face is tested with rays: the side
   with more free space around it is the outside. Faces the rays cannot
   decide inherit orientation from their decided neighbours.
"""

import math
from collections import Counter, defaultdict, deque

import bmesh
import bpy
from bpy.props import BoolProperty, EnumProperty, FloatProperty
from mathutils import Vector, kdtree
from mathutils.bvhtree import BVHTree

RESULT_TAG = "unite_model_result"
TWO_SIDED_LAYER = "unite_two_sided"


# --------------------------------------------------------------------------
# Geometry core (pure bmesh, no operators, so it works on any collection)
# --------------------------------------------------------------------------

def _hemisphere_dirs(count):
    """Evenly spread directions on the +Z hemisphere (Fibonacci spiral)."""
    golden = math.pi * (3.0 - math.sqrt(5.0))
    dirs = []
    for i in range(count):
        z = 1.0 - (i + 0.5) / count
        r = math.sqrt(max(0.0, 1.0 - z * z))
        t = golden * i
        dirs.append(Vector((r * math.cos(t), r * math.sin(t), z)))
    return dirs


def build_bmesh(objects):
    """Copy all mesh objects into one bmesh in world space.

    Returns (bm, materials) where materials[i] is the material for
    face.material_index == i (may contain None).
    """
    bm = bmesh.new()
    uv_layer = bm.loops.layers.uv.new("UVMap")
    materials = []
    mat_lookup = {}
    skipped = 0

    for obj in objects:
        me = obj.data
        mw = obj.matrix_world
        mirrored = mw.determinant() < 0.0
        src_uv = me.uv_layers.active
        verts = [bm.verts.new(mw @ v.co) for v in me.vertices]

        for poly in me.polygons:
            vids = list(poly.vertices)
            loops = list(poly.loop_indices)
            if mirrored:  # negative scale flips winding, undo it
                vids.reverse()
                loops.reverse()
            try:
                face = bm.faces.new([verts[i] for i in vids])
            except ValueError:  # face with repeated vertices
                skipped += 1
                continue

            mat = None
            if me.materials and poly.material_index < len(me.materials):
                mat = me.materials[poly.material_index]
            key = mat.name if mat else None
            if key not in mat_lookup:
                mat_lookup[key] = len(materials)
                materials.append(mat)
            face.material_index = mat_lookup[key]

            if src_uv:
                for loop, li in zip(face.loops, loops):
                    loop[uv_layer].uv = src_uv.data[li].uv

    bm.verts.index_update()
    bm.faces.index_update()
    return bm, materials, skipped


def _cluster_vertices(bm, dist):
    """Map every vertex index to a representative index within `dist`."""
    bm.verts.index_update()
    tree = kdtree.KDTree(len(bm.verts))
    for v in bm.verts:
        tree.insert(v.co, v.index)
    tree.balance()
    rep = list(range(len(bm.verts)))
    for v in bm.verts:
        i = v.index
        if rep[i] != i:
            continue
        for _co, j, _d in tree.find_range(v.co, dist):
            if j > i and rep[j] == j:
                rep[j] = i
    return rep


def remove_back_faces(bm, dist):
    """Delete the duplicate (back) face of every double-sided pair.

    Faces are duplicates when their vertex positions coincide within `dist`.
    The back face is recognised by its material: the material that occurs in
    most duplicate pairs (SketchUp's default back material). If a pair has no
    such face, one face is kept and orientation is decided later by rays.
    Returns (removed_count, back_material_index or None).
    """
    rep = _cluster_vertices(bm, dist)
    groups = defaultdict(list)
    for f in bm.faces:
        key = frozenset(rep[v.index] for v in f.verts)
        if len(key) >= 3:
            groups[key].append(f)

    dup_groups = [g for g in groups.values() if len(g) > 1]
    if not dup_groups:
        return 0, None

    mat_count = Counter()
    for g in dup_groups:
        for idx in {f.material_index for f in g}:
            mat_count[idx] += 1
    back, hits = mat_count.most_common(1)[0]
    if hits < 0.5 * len(dup_groups):
        back = None  # no clear back material

    kill = []
    for g in dup_groups:
        keep = next((f for f in g if f.material_index != back), g[0])
        kill.extend(f for f in g if f is not keep)
    bmesh.ops.delete(bm, geom=kill, context='FACES_ONLY')
    return len(kill), back


def weld(bm, dist):
    before = len(bm.verts)
    bmesh.ops.remove_doubles(bm, verts=bm.verts[:], dist=dist)
    loose_edges = [e for e in bm.edges if not e.link_faces]
    if loose_edges:
        bmesh.ops.delete(bm, geom=loose_edges, context='EDGES')
    loose_verts = [v for v in bm.verts if not v.link_faces]
    if loose_verts:
        bmesh.ops.delete(bm, geom=loose_verts, context='VERTS')
    return before - len(bm.verts)


def _is_flat_sliver(face):
    """True when the face has (almost) no area compared to its size,
    i.e. its vertices lie on one line."""
    longest = max(e.calc_length() for e in face.edges)
    if longest == 0.0:
        return True
    return face.calc_area() < 1e-6 * longest * longest


def remove_degenerate(bm, dist):
    """Remove zero-area slivers (3 collinear vertices).

    They are invisible, have no normal and break the winding between their
    neighbours. Short ones are collapsed, long thin ones are deleted.
    Returns the number of faces removed.
    """
    before = len(bm.faces)
    bmesh.ops.dissolve_degenerate(bm, dist=max(dist, 1e-6), edges=bm.edges[:])
    for _ in range(3):  # deleting can expose more coincident geometry
        flat = [f for f in bm.faces if _is_flat_sliver(f)]
        if not flat:
            break
        bmesh.ops.delete(bm, geom=flat, context='FACES_ONLY')
        weld(bm, 0.0)
    return before - len(bm.faces)


def _free_directions(bvh, origin, normal, dirs, far, eps, ground=None):
    """How many hemisphere rays around `normal` leave the model.

    `ground` is the terrain height or None. A ray heading down that misses
    the model lands on the terrain. That spot is outside only when it is open
    to the sky; under the house, a step or a threshold it is not. So the
    underside of eaves counts as outside (slanted rays land beside the house),
    while the open bottom of a model does not.
    """
    up = Vector((0.0, 0.0, 1.0))
    rot = normal.to_track_quat('Z', 'Y')
    start = origin + normal * eps
    free = 0
    for d in dirs:
        dd = rot @ d
        if bvh.ray_cast(start, dd, far)[0] is not None:
            continue
        if ground is not None and dd.z < -1e-6:
            t = (ground - start.z) / dd.z
            if t <= 0.0:
                continue  # already underground
            spot = start + dd * t
            if bvh.ray_cast(spot + up * eps, up, far)[0] is not None:
                continue  # landed under the model
        free += 1
    return free


def _outside_views(count, upper_only=True):
    """Viewing directions around the model: zenith to horizon, or the whole
    sphere when the model does not stand on the ground."""
    golden = math.pi * (3.0 - math.sqrt(5.0))
    views = []
    for i in range(count):
        if upper_only:
            z = 1.0 - (i + 0.5) / count
        else:
            z = 1.0 - 2.0 * (i + 0.5) / count
        r = math.sqrt(max(0.0, 1.0 - z * z))
        t = golden * i
        views.append(Vector((r * math.cos(t), r * math.sin(t), z)))
    return views


def _outside_hits(bm, bvh, center, size, views, grid, ground=None):
    """Shoot parallel rays at the model from every view direction.

    Returns {face_index: [front_hits, back_hits]} for the first face each ray
    meets, i.e. what a camera outside the model can actually see. With
    `ground`, a ray coming from below may only see points above the terrain.
    """
    hits = defaultdict(lambda: [0, 0])
    for d in views:
        cam = center + d * size * 2.0
        u = d.orthogonal().normalized()
        w = d.cross(u).normalized()
        step = 1.0 / grid
        for a in range(grid):
            ua = u * ((a + 0.5) * step - 0.5) * size
            for b in range(grid):
                origin = cam + ua + w * (((b + 0.5) * step - 0.5) * size)
                loc, _n, idx, _dist = bvh.ray_cast(origin, -d, size * 4.0)
                if idx is None:
                    continue
                if ground is not None and d.z < 0.0 and loc.z <= ground:
                    continue  # this point would be seen from underground
                if bm.faces[idx].normal.dot(d) >= 0.0:
                    hits[idx][0] += 1
                else:
                    hits[idx][1] += 1
    return hits


def orient_outward(bm, ray_count=48, remove_enclosed=True, on_ground=True,
                   view_count=64, view_grid=160, enclosed_ray_count=1024):
    """Flip faces so their normal points to the outside of the model.

    Step 1: per face, compare free space in front of and behind it (rays
            into both hemispheres). The freer side is the outside.
    Step 2: faces step 1 cannot decide copy the orientation of a decided
            neighbour across a clean (2-face) edge.
    Step 3: safety net - the model is "photographed" from many directions;
            any face seen from its back is flipped.
    Faces with no free space on either side are enclosed (hidden inside the
    model). With remove_enclosed they are deleted, unless step 3 saw them.

    With on_ground the model stands on terrain at its lowest point: nothing is
    visible from below, and horizontal faces lying on the ground are the base
    of the model - they are kept and point down.

    A face with free space on BOTH sides (a free-standing wall, a canopy, a
    fence) or seen from outside on both sides (the back wall of an open
    shelter) has no single outside. It is marked in the face layer
    TWO_SIDED_LAYER; make_two_sided() later gives it a reversed twin, which is
    what the double-sided SketchUp original showed.
    """
    # Add the marker layer first: adding a layer later would invalidate every
    # BMFace reference collected below.
    layer = bm.faces.layers.int.get(TWO_SIDED_LAYER) or bm.faces.layers.int.new(TWO_SIDED_LAYER)
    bm.normal_update()
    bm.faces.ensure_lookup_table()
    bm.faces.index_update()

    xs = [v.co for v in bm.verts]
    lo = Vector((min(c.x for c in xs), min(c.y for c in xs), min(c.z for c in xs)))
    hi = Vector((max(c.x for c in xs), max(c.y for c in xs), max(c.z for c in xs)))
    size = (hi - lo).length or 1.0
    center = (lo + hi) * 0.5
    far = size * 10.0
    eps = max(size * 1e-6, 1e-6)
    ground_tol = max(size * 1e-5, 1e-4)
    ground = lo.z + ground_tol if on_ground else None

    def on_base(face):
        return (on_ground and abs(face.normal.z) > 0.999
                and all(abs(v.co.z - lo.z) < ground_tol for v in face.verts))

    bvh = BVHTree.FromBMesh(bm)
    dirs = _hemisphere_dirs(ray_count)

    sign = {}          # face index -> +1 keep / -1 flip
    enclosed = set()
    two_sided = set()
    open_min = max(3, int(0.10 * ray_count))
    for f in bm.faces:
        n = f.normal
        if n.length < 0.5:  # degenerate, let neighbours decide
            continue
        c = f.calc_center_median()
        front = _free_directions(bvh, c, n, dirs, far, eps, ground)
        back = _free_directions(bvh, c, -n, dirs, far, eps, ground)
        if front == 0 and back == 0:
            if on_base(f):  # bottom of the house: keep it, facing down
                sign[f.index] = 1 if n.z < 0 else -1
                continue
            enclosed.add(f.index)
            continue
        if min(front, back) >= open_min:
            two_sided.add(f.index)
        margin = max(2, int(0.25 * max(front, back)))
        if abs(front - back) >= margin:
            sign[f.index] = 1 if front > back else -1

    # A face looked enclosed with a few rays. It may still see the outside
    # through a narrow gap, so test the candidates again with many more rays.
    dense = _hemisphere_dirs(enclosed_ray_count)
    for idx in list(enclosed):
        f = bm.faces[idx]
        c = f.calc_center_median()
        n = f.normal
        front = _free_directions(bvh, c, n, dense, far, eps, ground)
        back = _free_directions(bvh, c, -n, dense, far, eps, ground)
        if front or back:
            enclosed.discard(idx)
            if front != back:
                sign[idx] = 1 if front > back else -1
    by_rays = len(sign)

    queue = deque(bm.faces[i] for i in sign)
    while queue:
        f = queue.popleft()
        s = sign[f.index]
        for e in f.edges:
            if len(e.link_faces) != 2:
                continue
            g = e.link_faces[0] if e.link_faces[1] is f else e.link_faces[1]
            if g.index in sign:
                continue
            sign[g.index] = s if e.is_contiguous else -s
            queue.append(g)
    by_neighbours = len(sign) - by_rays
    untouched = len(bm.faces) - len(sign)

    # Step 3: what does a camera outside see? (orientation after steps 1+2)
    seen = _outside_hits(bm, bvh, center, size,
                         _outside_views(view_count, upper_only=on_ground),
                         view_grid, ground)
    corrected_by_view = 0
    # Seen from outside on both sides (e.g. the back wall of an open bus
    # shelter): both sides are exterior, whatever the free-space counts say.
    for idx, (front_hits, back_hits) in seen.items():
        if front_hits >= 2 and back_hits >= 2:
            two_sided.add(idx)
    for idx, (front_hits, back_hits) in seen.items():
        if idx in two_sided:  # seen from both sides by design
            continue
        current = sign.get(idx, 1)
        # hits were counted with the original normal; translate to final
        f_ok, b_ok = (front_hits, back_hits) if current > 0 else (back_hits, front_hits)
        if b_ok > f_ok:
            sign[idx] = -current
            corrected_by_view += 1

    to_delete = []
    if remove_enclosed:
        to_delete = [bm.faces[i] for i in enclosed if i not in seen]
    delete_ids = {f.index for f in to_delete}
    flip = [f for f in bm.faces
            if sign.get(f.index, 1) < 0 and f.index not in delete_ids]

    for idx in two_sided:
        bm.faces[idx][layer] = 1
    if flip:
        bmesh.ops.reverse_faces(bm, faces=flip)
    if to_delete:
        bmesh.ops.delete(bm, geom=to_delete, context='FACES_ONLY')
        weld(bm, 0.0)  # drop loose verts / edges

    bm.normal_update()
    return {
        "flipped": len(flip),
        "by_rays": by_rays,
        "by_neighbours": by_neighbours,
        "untouched": untouched,
        "corrected_by_view": corrected_by_view,
        "enclosed": len(enclosed),
        "enclosed_removed": len(to_delete),
        "faces_visible_from_outside": len(seen),
        "two_sided_marked": len(two_sided),
    }


def make_two_sided(bm):
    """Give every face marked in TWO_SIDED_LAYER a reversed twin.

    The twin gets its own vertices, so no later weld can merge the pair (a
    weld would delete one face of it - the very bug this plugin started with).
    Returns the number of twins created.
    """
    layer = bm.faces.layers.int.get(TWO_SIDED_LAYER)
    if layer is None:
        return 0
    faces = [f for f in bm.faces if f[layer]]
    if faces:
        dup = bmesh.ops.duplicate(bm, geom=faces)
        twins = [g for g in dup["geom"] if isinstance(g, bmesh.types.BMFace)]
        bmesh.ops.reverse_faces(bm, faces=twins)
        for f in twins:
            f[layer] = 0
    bm.faces.layers.int.remove(layer)
    return len(faces)


def unite_collection(collection, merge_distance=1e-4, fix_normals=True,
                     remove_enclosed=True, on_ground=True, two_sided=True,
                     smooth_angle=math.radians(30.0), ray_count=48):
    """Create one mesh object from all meshes in `collection`.

    The source objects are not modified. Returns (new_object, stats).
    """
    sources = [o for o in collection.all_objects
               if o.type == 'MESH' and not o.get(RESULT_TAG)]
    if not sources:
        return None, {"error": "no mesh objects"}

    bm, materials, skipped = build_bmesh(sources)
    stats = {
        "source_objects": len(sources),
        "faces_in": len(bm.faces),
        "verts_in": len(bm.verts),
        "invalid_faces_skipped": skipped,
    }

    removed, back = remove_back_faces(bm, merge_distance)
    stats["back_faces_removed"] = removed
    stats["back_material"] = (materials[back].name
                              if back is not None and materials[back] else None)
    stats["verts_welded"] = weld(bm, merge_distance)

    stats["degenerate_removed"] = remove_degenerate(bm, merge_distance)

    if fix_normals and bm.faces:
        stats.update(orient_outward(bm, ray_count, remove_enclosed, on_ground))

    stats["degenerate_removed"] += remove_degenerate(bm, merge_distance)

    # Last geometry step: nothing may weld after the twins exist.
    if two_sided:
        stats["two_sided_faces"] = make_two_sided(bm)
    else:
        layer = bm.faces.layers.int.get(TWO_SIDED_LAYER)
        if layer is not None:
            bm.faces.layers.int.remove(layer)

    # Keep only materials still in use, in a compact order.
    used = sorted({f.material_index for f in bm.faces})
    remap = {old: new for new, old in enumerate(used)}
    for f in bm.faces:
        f.material_index = remap[f.material_index]

    name = collection.name.rsplit(".dae", 1)[0] + "_united"
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    for old in used:
        me.materials.append(materials[old])

    if smooth_angle is not None:
        me.shade_smooth()
        me.set_sharp_from_angle(angle=smooth_angle)
    me.validate()
    me.update()

    obj = bpy.data.objects.new(name, me)
    obj[RESULT_TAG] = True
    collection.objects.link(obj)

    stats["faces_out"] = len(me.polygons)
    stats["verts_out"] = len(me.vertices)
    stats["materials_out"] = len(me.materials)
    return obj, stats


# --------------------------------------------------------------------------
# Blender UI
# --------------------------------------------------------------------------

def _world_bbox(obj):
    """(min, max) of the object's bounding box in world space."""
    pts = [obj.matrix_world @ Vector(c) for c in obj.bound_box]
    lo = Vector((min(p.x for p in pts), min(p.y for p in pts), min(p.z for p in pts)))
    hi = Vector((max(p.x for p in pts), max(p.y for p in pts), max(p.z for p in pts)))
    return lo, hi


def _boxes_overlap(a_lo, a_hi, b_lo, b_hi):
    return (a_lo.x < b_hi.x and b_lo.x < a_hi.x and
            a_lo.y < b_hi.y and b_lo.y < a_hi.y)


def place_beside(obj, sources, gap, scene):
    """Move `obj` along +X next to its source model, onto free space.

    The step is the model width plus `gap`; it keeps stepping while the new
    position would overlap another visible mesh in the scene (footprint check).
    """
    src_boxes = [_world_bbox(o) for o in sources]
    lo = Vector((min(b[0].x for b in src_boxes), min(b[0].y for b in src_boxes), 0.0))
    hi = Vector((max(b[1].x for b in src_boxes), max(b[1].y for b in src_boxes), 0.0))
    step = (hi.x - lo.x) + gap
    obstacles = [_world_bbox(o) for o in scene.objects
                 if o.type == 'MESH' and o is not obj and o.visible_get()]
    shift = step
    for _ in range(50):
        new_lo = lo + Vector((shift, 0.0, 0.0))
        new_hi = hi + Vector((shift, 0.0, 0.0))
        if not any(_boxes_overlap(new_lo, new_hi, b_lo, b_hi) for b_lo, b_hi in obstacles):
            break
        shift += step
    obj.location.x += shift
    return shift


def _target_collections(context, scope):
    if scope == 'ACTIVE':
        coll = context.view_layer.active_layer_collection.collection
        if coll is context.scene.collection:
            return []
        return [coll]
    # ALL: every top-level collection with meshes (one import = one collection)
    return [c for c in context.scene.collection.children
            if any(o.type == 'MESH' and not o.get(RESULT_TAG) for o in c.all_objects)]


class UNITE_OT_collection(bpy.types.Operator):
    """Join all meshes of a collection into one object, remove double-sided back faces and fix normals"""
    bl_idname = "unite_model.collection"
    bl_label = "Unite Collection"
    bl_options = {'REGISTER', 'UNDO'}

    scope: EnumProperty(
        name="Collections",
        items=[
            ('ACTIVE', "Active", "Only the active collection in the Outliner"),
            ('ALL', "All", "Every top-level collection that contains meshes"),
        ],
        default='ACTIVE',
    )
    merge_distance: FloatProperty(
        name="Merge Distance", default=0.0001, min=0.0, soft_max=0.01,
        precision=5, unit='LENGTH',
        description="Vertices closer than this are welded. Keep it well below the shortest edge",
    )
    fix_normals: BoolProperty(
        name="Normals Outward", default=True,
        description="Orient every face so it points to the outside of the model",
    )
    remove_enclosed: BoolProperty(
        name="Remove Hidden Inner Faces", default=True,
        description="Delete faces that are completely hidden inside the model "
                    "(never deleted when visible from outside)",
    )
    two_sided: BoolProperty(
        name="Free-Standing Faces Two-Sided", default=True,
        description="Faces open to the outside on both sides (free-standing walls, "
                    "canopies, fences) get a reversed copy so they look right from both sides",
    )
    on_ground: BoolProperty(
        name="Stands on Ground", default=True,
        description="The model stands on terrain at its lowest point: it is never "
                    "seen from below and its base faces point down",
    )
    smooth_angle: FloatProperty(
        name="Auto Smooth Angle", default=math.radians(30.0),
        min=0.0, max=math.pi, subtype='ANGLE',
        description="Edges sharper than this stay sharp",
    )
    source_action: EnumProperty(
        name="Source Objects",
        items=[
            ('HIDE', "Hide", "Keep the original objects but hide them"),
            ('KEEP', "Keep", "Leave the original objects visible"),
            ('DELETE', "Delete", "Delete the original objects and their empties"),
        ],
        default='KEEP',
    )
    place_beside: BoolProperty(
        name="Place Beside Original", default=True,
        description="Move the result next to the original model so both can be compared",
    )
    gap: FloatProperty(
        name="Gap", default=2.0, min=0.0, unit='LENGTH',
        description="Free space between the original and the result",
    )

    def execute(self, context):
        if context.mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')
        targets = _target_collections(context, self.scope)
        if not targets:
            self.report({'WARNING'}, "No collection with meshes selected")
            return {'CANCELLED'}

        results = []
        for coll in targets:
            obj, stats = unite_collection(
                coll,
                merge_distance=self.merge_distance,
                fix_normals=self.fix_normals,
                remove_enclosed=self.remove_enclosed,
                on_ground=self.on_ground,
                two_sided=self.two_sided,
                smooth_angle=self.smooth_angle,
            )
            if obj is None:
                continue
            others = [o for o in coll.all_objects if o is not obj and not o.get(RESULT_TAG)]
            if self.place_beside and self.source_action != 'DELETE':
                meshes = [o for o in others if o.type == 'MESH']
                stats["offset_x"] = round(place_beside(obj, meshes, self.gap, context.scene), 4)
            if self.source_action == 'HIDE':
                for o in others:
                    o.hide_set(True)
                    o.hide_render = True
            elif self.source_action == 'DELETE':
                for o in others:
                    bpy.data.objects.remove(o, do_unlink=True)
            for key, value in stats.items():
                if isinstance(value, (int, float, str)):
                    obj["unite_" + key] = value
            results.append((coll.name, stats))

        for o in context.selected_objects:
            o.select_set(False)
        last = None
        for coll in targets:
            for o in coll.objects:
                if o.get(RESULT_TAG):
                    o.select_set(True)
                    last = o
        if last:
            context.view_layer.objects.active = last

        msg = "; ".join(
            f"{name}: {s['source_objects']} obj -> 1, faces {s['faces_in']} -> {s['faces_out']}, "
            f"flipped {s.get('flipped', 0)}"
            for name, s in results
        )
        self.report({'INFO'}, msg or "Nothing to do")
        return {'FINISHED'}


class UNITE_PT_panel(bpy.types.Panel):
    bl_label = "Unite Model"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Unite"

    def draw(self, context):
        layout = self.layout
        coll = context.view_layer.active_layer_collection.collection
        layout.label(text=f"Active: {coll.name}", icon='OUTLINER_COLLECTION')
        op = layout.operator(UNITE_OT_collection.bl_idname, text="Unite Active Collection")
        op.scope = 'ACTIVE'
        op = layout.operator(UNITE_OT_collection.bl_idname, text="Unite All Collections")
        op.scope = 'ALL'


classes = (UNITE_OT_collection, UNITE_PT_panel)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
