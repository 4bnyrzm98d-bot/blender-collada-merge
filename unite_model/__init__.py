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
4. Files often contain geometry that is not part of the house: map
   underlays, photo backdrops, stray parts far away and neighbour houses.
   These are found per connected part and moved to a hidden "_removed"
   object.

The panel (3D View > Sidebar > Unite) imports .dae files with the Collada
Support importer, repairs them and exports the results as glTF 2.0 (.glb).
"""

import csv
import glob
import math
import os
import time
from collections import Counter, defaultdict, deque

import bmesh
import bpy
from bpy.props import (BoolProperty, CollectionProperty, EnumProperty,
                       FloatProperty, PointerProperty, StringProperty)
from bpy_extras.io_utils import ImportHelper
from mathutils import Matrix, Vector, kdtree
from mathutils.bvhtree import BVHTree

RESULT_TAG = "unite_model_result"
REMOVED_TAG = "unite_model_removed"
TWO_SIDED_LAYER = "unite_two_sided"
FLOOR_LAYER = "unite_filled_floor"


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


def _outside_hits(bm, bvh, center, size, views, grid, ground=None,
                  probe=0.02, samples=12):
    """Shoot parallel rays at the model from every view direction.

    Returns {face_index: [front_hits, back_hits]} for the first face each ray
    meets, i.e. what a camera outside the model can actually see. With
    `ground`, a ray coming from below may only see points above the terrain.

    SketchUp walls often leave hairline cracks, and out of ~1.6 M rays a few
    slip through and "see" faces deep inside. So for every face side up to
    `samples` hits are re-shot with four rays shifted by `probe` (2 cm)
    sideways. Through a real opening they land next to the first hit; through
    a crack they stop at the wall. The counts are scaled by the share of
    sampled hits that pass this test.
    """
    hits = defaultdict(lambda: [0, 0])
    kept = defaultdict(lambda: ([], []))
    for d in views:
        cam = center + d * size * 2.0
        u = d.orthogonal().normalized()
        w = d.cross(u).normalized()
        step = 1.0 / grid
        for a in range(grid):
            ua = u * ((a + 0.5) * step - 0.5) * size
            for b in range(grid):
                origin = cam + ua + w * (((b + 0.5) * step - 0.5) * size)
                loc, _n, idx, dist = bvh.ray_cast(origin, -d, size * 4.0)
                if idx is None:
                    continue
                if ground is not None and d.z < 0.0 and loc.z <= ground:
                    continue  # this point would be seen from underground
                side = 0 if bm.faces[idx].normal.dot(d) >= 0.0 else 1
                hits[idx][side] += 1
                bucket = kept[idx][side]
                if len(bucket) < samples:
                    bucket.append((origin, d, u, w, dist))

    def solid(origin, d, u, w, dist):
        ok = 0
        for off in (u * probe, -u * probe, w * probe, -w * probe):
            hit = bvh.ray_cast(origin + off, -d, size * 4.0)
            if hit[0] is not None and abs(hit[3] - dist) < 0.25:
                ok += 1
        return ok >= 3

    result = {}
    for idx, counts in hits.items():
        scaled = []
        for side in (0, 1):
            bucket = kept[idx][side]
            if not bucket:
                scaled.append(0)
                continue
            share = sum(solid(*s) for s in bucket) / len(bucket)
            scaled.append(int(round(counts[side] * share)))
        if scaled[0] or scaled[1]:
            result[idx] = scaled
    return result


def orient_outward(bm, ray_count=48, remove_enclosed=True, on_ground=True,
                   view_count=64, view_grid=160, enclosed_ray_count=1024,
                   two_sided_ratio=0.25, keep_small=0.05):
    """Flip faces so their normal points to the outside of the model.

    Step 1: per face, compare free space in front of and behind it (rays
            into both hemispheres). The freer side is the outside.
    Step 2: faces step 1 cannot decide copy the orientation of a decided
            neighbour across a clean (2-face) edge.
    Step 3: safety net - the model is "photographed" from many directions;
            any face seen from its back is flipped.
    Faces with no free space on either side are enclosed (hidden inside the
    model). With remove_enclosed they are deleted, unless step 3 saw them
    (views through hairline cracks do not count, see _outside_hits).

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

    base_tol = max(ground_tol, 0.02)  # floors a few mm above the terrain too

    def on_base(face):
        return (on_ground and abs(face.normal.z) > 0.999
                and all(abs(v.co.z - lo.z) < base_tol for v in face.verts))

    bvh = BVHTree.FromBMesh(bm)
    dirs = _hemisphere_dirs(ray_count)

    sign = {}          # face index -> +1 keep / -1 flip
    enclosed = set()
    closed_base = set()  # base faces with no free space above: point down
    two_sided = set()
    open_back = set()
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
                closed_base.add(f.index)
                continue
            enclosed.add(f.index)
            continue
        # Two-sided only when both sides are comparably open (a free-standing
        # wall, a fence, a canopy). A roof over an attic that is open at the
        # gable sees a little sky from below too, but its outside is clear:
        # one face, pointing out (a twin there is a redundant second layer).
        if min(front, back) >= open_min:
            if min(front, back) >= two_sided_ratio * max(front, back):
                two_sided.add(f.index)
            else:
                open_back.add(f.index)  # outside on both sides, one mostly
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
    inner_twin = set()
    # Seen from outside on both sides (e.g. the back wall of an open bus
    # shelter): both sides are exterior, whatever the free-space counts say.
    for idx, (front_hits, back_hits) in seen.items():
        if (min(front_hits, back_hits) >= 2
                and min(front_hits, back_hits) >= two_sided_ratio * max(front_hits, back_hits)):
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
            f_ok, b_ok = b_ok, f_ok
        if b_ok >= 2:
            # the back is seen too, only a little: through a niche, an opening,
            # over an eave. It gets a reversed twin set back behind it.
            inner_twin.add(idx)

    to_delete = []
    if remove_enclosed:
        # tiny enclosed faces stay: deleting them costs nothing visible inside,
        # but where the test was wrong (a nook seen through a narrow gap) a
        # deleted 5-500 cm2 face is a pinhole into the house
        to_delete = [bm.faces[i] for i in enclosed
                     if i not in seen and bm.faces[i].calc_area() >= keep_small]
    delete_ids = {f.index for f in to_delete}
    flip = [f for f in bm.faces
            if sign.get(f.index, 1) < 0 and f.index not in delete_ids]

    for idx in (inner_twin | open_back) - two_sided:
        if idx in delete_ids:
            continue
        f = bm.faces[idx]
        f[layer] = 2
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
        "inner_twins_marked": len((inner_twin | open_back) - two_sided),
    }


def make_two_sided(bm, inner_offset=0.01):
    """Give every face marked in TWO_SIDED_LAYER a reversed back face.

    Value 1 (open on both sides alike: a free-standing wall, a canopy): the
    back face lies exactly on the original; with backface culling - glTF's
    default - each side shows exactly one face.
    Value 2 (seen mostly from the front, its back only a little: through a
    niche, an opening, over an eave): the back face is set back
    `inner_offset` (1 cm) and along free edges a narrow rim joins the two,
    like Solidify. From outside the original is seen, from the niche the back
    face, over a free edge the rim - no coincident faces, nothing flickers.
    The back faces get their own vertices, so no later weld can merge them.
    Returns the number of faces made two-sided.
    """
    layer = bm.faces.layers.int.get(TWO_SIDED_LAYER)
    if layer is None:
        return 0
    faces = [f for f in bm.faces if f[layer]]
    for value in (1, 2):
        group = [f for f in faces if f[layer] == value]
        if not group:
            continue
        normals = {f: f.normal.copy() for f in group}
        free = [e for e in {e for f in group for e in f.edges} if len(e.link_faces) == 1] if value == 2 else []
        dup = bmesh.ops.duplicate(bm, geom=group)
        face_map = dup["face_map"]
        vmap = {}
        for f in group:
            for v, tv in zip(f.verts, face_map[f].verts):
                vmap[v] = tv
        if value == 2:
            shift = defaultdict(Vector)
            for f in group:
                for v in face_map[f].verts:
                    shift[v] -= normals[f]
            for v, n in shift.items():
                if n.length > 1e-9:
                    v.co += n.normalized() * inner_offset
        twins = [face_map[f] for f in group]
        bmesh.ops.reverse_faces(bm, faces=twins)
        for f in twins:
            f[layer] = 0
        for e in free:
            f = e.link_faces[0]
            v1, v2 = e.verts
            t1, t2 = vmap.get(v1), vmap.get(v2)
            if t1 is None or t2 is None:
                continue
            try:
                rim = bm.faces.new((v1, v2, t2, t1))
            except ValueError:
                continue
            rim.material_index = f.material_index
            rim.normal_update()
            if rim.normal.dot((v1.co + v2.co) * 0.5 - f.calc_center_median()) < 0.0:
                rim.normal_flip()
    bm.faces.layers.int.remove(layer)
    return len(faces)



def close_gaps(bm, max_gap=0.4, merge_distance=1e-4, min_edge=0.02):
    """Close slits the SketchUp model leaves open, e.g. between the top of a
    facade and the cornice above it.

    SketchUp draws both sides of every face, so a slit only shows the inner
    side of the walls behind it; with one-sided faces it becomes a see-through
    line or a strip of back faces. For every open edge (one face only) we look
    further in the plane of its face: when, along the whole edge, another face
    lying across that direction is reached within `max_gap`, the strip between
    them is filled with a new face (same material). Window-sized openings are
    larger than `max_gap` and stay open. Returns (faces added, area added).
    """
    bm.faces.ensure_lookup_table()
    bm.normal_update()
    bvh = BVHTree.FromBMesh(bm)
    faces = list(bm.faces)  # BVH indices; adding faces invalidates the table
    added = []
    for e in [e for e in bm.edges if len(e.link_faces) == 1]:
        if e.calc_length() < min_edge:
            continue
        f = e.link_faces[0]
        n = f.normal
        if n.length < 0.5:
            continue
        v1, v2 = e.verts
        ev = v2.co - v1.co
        out = n.cross(ev)
        if out.length < 1e-9:
            continue
        out.normalize()
        mid = (v1.co + v2.co) * 0.5
        if (mid - f.calc_center_median()).dot(out) < 0.0:
            out = -out
        shared = {v1, v2}
        dists = []
        for s in (0.1, 0.5, 0.9):
            p = v1.co + ev * s
            loc, _nrm, idx, dist = bvh.ray_cast(p + out * 1e-4, out, max_gap)
            if idx is None:
                break
            g = faces[idx]
            if g is f or abs(g.normal.dot(out)) < 0.5 or shared & set(g.verts):
                break
            dists.append(dist + 1e-4)
        if len(dists) < 3:
            continue
        if max(dists) - min(dists) > 0.02 + 0.05 * max(dists):
            continue  # not a straight slit along the whole edge
        # extrapolate the gap height to the edge ends (samples at 10 % / 90 %)
        t0 = dists[0] + (dists[0] - dists[2]) / 8.0
        t1 = dists[2] + (dists[2] - dists[0]) / 8.0
        if min(t0, t1) <= merge_distance:
            continue
        a = bm.verts.new(v1.co + out * t0)
        b = bm.verts.new(v2.co + out * t1)
        try:
            face = bm.faces.new((v1, v2, b, a))
        except ValueError:
            continue
        face.material_index = f.material_index
        added.append(face)
    if not added:
        return 0, 0.0
    area = sum(f.calc_area() for f in added)
    count = len(added)
    weld(bm, max(merge_distance, 1e-4))  # join neighbouring strips and their ends
    bm.normal_update()
    return count, area


def _plane_fit(points):
    """Centre and normal of the best plane through `points` (Newell)."""
    c = sum(points, Vector()) / len(points)
    n = Vector()
    for a, b in zip(points, points[1:] + points[:1]):
        n.x += (a.y - b.y) * (a.z + b.z)
        n.y += (a.z - b.z) * (a.x + b.x)
        n.z += (a.x - b.x) * (a.y + b.y)
    return c, (n.normalized() if n.length > 1e-12 else n)


def fill_missing_faces(bm, plane_tol=0.05, min_area=0.05, max_area=500.0, max_path=12,
                       min_depth=1.0):
    """Fill holes left by a missing face of a closed shape: the gable triangle
    of a roof, the floor of a house, the top of a chimney.

    A hole is a ring of open edges (one face only); it may also be closed by a
    few existing edges in its plane (a gable is open along both roof slopes but
    ends on the wall top). It is filled when the ring is planar and the faces
    around it run ACROSS that plane - the walls around a missing floor, the
    roof slopes around a missing gable - and reach at least `min_depth` away
    from it. A window cut into a wall or the outline of a free-standing panel
    is surrounded by faces IN its plane, a recessed window by a shallow reveal;
    both stay open. Returns (faces added, area added).
    """
    bm.normal_update()
    floor_layer = bm.faces.layers.int.get(FLOOR_LAYER) or bm.faces.layers.int.new(FLOOR_LAYER)
    zmin = min((v.co.z for v in bm.verts), default=0.0)
    open_edges = {e for e in bm.edges if len(e.link_faces) == 1}
    adj = defaultdict(list)
    for e in open_edges:
        for v in e.verts:
            adj[v].append(e)
    seen, added, area = set(), 0, 0.0
    for e0 in list(open_edges):
        if e0 in seen:
            continue
        comp, stack = [], [e0]
        seen.add(e0)
        while stack:
            e = stack.pop()
            comp.append(e)
            for v in e.verts:
                for e2 in adj[v]:
                    if e2 not in seen:
                        seen.add(e2)
                        stack.append(e2)
        verts = {v for e in comp for v in e.verts}
        ends = [v for v in verts if len(adj[v]) == 1]
        if any(len(adj[v]) > 2 for v in verts) or len(ends) not in (0, 2):
            continue  # branching outline: not a simple hole
        # order the ring / chain
        start = ends[0] if ends else next(iter(verts))
        order, prev_e, v = [start], None, start
        while True:
            nxt = [e for e in adj[v] if e is not prev_e]
            if not nxt:
                break
            prev_e = nxt[0]
            v = prev_e.other_vert(v)
            if v is start:
                break
            order.append(v)
        if len(order) != len(verts):
            continue
        c, n = _plane_fit([v.co for v in order])
        if n.length < 0.5:
            continue
        if ends:  # close the chain with existing edges lying in the plane
            target = order[-1]
            best = {order[0]: (0.0, None)}
            frontier = [order[0]]
            for _ in range(max_path):
                nf = []
                for u in frontier:
                    for e in u.link_edges:
                        if e in open_edges:
                            continue
                        w = e.other_vert(u)
                        if abs((w.co - c).dot(n)) > plane_tol:
                            continue
                        d = best[u][0] + e.calc_length()
                        if w not in best or d < best[w][0]:
                            best[w] = (d, u)
                            nf.append(w)
                frontier = nf
            if target not in best:
                continue
            if best[target][0] > sum(e.calc_length() for e in comp):
                continue  # mostly closed by existing edges: not a missing face
            path, w = [], best[target][1]
            while w is not None and w is not order[0]:
                path.append(w)
                w = best[w][1]
            ring = order + path
        else:
            ring = order
        if len(set(ring)) != len(ring) or len(ring) < 3:
            continue
        if any(abs((v.co - c).dot(n)) > plane_tol for v in ring):
            continue
        # the faces around must run across the hole's plane
        across_faces = [e.link_faces[0] for e in comp
                        if abs(e.link_faces[0].normal.dot(n)) < 0.9]
        across = sum(e.calc_length() for e in comp
                     if abs(e.link_faces[0].normal.dot(n)) < 0.9)
        if across < 0.5 * sum(e.calc_length() for e in comp):
            continue
        depth = max(abs((v.co - c).dot(n)) for f in across_faces for v in f.verts)
        if depth < min_depth:
            continue  # a shallow reveal: a window or a niche, not a missing face
        try:
            f = bm.faces.new(ring)
        except ValueError:
            continue
        a = f.calc_area()
        if not (min_area <= a <= max_area) or any(len(e.link_faces) > 3 for e in f.edges):
            bm.faces.remove(f)
            continue
        f.material_index = comp[0].link_faces[0].material_index
        if abs(n.z) > 0.999 and all(v.co.z - zmin < 0.02 for v in ring):
            f[floor_layer] = 1  # a missing floor: the bottom of the house
        added += 1
        area += a
    bm.normal_update()
    return added, area


# --------------------------------------------------------------------------
# Unrelated geometry: map underlays, photo backdrops, stray parts, neighbours
# --------------------------------------------------------------------------

def _edge_islands(bm):
    """Edge-connected face islands as lists of face indices."""
    bm.faces.index_update()
    seen = set()
    islands = []
    for f in bm.faces:
        if f.index in seen:
            continue
        seen.add(f.index)
        stack, cur = [f], []
        while stack:
            g = stack.pop()
            cur.append(g.index)
            for e in g.edges:
                for h in e.link_faces:
                    if h.index not in seen:
                        seen.add(h.index)
                        stack.append(h)
        islands.append(cur)
    return islands


def _bvh_subset(bm, face_ids, epsilon=0.0):
    """BVH of a subset of faces; returns (bvh, local polygon -> face index)."""
    vmap, verts, polys, back = {}, [], [], []
    for fi in face_ids:
        poly = []
        for v in bm.faces[fi].verts:
            if v.index not in vmap:
                vmap[v.index] = len(verts)
                verts.append(v.co.copy())
            poly.append(vmap[v.index])
        polys.append(poly)
        back.append(fi)
    return BVHTree.FromPolygons(verts, polys, epsilon=epsilon), back


def _bbox(bm, face_ids):
    pts = [v.co for fi in face_ids for v in bm.faces[fi].verts]
    lo = Vector((min(p.x for p in pts), min(p.y for p in pts), min(p.z for p in pts)))
    hi = Vector((max(p.x for p in pts), max(p.y for p in pts), max(p.z for p in pts)))
    return lo, hi


def _area(bm, face_ids):
    return sum(bm.faces[fi].calc_area() for fi in face_ids)


def _is_sheet(bm, face_ids, min_area, max_faces=8):
    """A flat panel of a few faces: SketchUp image, map underlay, backdrop."""
    if len(face_ids) > max_faces or _area(bm, face_ids) < min_area:
        return False
    f0 = bm.faces[face_ids[0]]
    n0, p0 = f0.normal, f0.verts[0].co
    for fi in face_ids:
        f = bm.faces[fi]
        if abs(abs(f.normal.dot(n0)) - 1.0) > 1e-3:
            return False
        if any(abs((v.co - p0).dot(n0)) > 1e-3 for v in f.verts):
            return False
    return True


def _min_distance(bvh, bm, face_ids, samples=300):
    """Smallest distance from the vertices of `face_ids` to the BVH."""
    pts = list({v.index: v.co for fi in face_ids for v in bm.faces[fi].verts}.values())
    step = max(1, len(pts) // samples)
    best = float("inf")
    for p in pts[::step]:
        hit = bvh.find_nearest(p)
        if hit[0] is not None:
            best = min(best, hit[3])
    # a centre sample catches big faces whose corners are far from everything
    if len(face_ids) <= 8:
        for fi in face_ids:
            hit = bvh.find_nearest(bm.faces[fi].calc_center_median())
            if hit[0] is not None:
                best = min(best, hit[3])
    return best


def _is_building(bm, face_ids):
    lo, hi = _bbox(bm, face_ids)
    sx, sy, sz = hi - lo
    return (len(face_ids) >= 100 and max(sx, sy) >= 3.0
            and min(sx, sy) >= 2.0 and sz >= 2.5)


def _uncovered_ground(bm, face_ids, level_tol=0.1, min_area=1.0, min_free=0.6):
    """Ground-level pieces attached to the house but lying outside it.

    SketchUp models often carry a strip of pavement or a driveway that shares
    an edge with the house base, so it is in the same island and the island
    tests keep it. Such a face is horizontal, lies at the lowest level of the
    model, nothing of the model is above it and most of its outline is open.
    A base slab, a floor or the ground under a canopy is covered from above,
    a courtyard floor is bordered by walls; both stay. Returns lists of
    face indices, one per edge-connected piece.
    """
    if not face_ids:
        return []
    zmin = min(v.co.z for fi in face_ids for v in bm.faces[fi].verts)
    ground = {fi for fi in face_ids
              if abs(bm.faces[fi].normal.z) > 0.999
              and all(v.co.z - zmin < level_tol for v in bm.faces[fi].verts)}
    if not ground:
        return []
    bvh, back = _bvh_subset(bm, face_ids)
    up = Vector((0.0, 0.0, 1.0))
    lift = Vector((0.0, 0.0, 0.01))

    def covered(p):
        origin = p + lift
        while True:
            hit = bvh.ray_cast(origin, up)
            if hit[0] is None:
                return False
            if back[hit[2]] not in ground or hit[3] > 0.05:
                return True
            origin = hit[0] + lift  # another ground face at the same level

    open_faces = set()
    for fi in ground:
        f = bm.faces[fi]
        c = f.calc_center_median()
        pts = [c] + [c + (v.co - c) * 0.8 for v in f.verts]
        if sum(not covered(p) for p in pts) >= len(pts) - 1:  # tolerate one sample
            open_faces.add(fi)
    pieces, seen = [], set()
    for fi in open_faces:
        if fi in seen:
            continue
        seen.add(fi)
        stack, cur = [fi], []
        while stack:
            g = stack.pop()
            cur.append(g)
            for e in bm.faces[g].edges:
                for h in e.link_faces:
                    if h.index in open_faces and h.index not in seen:
                        seen.add(h.index)
                        stack.append(h.index)
        if _area(bm, cur) >= min_area and _free_boundary_share(bm, cur) >= min_free:
            pieces.append(cur)
    return pieces


def _free_boundary_share(bm, face_ids):
    """Share of the piece's outline that is a free (open) edge.

    A pavement strip hangs on the house by one side, the rest of its outline
    is open. The floor of a courtyard meets the walls along its whole
    outline and is part of the house."""
    inside = set(face_ids)
    free = total = 0.0
    for fi in face_ids:
        for e in bm.faces[fi].edges:
            outside = [f for f in e.link_faces if f.index not in inside]
            if len(e.link_faces) > 1 and not outside:
                continue  # inner edge of the piece
            length = e.calc_length()
            total += length
            if not outside:
                free += length
    return free / total if total > 0.0 else 0.0


def find_unrelated(bm, cameras=(), keep_distance=1.0, sheet_min_area=25.0,
                   keep_buildings='MAIN', cluster_gap=0.05, remove_ground=True):
    """Decide which faces do not belong to the house.

    1. Sheets (flat panels of <= 8 faces, >= sheet_min_area) are taken out
       first, per edge island: a map underlay or photo backdrop can touch the
       house and would otherwise merge with it.
    2. Other islands closer than `cluster_gap` form clusters.
    3. The main building: with several building-like clusters, the one the
       saved SketchUp camera looks at (files may contain neighbour houses),
       otherwise the cluster with most faces.
    4. Kept: the main building (or every building with keep_buildings='ALL')
       plus clusters within `keep_distance` of what is kept, repeated so a
       chain of nearby parts stays. Sheets lying inside the house envelope
       (flat roof, base slab) count as ordinary clusters.
    5. Horizontal faces at the lowest level that nothing of the house covers
       (pavement or driveway strips attached to the base) - remove_ground.
    Returns (set of face indices to remove, info dict).
    """
    bm.faces.ensure_lookup_table()
    bm.normal_update()
    islands = _edge_islands(bm)
    sheets = [isl for isl in islands if _is_sheet(bm, isl, sheet_min_area)]
    sheet_set = {fi for isl in sheets for fi in isl}
    rest = [isl for isl in islands if isl[0] not in sheet_set]
    info = {"islands": len(islands), "sheets_found": len(sheets),
            "main_by": "single", "buildings": 0, "parts": []}
    if not rest:
        return set(), info

    # proximity clusters of the non-sheet islands
    island_of = {}
    for k, isl in enumerate(rest):
        for fi in isl:
            island_of[fi] = k
    parent = list(range(len(rest)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    all_rest = [fi for isl in rest for fi in isl]
    fat, back = _bvh_subset(bm, all_rest, epsilon=cluster_gap)
    for a, b in fat.overlap(fat):
        ra, rb = find(island_of[back[a]]), find(island_of[back[b]])
        if ra != rb:
            parent[ra] = rb
    groups = defaultdict(list)
    for k, isl in enumerate(rest):
        groups[find(k)].extend(isl)
    clusters = sorted(groups.values(), key=len, reverse=True)

    buildings = [c for c in clusters if _is_building(bm, c)]
    info["buildings"] = len(buildings)
    if len(buildings) >= 2 and cameras:
        origin, forward = cameras[0]

        def angle(c):
            lo, hi = _bbox(bm, c)
            to = (lo + hi) * 0.5 - origin
            return forward.angle(to) if to.length > 1e-6 else 0.0
        main = min(buildings, key=angle)
        info["main_by"] = "camera"
    elif buildings:
        main = buildings[0]
        info["main_by"] = "faces" if len(buildings) >= 2 else "single"
    else:
        main = clusters[0]

    kept = [main]
    if keep_buildings == 'ALL':
        kept += [c for c in buildings if c is not main]
    main_lo, main_hi = _bbox(bm, main)
    margin = Vector((0.5, 0.5, 0.5))
    candidates = [c for c in clusters if all(c is not k for k in kept)]
    removed_parts = []
    for isl in sheets:
        lo, hi = _bbox(bm, isl)
        inside = all(main_lo[i] - margin[i] <= lo[i] and hi[i] <= main_hi[i] + margin[i]
                     for i in range(3))
        if inside:
            candidates.append(isl)
        else:
            removed_parts.append(("sheet", isl))

    for _ in range(4):  # grow the kept set by nearby parts
        kept_faces = [fi for c in kept for fi in c]
        kbvh, _back = _bvh_subset(bm, kept_faces)
        near = [c for c in candidates if _min_distance(kbvh, bm, c) <= keep_distance]
        if not near:
            break
        kept += near
        candidates = [c for c in candidates if all(c is not n for n in near)]

    kept_faces = [fi for c in kept for fi in c]
    for c in candidates:
        kind = "building" if any(c is b for b in buildings) else "far"
        removed_parts.append((kind, c))
    if remove_ground:
        ground = _uncovered_ground(bm, kept_faces)
        removed_parts += [("ground", g) for g in ground]
        gone = {fi for g in ground for fi in g}
        kept_faces = [fi for fi in kept_faces if fi not in gone]
    kbvh, _back = _bvh_subset(bm, kept_faces)

    remove = set()
    for kind, c in removed_parts:
        lo, hi = _bbox(bm, c)
        info["parts"].append({
            "kind": kind, "faces": len(c), "area": round(_area(bm, c), 2),
            "size": [round(x, 2) for x in hi - lo],
            "distance": round(_min_distance(kbvh, bm, c), 2)})
        remove.update(c)
    return remove, info


def _describe_parts(parts, limit=6):
    """Short text for reports: 'sheet 3522m2 @24.8m; far 3.5m2 @10.1m'."""
    txt = [f"{p['kind']} {p['area']:g}m2 @{p['distance']:g}m"
           for p in sorted(parts, key=lambda p: -p["area"])[:limit]]
    if len(parts) > limit:
        txt.append(f"+{len(parts) - limit} more")
    return "; ".join(txt)


def _is_generated(obj):
    return bool(obj.get(RESULT_TAG) or obj.get(REMOVED_TAG))


def _collection_cameras(collection):
    """(origin, forward) of cameras imported with the model (SketchUp view)."""
    cams = []
    for o in collection.all_objects:
        if o.type == 'CAMERA':
            mw = o.matrix_world
            forward = (mw.to_3x3() @ Vector((0.0, 0.0, -1.0))).normalized()
            cams.append((mw.translation.copy(), forward))
    return cams


def _exact_area(face):
    """Face area in double precision (bmesh works in float32, so the three
    points of a collinear triangle far from the origin get a tiny area)."""
    pts = [tuple(float(c) for c in v.co) for v in face.verts]
    nx = ny = nz = 0.0
    for (x1, y1, z1), (x2, y2, z2) in zip(pts, pts[1:] + pts[:1]):
        nx += (y1 - y2) * (z1 + z2)
        ny += (z1 - z2) * (x1 + x2)
        nz += (x1 - x2) * (y1 + y2)
    return 0.5 * (nx * nx + ny * ny + nz * nz) ** 0.5


def set_area_weighted_normals(me, smooth_angle):
    """Shading normals weighted by face area (like the Weighted Normal
    modifier). With plain averaging a large flat roof that shares a vertex
    with a few small faces of a curved gable gets a tilted normal there and
    shows a dark smear across the whole roof. Weighted by area, a big face
    keeps its own normal, while curves made of similar small faces stay
    smooth. Faces meeting at more than `smooth_angle` stay sharp.
    """
    cos_limit = math.cos(smooth_angle)
    normals = [p.normal.copy() for p in me.polygons]
    areas = [p.area for p in me.polygons]
    vert_faces = defaultdict(list)
    for p in me.polygons:
        for v in p.vertices:
            vert_faces[v].append(p.index)
    loop_normals = [None] * len(me.loops)
    for p in me.polygons:
        n0 = normals[p.index]
        for li in p.loop_indices:
            v = me.loops[li].vertex_index
            acc = Vector()
            for g in vert_faces[v]:
                if normals[g].dot(n0) >= cos_limit:
                    acc += normals[g] * areas[g]
            loop_normals[li] = acc.normalized() if acc.length > 1e-12 else n0
    me.normals_split_custom_set(loop_normals)


def _bm_to_object(bm, materials, name, collection, smooth_angle):
    """Turn a bmesh into a new object; keeps only materials still in use."""
    zero = [f for f in bm.faces if _exact_area(f) < 1e-9]
    if zero:  # zero-area faces (float32) are invisible and break exporters
        bmesh.ops.delete(bm, geom=zero, context='FACES_ONLY')
    used = sorted({f.material_index for f in bm.faces})
    remap = {old: new for new, old in enumerate(used)}
    for f in bm.faces:
        f.material_index = remap[f.material_index]
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    for old in used:
        me.materials.append(materials[old])
    me.validate()
    if smooth_angle is not None and me.polygons:
        me.shade_smooth()
        me.set_sharp_from_angle(angle=smooth_angle)
        set_area_weighted_normals(me, smooth_angle)
    me.update()
    obj = bpy.data.objects.new(name, me)
    collection.objects.link(obj)
    return obj


def _base_name(collection):
    name = collection.name
    for ext in (".dae", ".zae", ".DAE"):
        if ext in name:
            name = name.rsplit(ext, 1)[0] + name.rsplit(ext, 1)[1]
    return name


def remove_previous_results(collection):
    """Delete objects an earlier run created in this collection."""
    for o in list(collection.objects):
        if _is_generated(o):
            me = o.data
            bpy.data.objects.remove(o, do_unlink=True)
            if me is not None and me.users == 0:
                bpy.data.meshes.remove(me)


def unite_collection(collection, merge_distance=1e-4, fix_normals=True,
                     remove_enclosed=True, on_ground=True, two_sided=True,
                     smooth_angle=math.radians(30.0), ray_count=48,
                     remove_unrelated=True, keep_buildings='MAIN',
                     keep_distance=1.0, keep_removed=True, remove_ground=True,
                     close_gap=0.4, fill_holes=True):
    """Create one mesh object from all meshes in `collection`.

    The source objects are not modified. Faces that do not belong to the
    house (remove_unrelated) go to a hidden object "<name>_removed" when
    keep_removed is set, so they can be reviewed. Returns (new_object, stats).
    """
    sources = [o for o in collection.all_objects
               if o.type == 'MESH' and not _is_generated(o)]
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

    # Unrelated geometry goes before orientation: a backdrop blocks the rays
    # and a map underlay would define the ground level.
    removed_bm = None
    stats["unrelated_parts"] = 0
    stats["unrelated_faces"] = 0
    stats["unrelated_area"] = 0.0
    if remove_unrelated and bm.faces:
        doomed, info = find_unrelated(
            bm, _collection_cameras(collection), keep_distance=keep_distance,
            keep_buildings=keep_buildings, remove_ground=remove_ground)
        stats["main_building_by"] = info["main_by"]
        stats["buildings_found"] = info["buildings"]
        stats["unrelated_parts"] = len(info["parts"])
        stats["unrelated_faces"] = len(doomed)
        stats["unrelated_area"] = round(sum(p["area"] for p in info["parts"]), 2)
        stats["unrelated_summary"] = _describe_parts(info["parts"])
        stats["unrelated_details"] = info["parts"]
        if doomed:
            bm.faces.ensure_lookup_table()
            if keep_removed:
                bm.faces.index_update()
                removed_bm = bm.copy()
                removed_bm.faces.index_update()
                removed_bm.faces.ensure_lookup_table()
                bmesh.ops.delete(removed_bm, geom=[f for f in removed_bm.faces
                                                   if f.index not in doomed],
                                 context='FACES')
            bmesh.ops.delete(bm, geom=[bm.faces[i] for i in doomed], context='FACES')

    if close_gap and bm.faces:
        count, area = close_gaps(bm, close_gap, merge_distance)
        stats["gaps_closed"] = count
        stats["gap_area"] = round(area, 2)

    if fill_holes and bm.faces:
        count, area = fill_missing_faces(bm)
        stats["holes_filled"] = count
        stats["hole_area"] = round(area, 2)

    if fix_normals and bm.faces:
        stats.update(orient_outward(bm, ray_count, remove_enclosed, on_ground))
    floor_layer = bm.faces.layers.int.get(FLOOR_LAYER)
    if floor_layer is not None:
        # a filled floor closes the house from below: it always points down
        up = [f for f in bm.faces if f[floor_layer] and f.normal.z > 0.0]
        if up:
            bmesh.ops.reverse_faces(bm, faces=up)
            bm.normal_update()
        bm.faces.layers.int.remove(floor_layer)

    stats["degenerate_removed"] += remove_degenerate(bm, merge_distance)

    # Last geometry step: nothing may weld after the twins exist.
    if two_sided:
        stats["two_sided_faces"] = make_two_sided(bm)
    else:
        layer = bm.faces.layers.int.get(TWO_SIDED_LAYER)
        if layer is not None:
            bm.faces.layers.int.remove(layer)

    remove_previous_results(collection)
    name = _base_name(collection) + "_united"
    obj = _bm_to_object(bm, materials, name, collection, smooth_angle)
    bm.free()
    obj[RESULT_TAG] = True

    if removed_bm is not None:
        rem = _bm_to_object(removed_bm, materials, _base_name(collection) + "_removed",
                            collection, None)
        removed_bm.free()
        rem[REMOVED_TAG] = True
        rem.hide_render = True
        try:
            rem.hide_set(True)
        except RuntimeError:  # collection excluded from the view layer
            pass
        obj["unite_removed_object"] = rem.name
        stats["removed_object"] = rem.name

    stats["faces_out"] = len(obj.data.polygons)
    stats["verts_out"] = len(obj.data.vertices)
    stats["materials_out"] = len(obj.data.materials)
    return obj, stats


# --------------------------------------------------------------------------
# Placement, import, export
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


def collada_importer_available():
    return hasattr(bpy.ops.import_scene, "collada")


def import_dae(context, filepath):
    """Import one COLLADA file with the Collada Support importer.

    Returns the collection that holds the imported objects (the importer
    creates one per file; if it does not, the new objects are moved into a
    new collection named after the file).
    """
    if not collada_importer_available():
        raise RuntimeError("COLLADA importer not found: install the "
                           "'Collada Support' extension (Blender 5.x)")
    before_colls = set(bpy.data.collections)
    before_objs = set(bpy.data.objects)
    result = bpy.ops.import_scene.collada(filepath=filepath)
    if 'FINISHED' not in result:
        raise RuntimeError(f"import failed: {os.path.basename(filepath)}")
    new_colls = [c for c in bpy.data.collections if c not in before_colls]
    if new_colls:
        coll = max(new_colls, key=lambda c: len(c.all_objects))
    else:
        coll = bpy.data.collections.new(os.path.basename(filepath))
        context.scene.collection.children.link(coll)
        for o in bpy.data.objects:
            if o not in before_objs:
                for c in list(o.users_collection):
                    c.objects.unlink(o)
                coll.objects.link(o)
    coll["unite_source"] = filepath
    return coll


def _unite_with_settings(collection, s):
    return unite_collection(
        collection,
        merge_distance=s.merge_distance,
        fix_normals=s.fix_normals,
        remove_enclosed=s.remove_enclosed,
        on_ground=s.on_ground,
        two_sided=s.two_sided,
        smooth_angle=s.smooth_angle,
        remove_unrelated=s.remove_unrelated,
        keep_buildings=s.keep_buildings,
        keep_distance=s.keep_distance,
        keep_removed=s.keep_removed,
        remove_ground=s.remove_ground,
        close_gap=s.gap_width if s.close_gaps else 0.0,
        fill_holes=s.fill_holes,
    )


def _top_level(coll):
    members = set(coll.objects)
    return [o for o in coll.objects if o.parent is None or o.parent not in members]


def repair_collection(context, collection, s):
    """Run the pipeline with the panel settings `s` and place the result.

    A model moved by "Arrange Models" is put back to its exact imported
    position for the repair, so the result does not depend on the layout
    (float precision far from the origin) and its mesh keeps the original
    coordinates; only the result's location follows the arranged model.
    """
    arranged = [(o, o.location.copy()) for o in _top_level(collection) if "unite_home" in o]
    if arranged:
        for o, _loc in arranged:
            o.location = o["unite_home"]
        context.view_layer.update()
    try:
        obj, stats = _unite_with_settings(collection, s)
    finally:
        if arranged:
            for o, loc in arranged:
                if o.name in bpy.data.objects:
                    o.location = loc
            context.view_layer.update()
    if obj is None:
        return None, stats
    removed = bpy.data.objects.get(stats.get("removed_object", ""))
    offset = Vector(collection.get("unite_offset", (0.0, 0.0, 0.0)))
    if arranged:
        for o in (obj, removed):
            if o is not None:
                o.location = offset
        context.view_layer.update()
    if collection.get("unite_source"):
        obj["unite_source"] = collection["unite_source"]
    others = [o for o in collection.all_objects if o is not obj and not _is_generated(o)]
    if s.place_beside and s.source_action != 'DELETE':
        meshes = [o for o in others if o.type == 'MESH']
        if meshes:
            stats["offset_x"] = round(place_beside(obj, meshes, s.gap, context.scene), 4)
    if removed is not None:
        removed.location = obj.location
    if s.source_action == 'HIDE':
        for o in others:
            o.hide_set(True)
            o.hide_render = True
    elif s.source_action == 'DELETE':
        for o in others:
            bpy.data.objects.remove(o, do_unlink=True)
    for key, value in stats.items():
        if isinstance(value, (int, float, str)):
            obj["unite_" + key] = value
    context.view_layer.update()  # later placements read matrix_world
    return obj, stats


def export_name(obj):
    source = obj.get("unite_source")
    if source:
        name = os.path.splitext(os.path.basename(source))[0]
    else:
        name = obj.name
        if name.endswith("_united"):
            name = name[:-len("_united")]
    return bpy.path.clean_name(name).strip("_") or "model"


def export_subdir(obj):
    """Folder path below the export folder: the import's subfolder ('' if none)."""
    return obj.get("unite_subdir", "")


def export_glb(context, obj, filepath, s):
    """Export one repaired object as binary glTF 2.0 (.glb).

    A temporary object that shares the mesh is exported at the original
    coordinates (the offset used to show the result beside the original is
    ignored), alone, without cameras/lights. The repaired object itself is not
    touched. Everything is looked up by name afterwards because the glTF
    exporter may use undo internally, which invalidates Python references.
    """
    view_layer = context.view_layer
    selected = [o.name for o in view_layer.objects if o.select_get()]
    active = view_layer.objects.active.name if view_layer.objects.active else None
    tmp = bpy.data.objects.new(os.path.splitext(os.path.basename(filepath))[0], obj.data)
    tmp.matrix_world = Matrix.Identity(4)
    context.scene.collection.objects.link(tmp)
    tmp_name = tmp.name
    try:
        for o in view_layer.objects:
            if o.select_get():
                o.select_set(False)
        tmp.select_set(True)
        view_layer.objects.active = tmp
        os.makedirs(os.path.dirname(filepath) or ".", exist_ok=True)
        result = bpy.ops.export_scene.gltf(
            filepath=filepath,
            export_format='GLB',
            use_selection=True,
            use_active_scene=True,  # selections in other scenes must not leak in
            export_apply=True,
            export_yup=s.export_yup,
            export_materials='EXPORT' if s.export_materials else 'NONE',
            export_normals=True,
            export_texcoords=True,
            export_cameras=False,
            export_lights=False,
            export_animations=False,
            export_extras=False,
        )
        if 'FINISHED' not in result:
            raise RuntimeError(f"glTF export failed: {filepath}")
    finally:
        tmp = bpy.data.objects.get(tmp_name)
        if tmp is not None:
            bpy.data.objects.remove(tmp, do_unlink=True)
        for name in selected:
            o = view_layer.objects.get(name)
            if o is not None:
                o.select_set(True)
        view_layer.objects.active = view_layer.objects.get(active) if active else None
    return filepath


# --------------------------------------------------------------------------
# Several imports in one scene: folder collections and a tidy layout
# --------------------------------------------------------------------------

def folder_collection(context, root_dir, rel_dir, cache):
    """Collection for a folder of the import (nested like the folders).

    root_dir itself becomes the top collection; a model in root/a/b goes into
    the collection 'b' inside 'a' inside the top one. Reuses a collection made
    by an earlier import of the same folder.
    """
    key = os.path.normpath(os.path.join(root_dir, rel_dir))
    if key in cache:
        return cache[key]
    if rel_dir in ("", "."):
        parent = context.scene.collection
        label = os.path.basename(os.path.normpath(root_dir)) or "Import"
    else:
        parent = folder_collection(context, root_dir, os.path.dirname(rel_dir), cache)
        label = os.path.basename(rel_dir)
    coll = next((c for c in parent.children if c.get("unite_folder") == key), None)
    if coll is None:
        coll = bpy.data.collections.new(label)
        coll["unite_folder"] = key
        parent.children.link(coll)
    cache[key] = coll
    return coll


def move_collection(coll, new_parent, scene):
    if coll.name in new_parent.children:
        return
    for parent in [scene.collection] + list(bpy.data.collections):
        if coll.name in parent.children:
            parent.children.unlink(coll)
    new_parent.children.link(coll)


def _model_box(coll, reserve_result):
    """XY footprint of an imported model, its repaired result included."""
    boxes = [_world_bbox(o) for o in coll.objects
             if o.type == 'MESH' and not o.get(REMOVED_TAG)]
    if not boxes:
        return None
    lo = Vector((min(b[0].x for b in boxes), min(b[0].y for b in boxes), 0.0))
    hi = Vector((max(b[1].x for b in boxes), max(b[1].y for b in boxes), 0.0))
    if reserve_result:  # room for the result that "place beside" will add later
        hi.x += (hi.x - lo.x) + 2.0
    return lo, hi


def shift_model(coll, delta):
    """Move a whole imported model; only top-level objects are moved, so the
    geometry and the internal hierarchy stay exactly as imported."""
    for o in _top_level(coll):
        # exact imported position, used by the repair (results are rebuilt)
        if "unite_home" not in o and not _is_generated(o):
            o["unite_home"] = tuple(o.location)
        o.location += delta
    coll["unite_offset"] = Vector(coll.get("unite_offset", (0.0, 0.0, 0.0))) + delta


LAYOUT_KEY = "unite_layout"
ROW_LENGTH = 8


def _has_result(coll):
    return any(o.get(RESULT_TAG) for o in coll.objects)


def _free_row_top(context, exclude, spacing):
    """Y where a new row can start: below every mesh not in `exclude`."""
    boxes = [_world_bbox(o) for o in context.scene.objects
             if o.type == 'MESH' and not any(o.name in c.objects for c in exclude)]
    return min((b[0].y for b in boxes), default=0.0) - (spacing if boxes else 0.0)


def append_model(context, coll, spacing=10.0, reserve_result=False):
    """Place one newly imported model next to the previous one.

    Imports one by one fill rows of ROW_LENGTH models; a new row starts below
    everything in the scene. The row state lives in scene[LAYOUT_KEY].
    """
    scene = context.scene
    box = _model_box(coll, reserve_result and not _has_result(coll))
    if box is None:
        return
    lo, hi = box
    state = scene.get(LAYOUT_KEY)
    state = dict(state) if state else None
    if not state or state["count"] >= ROW_LENGTH:
        state = {"top": _free_row_top(context, [coll], spacing),
                 "x": 0.0, "depth": 0.0, "count": 0}
    shift_model(coll, Vector((state["x"] - lo.x, state["top"] - hi.y, 0.0)))
    state["x"] += (hi.x - lo.x) + spacing
    state["depth"] = max(state["depth"], hi.y - lo.y)
    state["count"] += 1
    scene[LAYOUT_KEY] = state
    context.view_layer.update()


def arrange_models(context, groups, spacing=10.0, reserve_result=False):
    """Lay imported models out in a grid instead of on top of each other.

    `groups` is a list of lists of collections (one list per folder). Every
    folder becomes a block of rows; blocks follow each other along -Y, below
    whatever else is in the scene.
    """
    scene = context.scene
    placed = [c for g in groups for c in g]
    y = _free_row_top(context, placed, spacing)
    for group in groups:
        items = [(c, _model_box(c, reserve_result and not _has_result(c))) for c in group]
        items = [(c, b) for c, b in items if b is not None]
        if not items:
            continue
        cols = max(1, math.ceil(math.sqrt(len(items))))
        for r in range(0, len(items), cols):
            row = items[r:r + cols]
            depth = max(b[1].y - b[0].y for _c, b in row)
            x = 0.0
            for coll, (lo, hi) in row:
                delta = Vector((x - lo.x, y - hi.y, 0.0))
                shift_model(coll, delta)
                x += (hi.x - lo.x) + spacing
            y -= depth + spacing
        y -= spacing  # extra gap between folders
    if LAYOUT_KEY in scene:  # the next single import starts a new row
        del scene[LAYOUT_KEY]
    # matrix_world of moved objects is only refreshed by a depsgraph update;
    # the repair reads it right after this
    context.view_layer.update()


def scene_model_groups(context):
    """Every imported model in the scene, grouped by its parent collection."""
    groups = {}

    def walk(parent):
        for child in parent.children:
            if child.get("unite_source"):
                groups.setdefault(parent.name, []).append(child)
            else:
                walk(child)
    walk(context.scene.collection)
    return [sorted(g, key=lambda c: c.get("unite_source", c.name)) for _k, g in
            sorted(groups.items())]


def _snapshot_data():
    return (set(bpy.data.meshes), set(bpy.data.materials), set(bpy.data.images),
            set(bpy.data.cameras), set(bpy.data.lights))


def _remove_new_data(snapshot):
    """Free datablocks created since `snapshot` that nothing uses any more."""
    for coll, before in zip((bpy.data.meshes, bpy.data.materials, bpy.data.images,
                             bpy.data.cameras, bpy.data.lights), snapshot):
        for block in [b for b in coll if b not in before]:
            if block.users == 0:
                coll.remove(block)


def remove_collection(coll):
    for o in list(coll.all_objects):
        bpy.data.objects.remove(o, do_unlink=True)
    for child in list(coll.children_recursive):
        bpy.data.collections.remove(child)
    bpy.data.collections.remove(coll)


def process_dae_file(context, filepath, out_path, s, keep_in_scene=False):
    """Batch step: import -> repair -> export one file. Returns a report row."""
    row = {"file": filepath, "glb": "", "status": "ok", "error": ""}
    snapshot = _snapshot_data()
    coll = None
    t0 = time.time()
    try:
        coll = import_dae(context, filepath)
        obj, stats = repair_collection(context, coll, s)
        if obj is None:
            raise RuntimeError(stats.get("error", "nothing to repair"))
        export_glb(context, obj, out_path, s)
        row["glb"] = out_path
        for key in ("source_objects", "faces_in", "faces_out", "flipped",
                    "back_faces_removed", "enclosed_removed", "two_sided_faces",
                    "unrelated_parts", "unrelated_faces", "unrelated_area",
                    "buildings_found", "main_building_by", "unrelated_summary"):
            row[key] = stats.get(key, "")
    except Exception as exc:  # report and continue with the next file
        row["status"] = "error"
        row["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        if coll is not None and not keep_in_scene:
            remove_collection(coll)
            _remove_new_data(snapshot)
    row["seconds"] = round(time.time() - t0, 2)
    return row


REPORT_COLUMNS = ["file", "status", "glb", "source_objects", "faces_in", "faces_out",
                  "flipped", "back_faces_removed", "enclosed_removed", "two_sided_faces",
                  "unrelated_parts", "unrelated_faces", "unrelated_area",
                  "buildings_found", "main_building_by", "unrelated_summary",
                  "seconds", "error"]


def write_report(rows, out_dir):
    path = os.path.join(out_dir, "unite_report.csv")
    with open(path, "w", newline="", encoding="utf-8-sig") as fh:
        w = csv.DictWriter(fh, fieldnames=REPORT_COLUMNS, extrasaction="ignore", delimiter=";")
        w.writeheader()
        for r in rows:
            w.writerow(r)
    return path


def list_dae_files(directory, recursive):
    pattern = os.path.join(directory, "**", "*") if recursive else os.path.join(directory, "*")
    return sorted(p for p in glob.glob(pattern, recursive=recursive)
                  if p.lower().endswith((".dae", ".zae")) and os.path.isfile(p))


# --------------------------------------------------------------------------
# Blender UI
# --------------------------------------------------------------------------

def _has_own_meshes(coll):
    return any(o.type == 'MESH' and not _is_generated(o) for o in coll.objects)


def _model_collections(root):
    """Collections below `root` (itself included) that hold a model's meshes.

    One import = one collection; folder collections only hold collections."""
    found = [root] if _has_own_meshes(root) else []
    for child in root.children:
        found += _model_collections(child)
    return found


def _target_collections(context, scope):
    if scope == 'ACTIVE':
        coll = context.view_layer.active_layer_collection.collection
        if coll is context.scene.collection:
            return []
        return _model_collections(coll)  # a folder collection: every model in it
    return [c for child in context.scene.collection.children
            for c in _model_collections(child)]


class UniteModelSettings(bpy.types.PropertyGroup):
    # import
    repair_after_import: BoolProperty(
        name="Repair After Import", default=True,
        description="Run the repair on every imported model right away",
    )
    import_subfolders: BoolProperty(
        name="Include Subfolders", default=False,
        description="When a folder is imported, also import .dae files in its subfolders",
    )
    arrange_imports: BoolProperty(
        name="Arrange Models", default=True,
        description="Place every imported model (and its result) in its own spot of a "
                    "grid instead of on top of the others: a folder import as one block per "
                    "folder, single imports one after another in rows. Only whole models "
                    "are moved; the export still uses the original coordinates",
    )
    group_by_folder: BoolProperty(
        name="Collections by Folder", default=True,
        description="When a folder is imported, put the models into collections named and "
                    "nested like the folders",
    )
    # repair
    merge_distance: FloatProperty(
        name="Merge Distance", default=0.0001, min=0.0, soft_max=0.01,
        precision=5, unit='LENGTH',
        description="Vertices closer than this are welded. Keep it well below the shortest edge",
    )
    remove_unrelated: BoolProperty(
        name="Remove Unrelated Geometry", default=True,
        description="Delete map underlays, photo backdrops, stray parts and other "
                    "geometry that does not belong to the house",
    )
    keep_buildings: EnumProperty(
        name="Buildings",
        items=[
            ('MAIN', "Main Only", "Keep only the house the file is about (chosen by the "
                                  "saved SketchUp camera, otherwise the largest)"),
            ('ALL', "All", "Keep every building found in the file"),
        ],
        default='MAIN',
    )
    keep_distance: FloatProperty(
        name="Keep Distance", default=1.0, min=0.0, soft_max=10.0, unit='LENGTH',
        description="Detached parts closer than this to the house are kept (doors, signs, "
                    "railings); farther ones are removed",
    )
    remove_ground: BoolProperty(
        name="Remove Ground Strips", default=True,
        description="Delete flat faces at ground level that are attached to the house "
                    "but not covered by it (pavement, driveway, yard)",
    )
    keep_removed: BoolProperty(
        name="Keep Removed Parts", default=True,
        description="Store removed geometry in a hidden '<name>_removed' object for review",
    )
    close_gaps: BoolProperty(
        name="Close Slits", default=True,
        description="Fill narrow slits the model leaves open (e.g. between a facade and "
                    "the cornice). SketchUp hides them by drawing both sides of faces",
    )
    gap_width: FloatProperty(
        name="Max Slit Width", default=0.4, min=0.01, soft_max=1.0, unit='LENGTH',
        description="Wider openings (windows, doors) stay open",
    )
    fill_holes: BoolProperty(
        name="Fill Missing Faces", default=True,
        description="Close holes where a face of a closed shape is missing (a gable, "
                    "the floor, a chimney top). Windows cut into walls stay open",
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
            ('KEEP', "Keep", "Leave the original objects visible"),
            ('HIDE', "Hide", "Keep the original objects but hide them"),
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
    show_repair_options: BoolProperty(name="Options", default=False)
    # export
    export_dir: StringProperty(
        name="Folder", subtype='DIR_PATH', default="",
        description="Folder for the exported .glb files",
    )
    export_scope: EnumProperty(
        name="Export",
        items=[
            ('SELECTED', "Selected", "Repaired objects that are selected"),
            ('ALL', "All", "Every repaired object in the scene"),
        ],
        default='ALL',
    )
    export_materials: BoolProperty(
        name="Materials", default=True,
        description="Export materials and textures",
    )
    export_yup: BoolProperty(
        name="+Y Up", default=True,
        description="glTF convention: +Y is up (Blender's +Z is converted)",
    )
    # batch
    batch_input_dir: StringProperty(
        name="Input", subtype='DIR_PATH', default="",
        description="Folder with .dae files to import, repair and export",
    )
    batch_subfolders: BoolProperty(
        name="Include Subfolders", default=True,
        description="Process .dae files in subfolders too; the folder structure is "
                    "kept in the output folder",
    )
    batch_keep_in_scene: BoolProperty(
        name="Keep in Scene", default=False,
        description="Leave imported and repaired models in the scene (uses a lot of "
                    "memory for many files)",
    )


def _find_layer_collection(layer, coll):
    if layer.collection is coll:
        return layer
    for child in layer.children:
        found = _find_layer_collection(child, coll)
        if found is not None:
            return found
    return None


def _settings(context):
    return context.scene.unite_model


class UNITE_OT_import_dae(bpy.types.Operator, ImportHelper):
    """Import COLLADA (.dae) files with the Collada Support importer; select files, or a folder to import every .dae in it"""
    bl_idname = "unite_model.import_dae"
    bl_label = "Import COLLADA"
    bl_options = {'REGISTER', 'UNDO'}

    filename_ext = ".dae"
    filter_glob: StringProperty(default="*.dae;*.zae", options={'HIDDEN'})
    files: CollectionProperty(type=bpy.types.OperatorFileListElement,
                              options={'HIDDEN', 'SKIP_SAVE'})
    directory: StringProperty(subtype='DIR_PATH', options={'HIDDEN', 'SKIP_SAVE'})

    def execute(self, context):
        if not collada_importer_available():
            self.report({'ERROR'}, "COLLADA importer not found: install the "
                                   "'Collada Support' extension")
            return {'CANCELLED'}
        s = _settings(context)
        names = [f.name for f in self.files if f.name]
        folder_import = not names
        if names:
            paths = [os.path.join(self.directory, n) for n in names]
        else:
            paths = list_dae_files(self.directory, s.import_subfolders)
        if not paths:
            self.report({'WARNING'}, "No .dae files selected")
            return {'CANCELLED'}
        if context.mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')
        done, failed, results, last = 0, [], [], None
        groups, folders = {}, {}
        for path in paths:
            try:
                coll = import_dae(context, path)
            except Exception as exc:
                failed.append(f"{os.path.relpath(path, self.directory)} ({exc})")
                continue
            done += 1
            last = coll
            rel_dir = os.path.dirname(os.path.relpath(path, self.directory))
            coll["unite_subdir"] = rel_dir
            if folder_import and s.group_by_folder:
                move_collection(coll, folder_collection(context, self.directory, rel_dir,
                                                        folders), context.scene)
            groups.setdefault(rel_dir, []).append(coll)
        imported = [c for g in groups.values() for c in g]
        if s.arrange_imports and len(imported) > 1:
            arrange_models(context, [groups[k] for k in sorted(groups)],
                           reserve_result=s.place_beside)
        elif s.arrange_imports and imported:
            append_model(context, imported[0], reserve_result=s.place_beside)
        if s.repair_after_import:
            for coll in imported:
                obj, _stats = repair_collection(context, coll, s)
                if obj is not None:
                    obj["unite_subdir"] = coll.get("unite_subdir", "")
                    results.append(obj)
        if results:
            for o in context.selected_objects:
                o.select_set(False)
            for o in results:
                o.select_set(True)
            context.view_layer.objects.active = results[-1]
        if last is not None:  # the Repair section then works on the new models
            top = folders.get(os.path.normpath(self.directory)) if folders else None
            layer = _find_layer_collection(context.view_layer.layer_collection, top or last)
            if layer is not None:
                context.view_layer.active_layer_collection = layer
        msg = f"Imported {done} file(s)"
        if s.repair_after_import:
            msg += f", repaired {len(results)}"
        if failed:
            msg += f"; failed: {', '.join(failed[:3])}"
            self.report({'WARNING'}, msg)
        else:
            self.report({'INFO'}, msg)
        return {'FINISHED'} if done else {'CANCELLED'}


class UNITE_OT_arrange(bpy.types.Operator):
    """Lay out every imported model in the scene in a grid (one block per folder collection), so no model overlaps another. Only whole models move; exports keep the original coordinates"""
    bl_idname = "unite_model.arrange"
    bl_label = "Arrange Scene"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        if context.mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')
        groups = scene_model_groups(context)
        count = sum(len(g) for g in groups)
        if not count:
            self.report({'WARNING'}, "No imported models in the scene")
            return {'CANCELLED'}
        arrange_models(context, groups, reserve_result=_settings(context).place_beside)
        self.report({'INFO'}, f"Arranged {count} model(s)")
        return {'FINISHED'}


class UNITE_OT_collection(bpy.types.Operator):
    """Join all meshes of a collection into one object, remove unrelated geometry and double-sided back faces, fix normals"""
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

    def execute(self, context):
        if context.mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')
        targets = _target_collections(context, self.scope)
        if not targets:
            self.report({'WARNING'}, "No collection with meshes selected")
            return {'CANCELLED'}
        s = _settings(context)
        results = []
        for coll in targets:
            obj, stats = repair_collection(context, coll, s)
            if obj is not None:
                results.append((coll.name, obj, stats))

        for o in context.selected_objects:
            o.select_set(False)
        for _name, obj, _stats in results:
            obj.select_set(True)
        if results:
            context.view_layer.objects.active = results[-1][1]

        msg = "; ".join(
            f"{name}: {st['source_objects']} obj -> 1, faces {st['faces_in']} -> "
            f"{st['faces_out']}, unrelated parts {st.get('unrelated_parts', 0)}"
            for name, _obj, st in results)
        self.report({'INFO'}, msg or "Nothing to do")
        return {'FINISHED'}


class UNITE_OT_toggle_removed(bpy.types.Operator):
    """Show or hide the geometry that was removed as unrelated"""
    bl_idname = "unite_model.toggle_removed"
    bl_label = "Show Removed Parts"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        objs = [o for o in context.view_layer.objects if o.get(REMOVED_TAG)]
        if not objs:
            self.report({'INFO'}, "No removed parts in the scene")
            return {'CANCELLED'}
        show = any(o.hide_get() for o in objs)
        for o in objs:
            o.hide_set(not show)
        self.report({'INFO'}, ("Showing" if show else "Hiding") + f" {len(objs)} object(s)")
        return {'FINISHED'}


class UNITE_OT_check_view(bpy.types.Operator):
    """Toggle Face Orientation together with Backface Culling in this 3D view.
With culling, only the side a face points to is drawn: a wrong normal shows as a hole, red as a wrong side; two-sided faces show correctly from both sides"""
    bl_idname = "unite_model.check_view"
    bl_label = "Normals Check View"

    @classmethod
    def poll(cls, context):
        return context.space_data is not None and context.space_data.type == 'VIEW_3D'

    def execute(self, context):
        space = context.space_data
        on = not (space.overlay.show_face_orientation and space.shading.show_backface_culling)
        space.overlay.show_face_orientation = on
        space.shading.show_backface_culling = on
        return {'FINISHED'}


class UNITE_OT_export_glb(bpy.types.Operator):
    """Export repaired models as glTF 2.0 binary (.glb), one file per model"""
    bl_idname = "unite_model.export_glb"
    bl_label = "Export GLB"
    bl_options = {'REGISTER'}

    def execute(self, context):
        s = _settings(context)
        out_dir = bpy.path.abspath(s.export_dir)
        if not s.export_dir:
            self.report({'ERROR'}, "Choose the export folder first")
            return {'CANCELLED'}
        objs = [o for o in context.view_layer.objects if o.get(RESULT_TAG)]
        if s.export_scope == 'SELECTED':
            objs = [o for o in objs if o.select_get()]
        if not objs:
            self.report({'WARNING'}, "No repaired objects to export")
            return {'CANCELLED'}
        written, used = [], set()
        for obj in objs:
            sub = export_subdir(obj)
            name = export_name(obj)
            base, k = name, 2
            while os.path.join(sub, name).lower() in used:
                name = f"{base}_{k}"
                k += 1
            used.add(os.path.join(sub, name).lower())
            try:
                written.append(export_glb(context, obj, os.path.join(out_dir, sub, name + ".glb"), s))
            except Exception as exc:
                self.report({'ERROR'}, f"{obj.name}: {exc}")
        self.report({'INFO'}, f"Exported {len(written)} file(s) to {out_dir}")
        return {'FINISHED'} if written else {'CANCELLED'}


class UNITE_OT_batch(bpy.types.Operator):
    """Import every .dae in the input folder, repair it and export it as .glb into the export folder; writes unite_report.csv (Esc cancels)"""
    bl_idname = "unite_model.batch"
    bl_label = "Run Batch"

    _timer = None

    def execute(self, context):
        s = _settings(context)
        if not collada_importer_available():
            self.report({'ERROR'}, "COLLADA importer not found: install the "
                                   "'Collada Support' extension")
            return {'CANCELLED'}
        self.in_dir = bpy.path.abspath(s.batch_input_dir)
        self.out_dir = bpy.path.abspath(s.export_dir)
        if not s.batch_input_dir or not os.path.isdir(self.in_dir):
            self.report({'ERROR'}, "Choose an existing input folder")
            return {'CANCELLED'}
        if not s.export_dir:
            self.report({'ERROR'}, "Choose the export folder")
            return {'CANCELLED'}
        self.files = list_dae_files(self.in_dir, s.batch_subfolders)
        if not self.files:
            self.report({'WARNING'}, "No .dae files in the input folder")
            return {'CANCELLED'}
        if context.mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')
        os.makedirs(self.out_dir, exist_ok=True)
        self.rows = []
        self.index = 0
        wm = context.window_manager
        self._timer = wm.event_timer_add(0.05, window=context.window)
        wm.modal_handler_add(self)
        wm.progress_begin(0, len(self.files))
        return {'RUNNING_MODAL'}

    def _finish(self, context, cancelled=False):
        wm = context.window_manager
        wm.event_timer_remove(self._timer)
        wm.progress_end()
        context.workspace.status_text_set(None)
        report = write_report(self.rows, self.out_dir)
        bad = sum(1 for r in self.rows if r["status"] != "ok")
        msg = (f"{'Cancelled' if cancelled else 'Done'}: {len(self.rows) - bad} exported, "
               f"{bad} failed. Report: {report}")
        self.report({'WARNING'} if bad or cancelled else {'INFO'}, msg)
        return {'CANCELLED'} if cancelled else {'FINISHED'}

    def modal(self, context, event):
        if event.type == 'ESC':
            return self._finish(context, cancelled=True)
        if event.type != 'TIMER':
            return {'PASS_THROUGH'}
        if self.index >= len(self.files):
            return self._finish(context)
        s = _settings(context)
        path = self.files[self.index]
        rel = os.path.relpath(path, self.in_dir)
        out_path = os.path.join(self.out_dir, os.path.splitext(rel)[0] + ".glb")
        context.workspace.status_text_set(
            f"Unite batch {self.index + 1}/{len(self.files)}: {rel}   (Esc to cancel)")
        self.rows.append(process_dae_file(context, path, out_path, s,
                                          keep_in_scene=s.batch_keep_in_scene))
        self.index += 1
        context.window_manager.progress_update(self.index)
        return {'RUNNING_MODAL'}


class UNITE_PT_main(bpy.types.Panel):
    bl_label = "Unite Model"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Unite"

    def draw(self, context):
        layout = self.layout
        obj = context.active_object
        if obj is not None and obj.get(RESULT_TAG):
            box = layout.box()
            box.label(text=obj.name, icon='MESH_DATA')
            col = box.column(align=True)
            col.label(text=f"Objects {obj.get('unite_source_objects', '?')} -> 1, "
                           f"faces {obj.get('unite_faces_in', '?')} -> {obj.get('unite_faces_out', '?')}")
            col.label(text=f"Flipped {obj.get('unite_flipped', 0)}, "
                           f"unrelated parts removed {obj.get('unite_unrelated_parts', 0)}")
        else:
            layout.label(text="Import a .dae, then repair and export", icon='INFO')


class UNITE_PT_import(bpy.types.Panel):
    bl_label = "1  Import COLLADA"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Unite"
    bl_parent_id = "UNITE_PT_main"

    def draw(self, context):
        layout = self.layout
        s = _settings(context)
        if not collada_importer_available():
            box = layout.box()
            box.alert = True
            box.label(text="COLLADA importer missing", icon='ERROR')
            box.label(text="Install the 'Collada Support' extension")
        layout.operator(UNITE_OT_import_dae.bl_idname, text="Import .dae...", icon='IMPORT')
        col = layout.column(align=True)
        col.prop(s, "repair_after_import")
        col.prop(s, "import_subfolders")
        col.prop(s, "group_by_folder")
        col.prop(s, "arrange_imports")
        layout.operator(UNITE_OT_arrange.bl_idname, icon='MOD_ARRAY')


class UNITE_PT_repair(bpy.types.Panel):
    bl_label = "2  Repair"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Unite"
    bl_parent_id = "UNITE_PT_main"

    def draw(self, context):
        layout = self.layout
        s = _settings(context)
        coll = context.view_layer.active_layer_collection.collection
        layout.label(text=f"Active: {coll.name}", icon='OUTLINER_COLLECTION')
        col = layout.column(align=True)
        op = col.operator(UNITE_OT_collection.bl_idname, text="Unite Active Collection")
        op.scope = 'ACTIVE'
        op = col.operator(UNITE_OT_collection.bl_idname, text="Unite All Collections")
        op.scope = 'ALL'
        row = layout.row(align=True)
        row.operator(UNITE_OT_toggle_removed.bl_idname, icon='HIDE_OFF')
        row.operator(UNITE_OT_check_view.bl_idname, text="Check Normals", icon='NORMALS_FACE')

        box = layout.box()
        box.prop(s, "show_repair_options",
                 icon='TRIA_DOWN' if s.show_repair_options else 'TRIA_RIGHT', emboss=False)
        if s.show_repair_options:
            col = box.column()
            col.prop(s, "remove_unrelated")
            sub = col.column()
            sub.active = s.remove_unrelated
            sub.prop(s, "keep_buildings")
            sub.prop(s, "keep_distance")
            sub.prop(s, "remove_ground")
            sub.prop(s, "keep_removed")
            col.separator()
            col.prop(s, "merge_distance")
            col.prop(s, "close_gaps")
            sub = col.column()
            sub.active = s.close_gaps
            sub.prop(s, "gap_width")
            col.prop(s, "fill_holes")
            col.prop(s, "fix_normals")
            col.prop(s, "remove_enclosed")
            col.prop(s, "two_sided")
            col.prop(s, "on_ground")
            col.prop(s, "smooth_angle")
            col.separator()
            col.prop(s, "source_action")
            col.prop(s, "place_beside")
            sub = col.column()
            sub.active = s.place_beside
            sub.prop(s, "gap")


class UNITE_PT_export(bpy.types.Panel):
    bl_label = "3  Export glTF 2.0 (.glb)"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Unite"
    bl_parent_id = "UNITE_PT_main"

    def draw(self, context):
        layout = self.layout
        s = _settings(context)
        layout.prop(s, "export_dir")
        row = layout.row()
        row.prop(s, "export_scope", expand=True)
        row = layout.row()
        row.prop(s, "export_materials")
        row.prop(s, "export_yup")
        layout.operator(UNITE_OT_export_glb.bl_idname, icon='EXPORT')


class UNITE_PT_batch(bpy.types.Panel):
    bl_label = "Batch: Folder to GLB"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Unite"
    bl_parent_id = "UNITE_PT_main"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        s = _settings(context)
        layout.prop(s, "batch_input_dir")
        layout.label(text="Output: export folder (section 3)")
        col = layout.column(align=True)
        col.prop(s, "batch_subfolders")
        col.prop(s, "batch_keep_in_scene")
        layout.operator(UNITE_OT_batch.bl_idname, icon='FILE_REFRESH')


classes = (
    UniteModelSettings,
    UNITE_OT_import_dae,
    UNITE_OT_arrange,
    UNITE_OT_collection,
    UNITE_OT_toggle_removed,
    UNITE_OT_check_view,
    UNITE_OT_export_glb,
    UNITE_OT_batch,
    UNITE_PT_main,
    UNITE_PT_import,
    UNITE_PT_repair,
    UNITE_PT_export,
    UNITE_PT_batch,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.unite_model = PointerProperty(type=UniteModelSettings)


def unregister():
    del bpy.types.Scene.unite_model
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
