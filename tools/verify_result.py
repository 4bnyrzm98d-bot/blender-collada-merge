# Independent quality check of a repaired ("<name>_united") object.
# Run through the Blender MCP tool (execute_blender_code). Before running, set
# RESULT_NAME below (or leave None to check every object the plugin produced).
# The script only reads; the temporary reference mesh it builds is removed.
#
# It needs the Unite Model plugin module importable: either the installed
# extension or the plugin file loaded by path (PLUGIN_PATH).
#
# Checks, with camera directions DIFFERENT from the plugin's own view check:
#   back_hits      rays from outside that land on the back of a face
#                  (= red in Face Orientation). Expect ~0; a few single hits
#                  on big faces are grazing-angle noise.
#   hole_rays      rays that hit the reference (nothing deleted) but pass
#                  through, or go >1 cm deeper, on the result. Expect 0.
#   inconsistent_edges  edges whose two faces disagree on winding. Expect 0.
#   duplicate_faces / zero_area   leftovers of the cleanup. Expect 0.
# Views from below the horizon count only when the ray leaves the terrain on
# open ground (a pedestrian under the eaves looking up). A ray that enters
# through the open bottom of a model is not a real view: it once made the
# deleted, truly hidden attic floors look like holes.

import bpy, bmesh, math, os, sys, importlib.util
from mathutils import Vector
from mathutils.bvhtree import BVHTree

RESULT_NAME = None
PLUGIN_PATH = None   # e.g. "<repo>/unite_model" (folder or its __init__.py)
GRID = 150


def load_plugin():
    for name in ("unite_model_dev", "unite_model_test"):
        if name in sys.modules:
            return sys.modules[name]
    for key, mod in sys.modules.items():
        if key.endswith("unite_model") and hasattr(mod, "unite_collection"):
            return mod
    if PLUGIN_PATH:
        plugin_dir = PLUGIN_PATH if os.path.isdir(PLUGIN_PATH) else os.path.dirname(PLUGIN_PATH)
        spec = importlib.util.spec_from_file_location(
            "unite_model_dev", os.path.join(plugin_dir, "__init__.py"),
            submodule_search_locations=[plugin_dir])
        mod = importlib.util.module_from_spec(spec)
        sys.modules["unite_model_dev"] = mod
        spec.loader.exec_module(mod)
        return mod
    raise RuntimeError("Unite Model plugin not loaded; set PLUGIN_PATH")


HORIZONTAL = [Vector((math.cos(a), math.sin(a), 0.0)) for a in (0.0, 1.5708, 3.1416, 4.7124, 0.785, 2.356, 3.927, 5.498)]


def pedestrian_can_stand(bvh, spot, size):
    """A view from below the horizon is real only for a person standing on
    open ground: from ankle height some horizontal ray must reach the open.
    Under eaves that holds; inside the open bottom of a model it does not."""
    p = spot + Vector((0.0, 0.0, 0.3))
    return any(bvh.ray_cast(p, d, size * 4)[0] is None for d in HORIZONTAL)


def view_from_below_ok(bvh, origin, ray_dir, hit_dist, ground, size, cache):
    """For a ray coming up from below: the hit must be above the terrain and
    the point where the ray leaves the terrain must be open ground."""
    t = (ground - origin.z) / ray_dir.z
    if hit_dist is None or hit_dist <= t + 1e-4:
        return False
    # Only parts well above the terrain: the lowest point of a model is often a
    # step or cellar, and rays skimming under a raised floor are not real views.
    if (origin + ray_dir * hit_dist).z < ground + 0.5:
        return False
    spot = origin + ray_dir * t
    key = (round(spot.x, 1), round(spot.y, 1))
    if key not in cache:
        cache[key] = pedestrian_can_stand(bvh, spot, size)
    return cache[key]


def world_bm(obj):
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bm.transform(obj.matrix_world)
    bm.normal_update()
    bm.faces.ensure_lookup_table()
    return bm


def verify(mod, obj):
    coll = obj.users_collection[0]
    bm = world_bm(obj)
    bvh = BVHTree.FromBMesh(bm)

    ref, _ = mod.unite_collection(coll, remove_enclosed=False, fix_normals=False)
    ref.location = obj.location
    bpy.context.view_layer.update()
    rbm = world_bm(ref)
    rbvh = BVHTree.FromBMesh(rbm)
    me = ref.data
    bpy.data.objects.remove(ref, do_unlink=True)
    bpy.data.meshes.remove(me)

    cos = [v.co for v in rbm.verts]
    lo = Vector((min(c.x for c in cos), min(c.y for c in cos), min(c.z for c in cos)))
    hi = Vector((max(c.x for c in cos), max(c.y for c in cos), max(c.z for c in cos)))
    center, size = (lo + hi) / 2, (hi - lo).length
    views = [Vector((math.cos(a), math.sin(a), h)).normalized()
             for a in [k * math.pi / 12 + 0.29 for k in range(24)]
             for h in (-0.6, -0.3, 0.05, 0.2, 0.45, 1.2)] + [Vector((0.01, 0, 1))]

    hits = back = holes = 0
    per_face = {}
    cache = {}
    # Two-sided faces: a face and its reversed twin share positions; a back
    # hit on a face that has a twin is the twin's front, not an error.
    by_pos = {}
    for f in bm.faces:
        by_pos.setdefault(frozenset(tuple(round(c, 4) for c in v.co) for v in f.verts), []).append(f.index)
    has_twin = {i for group in by_pos.values() if len(group) > 1 for i in group}
    for d in views:
        cam = center + d * size * 2
        u = d.orthogonal().normalized()
        w = d.cross(u).normalized()
        for a in range(GRID):
            for b in range(GRID):
                o = cam + (u * ((a + 0.5) / GRID - 0.5) + w * ((b + 0.5) / GRID - 0.5)) * size
                loc, _n, idx, dist = bvh.ray_cast(o, -d, size * 4)
                rloc, _rn, ridx, rdist = rbvh.ray_cast(o, -d, size * 4)
                if d.z < 0:  # pedestrian looking up; judged on the full reference
                    if idx is not None and not view_from_below_ok(rbvh, o, -d, dist, lo.z, size, cache):
                        idx = None
                    if ridx is not None and not view_from_below_ok(rbvh, o, -d, rdist, lo.z, size, cache):
                        ridx = None
                if ridx is not None and (idx is None or dist - rdist > 0.01):
                    holes += 1
                if idx is None:
                    continue
                hits += 1
                if bm.faces[idx].normal.dot(d) < 0 and idx not in has_twin:
                    back += 1
                    per_face[idx] = per_face.get(idx, 0) + 1

    worst = sorted(per_face.items(), key=lambda x: -x[1])[:5]
    worst = [{"face": i, "hits": c, "area_m2": round(bm.faces[i].calc_area(), 4),
              "height_above_ground": round(bm.faces[i].calc_center_median().z - lo.z, 3),
              "normal": [round(x, 2) for x in bm.faces[i].normal]} for i, c in worst]

    local = bmesh.new()
    local.from_mesh(obj.data)
    local.normal_update()
    incons = sum(1 for e in local.edges if len(e.link_faces) == 2 and not e.is_contiguous)
    keys, dup = set(), 0
    for f in local.faces:
        k = frozenset(v.index for v in f.verts)
        dup += k in keys
        keys.add(k)
    local.free()
    bm.free()
    rbm.free()
    return {
        "faces": len(obj.data.polygons),
        "rays_hit": hits,
        "back_hits": back,
        "back_share_pct": round(100.0 * back / max(1, hits), 5),
        "worst_back_faces": worst,
        "hole_rays": holes,
        "inconsistent_edges": incons,
        "duplicate_faces": dup,
        "two_sided_faces": len(has_twin) // 2,
        "zero_area": sum(1 for p in obj.data.polygons if p.area < 1e-10),
    }


mod = load_plugin()
targets = ([bpy.data.objects[RESULT_NAME]] if RESULT_NAME else
           [o for o in bpy.data.objects if o.get("unite_model_result")])
result = {o.name: verify(mod, o) for o in targets}
