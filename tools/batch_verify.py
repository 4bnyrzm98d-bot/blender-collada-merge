# Batch test of the Unite Model plugin on many .dae files, with GLB export check.
# Runs INSIDE a background Blender process, never in the user's open session:
#
#   blender --background --python batch_verify.py -- \
#       <plugin folder or its __init__.py> <list.txt> <out.jsonl> <glb_dir> <models_root>
#
# list.txt holds one .dae path per line. For every file it imports the model
# (needs a COLLADA importer that registers bpy.ops.import_scene.collada, e.g.
# the "Collada Support" extension on Blender 5.x), runs unite_collection on the
# imported collection, exports the result to <glb_dir>/<path relative to
# models_root>.glb, re-imports the GLB, checks the result with ray tests and
# appends one JSON line to out.jsonl.
#
# Compared with batch_runner.py this adds:
#   - "solid" metrics: a back hit / hole counts only when 4 probe rays shifted
#     by 2 cm hit at a similar depth (filters views through hairline cracks)
#   - back_hits_base: hits on the underside of the base (points down by design)
#   - diag: per-face details of the worst back hits and holes
#   - GLB export + re-import comparison (triangle count, bounding box)

import bpy, bmesh, sys, os, json, time, math, traceback, importlib.util
from mathutils import Vector
from mathutils.bvhtree import BVHTree

args = sys.argv[sys.argv.index("--") + 1:]
plugin_path, list_path, out_path, GLB_DIR, ROOT = args[0], args[1], args[2], args[3], args[4]
# the plugin is a package: accept its folder or its __init__.py
plugin_dir = plugin_path if os.path.isdir(plugin_path) else os.path.dirname(plugin_path)
spec = importlib.util.spec_from_file_location(
    "unite_model_test", os.path.join(plugin_dir, "__init__.py"), submodule_search_locations=[plugin_dir])
mod = importlib.util.module_from_spec(spec)
sys.modules["unite_model_test"] = mod
spec.loader.exec_module(mod)
mod.register()
files = [l.strip() for l in open(list_path, encoding="utf-8") if l.strip()]


def clear_scene():
    for o in list(bpy.data.objects):
        bpy.data.objects.remove(o, do_unlink=True)
    for c in list(bpy.data.collections):
        bpy.data.collections.remove(c)
    bpy.ops.outliner.orphans_purge(do_recursive=True)


def world_bm(obj):
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bm.transform(obj.matrix_world)
    bm.normal_update()
    bm.faces.ensure_lookup_table()
    return bm


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


def verify(obj, ref, grid=120):
    bm, rbm = world_bm(obj), world_bm(ref)
    bvh, rbvh = BVHTree.FromBMesh(bm), BVHTree.FromBMesh(rbm)
    cos = [v.co for v in rbm.verts]
    lo = Vector((min(c.x for c in cos), min(c.y for c in cos), min(c.z for c in cos)))
    hi = Vector((max(c.x for c in cos), max(c.y for c in cos), max(c.z for c in cos)))
    center, size = (lo + hi) / 2, (hi - lo).length
    cache = {}
    # Two-sided faces: a face and its reversed twin share positions. A ray may
    # land on either one, so a back hit on a face with a twin is not an error.
    by_pos = {}
    for f in bm.faces:
        by_pos.setdefault(frozenset(tuple(round(c, 4) for c in v.co) for v in f.verts), []).append(f.index)
    has_twin = {i for group in by_pos.values() if len(group) > 1 for i in group}
    views = [Vector((math.cos(a), math.sin(a), h)).normalized()
             for a in [k * math.pi / 12 + 0.29 for k in range(24)]
             for h in (-0.6, -0.3, 0.05, 0.2, 0.45, 1.2)] + [Vector((0.01, 0, 1))]
    hits = back = holes = 0
    back_above = holes_above = 0   # from views above the horizon: unambiguous
    back_solid = holes_solid = 0   # the same, without views through hairline cracks
    back_base = 0                  # of those: the underside of the base (points down by design)
    diag_back, diag_hole = {}, {}
    def is_base(f):
        return f.normal.z < -0.999 and all(abs(v.co.z - lo.z) < 0.02 for v in f.verts)
    def solid(tree, o, d, u, w, dist, probe=0.02):
        ok = 0
        for off in (u * probe, -u * probe, w * probe, -w * probe):
            h = tree.ray_cast(o + off, -d, size * 4)
            if h[0] is not None and abs(h[3] - dist) < 0.25:
                ok += 1
        return ok >= 3
    per_face = {}
    front_face, back_face = {}, {}   # solid views from above, per face
    for d in views:
        cam = center + d * size * 2
        u = d.orthogonal().normalized()
        w = d.cross(u).normalized()
        for a in range(grid):
            for b in range(grid):
                o = cam + (u * ((a + 0.5) / grid - 0.5) + w * ((b + 0.5) / grid - 0.5)) * size
                loc, _n, idx, dist = bvh.ray_cast(o, -d, size * 4)
                rloc, _rn, ridx, rdist = rbvh.ray_cast(o, -d, size * 4)
                if d.z < 0:  # pedestrian looking up; judged on the full reference
                    if idx is not None and not view_from_below_ok(rbvh, o, -d, dist, lo.z, size, cache):
                        idx = None
                    if ridx is not None and not view_from_below_ok(rbvh, o, -d, rdist, lo.z, size, cache):
                        ridx = None
                if ridx is not None and (idx is None or dist - rdist > 0.01):
                    holes += 1
                    holes_above += d.z >= 0
                    if d.z >= 0 and solid(rbvh, o, d, u, w, rdist):
                        holes_solid += 1
                        rec = diag_hole.setdefault(ridx, {"hits": 0, "gap": [], "res_face_n": []})
                        rec["hits"] += 1
                        if len(rec["gap"]) < 3:
                            rec["gap"].append(None if idx is None else round(dist - rdist, 3))
                            rec["res_face_n"].append(None if idx is None else round(bm.faces[idx].normal.dot(d), 2))
                if idx is None:
                    continue
                hits += 1
                if d.z >= 0 and bm.faces[idx].normal.dot(d) >= 0:
                    front_face[idx] = front_face.get(idx, 0) + 1
                if bm.faces[idx].normal.dot(d) < 0 and idx not in has_twin:
                    back += 1
                    back_above += d.z >= 0
                    if d.z >= 0 and solid(bvh, o, d, u, w, dist):
                        back_solid += 1
                        back_face[idx] = back_face.get(idx, 0) + 1
                        if is_base(bm.faces[idx]):
                            back_base += 1
                        else:
                            nx = bvh.ray_cast(loc - d * 1e-4, -d, size * 4)
                            rec = diag_back.setdefault(idx, {"hits": 0, "behind": [], "dirs": []})
                            rec["hits"] += 1
                            if len(rec["behind"]) < 3:
                                rec["behind"].append(None if nx[0] is None else [round(nx[3], 4), round(bm.faces[nx[2]].normal.dot(d), 2)])
                                rec["dirs"].append([round(x, 2) for x in d])
                    per_face[idx] = per_face.get(idx, 0) + 1
    worst = sorted(per_face.items(), key=lambda x: -x[1])[:5]
    worst = [{"face": i, "hits": c, "area": round(bm.faces[i].calc_area(), 4),
              "z_rel": round(bm.faces[i].calc_center_median().z - lo.z, 3),
              "n": [round(x, 2) for x in bm.faces[i].normal]} for i, c in worst]
    mats = [m.name if m else None for m in obj.data.materials]
    rmats = [m.name if m else None for m in ref.data.materials]
    def fdesc(b, i, mm):
        f = b.faces[i]
        c = f.calc_center_median()
        return {"face": i, "area": round(f.calc_area(), 4), "n": [round(x, 2) for x in f.normal],
                "z_rel": round(c.z - lo.z, 2), "c": [round(x, 2) for x in c],
                "edge_faces": sorted(len(e.link_faces) for e in f.edges),
                "mat": mm[f.material_index] if f.material_index < len(mm) else None}
    diag = {"back": [dict(fdesc(bm, i, mats), **r) for i, r in sorted(diag_back.items(), key=lambda x: -x[1]["hits"])[:10]],
            "hole": [dict(fdesc(rbm, i, rmats), **r) for i, r in sorted(diag_hole.items(), key=lambda x: -x[1]["hits"])[:10]]}
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
    return {"rays_hit": hits, "back_hits": back, "back_hits_from_above": back_above,
            "hole_rays_from_above": holes_above, "faces_seen_back": len(per_face),
            "back_hits_from_above_solid": back_solid, "hole_rays_from_above_solid": holes_solid,
            "back_hits_base": back_base, "diag": diag,
            "faces_mostly_back": sum(1 for i, b in back_face.items() if b >= 2 and b > front_face.get(i, 0)),
            "back_hits_mostly_back": sum(b for i, b in back_face.items() if b >= 2 and b > front_face.get(i, 0)),
            "mostly_back_list": sorted(([i, b, front_face.get(i, 0)] for i, b in back_face.items() if b >= 2 and b > front_face.get(i, 0)), key=lambda x: -x[1])[:8],
            "max_back_one_face": max(per_face.values()) if per_face else 0,
            "worst_back": worst, "hole_rays": holes, "inconsistent_edges": incons,
            "duplicate_faces": dup, "two_sided_faces": len(has_twin) // 2,
            "zero_area": sum(1 for p in obj.data.polygons if p.area < 1e-10),
            "size": [round(x, 2) for x in (hi - lo)]}


with open(out_path, "a", encoding="utf-8") as out:
    for fp in files:
        rec = {"file": fp}
        try:
            clear_scene()
            before = set(bpy.data.collections)
            t = time.time()
            bpy.ops.import_scene.collada(filepath=fp)
            rec["import_s"] = round(time.time() - t, 2)
            new = [c for c in bpy.data.collections if c not in before]
            if new:
                coll = max(new, key=lambda c: len(c.all_objects))
            else:
                coll = bpy.data.collections.new("imported")
                bpy.context.scene.collection.children.link(coll)
                for o in list(bpy.context.scene.collection.objects):
                    coll.objects.link(o)
                    bpy.context.scene.collection.objects.unlink(o)
            rec["mesh_objects"] = sum(1 for o in coll.all_objects if o.type == 'MESH')
            if rec["mesh_objects"] == 0:
                rec["status"] = "no_meshes"
            else:
                t = time.time()
                obj, stats = mod.unite_collection(coll)
                rec["unite_s"] = round(time.time() - t, 2)
                rec["stats"] = {k: v for k, v in stats.items() if k != "unrelated_details"}
                rec["unrelated_details"] = stats.get("unrelated_details", [])
                # keep the result when the reference run clears previous results
                for o in [obj, bpy.data.objects.get(stats.get("removed_object", ""))]:
                    if o is not None:
                        bpy.context.scene.collection.objects.link(o)
                        coll.objects.unlink(o)
                # GLB export + re-import check
                t = time.time()
                gp = os.path.join(GLB_DIR, os.path.splitext(os.path.relpath(fp, ROOT))[0] + ".glb")
                oname = obj.name
                mod.export_glb(bpy.context, obj, gp, bpy.context.scene.unite_model)
                obj = bpy.data.objects[oname]
                before_o = set(bpy.data.objects)
                bpy.ops.import_scene.gltf(filepath=gp)
                gl = [o for o in bpy.data.objects if o not in before_o]
                gm = [o for o in gl if o.type == 'MESH']
                tris = lambda me: sum(len(p.vertices) - 2 for p in me.polygons)
                g = {"objects": len(gl), "meshes": len(gm), "bytes": os.path.getsize(gp),
                     "tris": sum(tris(o.data) for o in gm), "united_tris": tris(obj.data)}
                if gm:
                    gpts = [o.matrix_world @ v.co for o in gm for v in o.data.vertices]
                    upts = [obj.matrix_world.inverted() @ (obj.matrix_world @ v.co) for v in obj.data.vertices]
                    g["bbox_err"] = round(max(
                        abs(min(p[i] for p in gpts) - min(p[i] for p in upts)) +
                        abs(max(p[i] for p in gpts) - max(p[i] for p in upts)) for i in range(3)), 5)
                for o in gl:
                    bpy.data.objects.remove(o, do_unlink=True)
                g["s"] = round(time.time() - t, 2)
                rec["glb"] = g
                t = time.time()
                ref, _ = mod.unite_collection(coll, remove_enclosed=False, fix_normals=False)
                bpy.context.view_layer.update()
                t = time.time()
                rec["verify"] = verify(obj, ref)
                rec["verify_s"] = round(time.time() - t, 2)
                rec["status"] = "ok"
        except Exception:
            rec["status"] = "error"
            rec["error"] = traceback.format_exc()[-1500:]
        out.write(json.dumps(rec) + "\n")
        out.flush()
