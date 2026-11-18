# Read-only analysis of imported COLLADA collections in the open Blender scene.
# Run it through the Blender MCP tool (execute_blender_code): paste the whole
# file as `code`. It changes nothing in the scene and fills `result`.
#
# What it measures, per collection that contains meshes:
#   - mesh objects, triangles vs other faces, materials, negative scale
#   - vertices total vs unique (are triangles welded or not?)
#   - double-sided pairs: faces with the same vertex positions, their normals
#     (opposite = front/back pair) and the material that sits on every back
#   - how a plain "Merge by Distance" threshold would collapse geometry
#   - model size and edge lengths, to choose a safe weld distance

import bpy
from mathutils import kdtree
from collections import Counter, defaultdict

THRESHOLDS = (1e-6, 1e-5, 1e-4, 1e-3, 1e-2)


def collect(coll):
    verts, faces = [], []          # world positions; (vertex ids, material, normal)
    for o in coll.all_objects:
        if o.type != 'MESH' or o.get("unite_model_result"):
            continue
        me, mw = o.data, o.matrix_world
        n3 = mw.to_3x3()
        base = len(verts)
        verts.extend(mw @ v.co for v in me.vertices)
        for p in me.polygons:
            mat = None
            if me.materials and p.material_index < len(me.materials) and me.materials[p.material_index]:
                mat = me.materials[p.material_index].name
            faces.append((tuple(base + i for i in p.vertices), mat, (n3 @ p.normal).normalized()))
    return verts, faces


def cluster(verts, dist):
    tree = kdtree.KDTree(len(verts))
    for i, v in enumerate(verts):
        tree.insert(v, i)
    tree.balance()
    rep = list(range(len(verts)))
    for i, v in enumerate(verts):
        if rep[i] != i:
            continue
        for _co, j, _d in tree.find_range(v, dist):
            if j > i and rep[j] == j:
                rep[j] = i
    return rep


def analyze(coll):
    meshes = [o for o in coll.all_objects if o.type == 'MESH' and not o.get("unite_model_result")]
    verts, faces = collect(coll)
    if not faces:
        return {"mesh_objects": len(meshes), "note": "no faces"}
    info = {
        "mesh_objects": len(meshes),
        "negative_scale_objects": sum(1 for o in meshes if o.matrix_world.determinant() < 0),
        "faces": len(faces),
        "face_sizes": dict(Counter(len(f[0]) for f in faces)),
        "verts_total": len(verts),
        "materials": len({f[1] for f in faces}),
    }
    xs = [v.x for v in verts]; ys = [v.y for v in verts]; zs = [v.z for v in verts]
    info["size_m"] = [round(max(xs) - min(xs), 3), round(max(ys) - min(ys), 3), round(max(zs) - min(zs), 3)]

    lengths = sorted((verts[f[0][i]] - verts[f[0][(i + 1) % len(f[0])]]).length
                     for f in faces for i in range(len(f[0])))
    info["edge_len_min"] = round(lengths[0], 6)
    info["edge_len_median"] = round(lengths[len(lengths) // 2], 4)

    # Merge-by-distance effects at several thresholds
    effects = {}
    for t in THRESHOLDS:
        rep = cluster(verts, t)
        keys = Counter()
        degenerate = 0
        for ids, _m, _n in faces:
            k = frozenset(rep[i] for i in ids)
            if len(k) < 3:
                degenerate += 1
            else:
                keys[k] += 1
        effects[str(t)] = {
            "unique_verts": sum(1 for i, r in enumerate(rep) if r == i),
            "faces_collapsing": degenerate,
            "duplicate_faces": sum(c - 1 for c in keys.values() if c > 1),
        }
    info["merge_effects"] = effects

    # Double-sided pairs at a safe tolerance
    rep = cluster(verts, 1e-5)
    groups = defaultdict(list)
    for f in faces:
        k = frozenset(rep[i] for i in f[0])
        if len(k) >= 3:
            groups[k].append(f)
    pairs = [g for g in groups.values() if len(g) == 2]
    mat_in_pairs = Counter()
    opposite = 0
    for a, b in pairs:
        if a[2].dot(b[2]) < -0.99:
            opposite += 1
        for m in {a[1], b[1]}:
            mat_in_pairs[m] += 1
    info["duplicate_pairs"] = len(pairs)
    info["pairs_opposite_normals"] = opposite
    info["share_of_faces_in_pairs"] = round(2 * len(pairs) / len(faces), 3)
    if pairs:
        mat, hits = mat_in_pairs.most_common(1)[0]
        info["likely_back_material"] = mat if hits >= 0.5 * len(pairs) else None
        info["back_material_in_pairs_share"] = round(hits / len(pairs), 3)
    return info


scene = bpy.context.scene
result = {
    "blender": bpy.app.version_string,
    "unit_system": scene.unit_settings.system,
    "scale_length": scene.unit_settings.scale_length,
    "collections": {c.name: analyze(c) for c in scene.collection.children
                    if any(o.type == 'MESH' for o in c.all_objects)},
}
