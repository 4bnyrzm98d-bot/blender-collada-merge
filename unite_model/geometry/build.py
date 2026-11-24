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


"""Collect meshes into one bmesh, remove double-sided back faces, weld, clean slivers."""

from collections import Counter, defaultdict

import bmesh
from mathutils import kdtree


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


def exact_area(face):
    """Face area in double precision (bmesh works in float32, so the three
    points of a collinear triangle far from the origin get a tiny area)."""
    pts = [tuple(float(c) for c in v.co) for v in face.verts]
    nx = ny = nz = 0.0
    for (x1, y1, z1), (x2, y2, z2) in zip(pts, pts[1:] + pts[:1]):
        nx += (y1 - y2) * (z1 + z2)
        ny += (z1 - z2) * (x1 + x2)
        nz += (x1 - x2) * (y1 + y2)
    return 0.5 * (nx * nx + ny * ny + nz * nz) ** 0.5
