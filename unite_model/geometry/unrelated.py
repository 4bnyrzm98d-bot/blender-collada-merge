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


"""Find geometry that does not belong to the house (sheets, far parts, neighbours, ground strips)."""

from collections import defaultdict

from mathutils import Vector
from mathutils.bvhtree import BVHTree


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


def describe_parts(parts, limit=6):
    """Short text for reports: 'sheet 3522m2 @24.8m; far 3.5m2 @10.1m'."""
    txt = [f"{p['kind']} {p['area']:g}m2 @{p['distance']:g}m"
           for p in sorted(parts, key=lambda p: -p["area"])[:limit]]
    if len(parts) > limit:
        txt.append(f"+{len(parts) - limit} more")
    return "; ".join(txt)
