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


"""Close narrow slits and fill missing planar faces (gables, floors)."""

from collections import defaultdict

from mathutils import Vector
from mathutils.bvhtree import BVHTree

from .build import weld
from ..tags import FLOOR_LAYER


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
