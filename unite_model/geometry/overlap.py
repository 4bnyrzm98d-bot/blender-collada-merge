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

"""Coplanar faces lying on top of each other (z-fighting).

SketchUp files often contain the same surface twice with a different
triangulation, or a small face drawn onto a bigger one. After welding and
orientation both point the same way and the renderer cannot decide which one
is in front, so the surface flickers between their materials (z-fighting).

A face counts as overlapped where a point inside it lies inside another face
that points the same way and lies in the same plane (within 0.2 mm). Edge
neighbours do not count: a point near a shared edge is never INSIDE the
neighbour.
"""

import math

import bmesh
from bmesh.geometry import intersect_face_point
from mathutils.bvhtree import BVHTree


def face_samples(face, inset=0.01, spacing=0.05, max_samples=200):
    """Points inside `face`: a triangular grid (about 5 cm apart, at most
    `max_samples`), pulled `inset` towards the centre so none lies on an edge."""
    vs = [v.co for v in face.verts]
    c = face.calc_center_median()
    pts = []
    step = max(spacing, math.sqrt(max(face.calc_area(), 1e-12) / max_samples))
    for k in range(1, len(vs) - 1):
        a, b, d = vs[0], vs[k], vs[k + 1]
        n = max(3, min(30, math.ceil(max((b - a).length, (d - a).length, (d - b).length) / step)))
        for i in range(1, n):
            for j in range(1, n - i):
                p = a + (b - a) * (i / n) + (d - a) * (j / n)
                q = c - p
                if q.length > 1e-9:
                    p = p + q * min(0.5, inset / q.length)
                pts.append(p)
    return pts or [c]


def find_overlaps(bm, plane_tol=2e-4, same_dir=0.999, only=None):
    """{face: (share covered, set of covering faces)} for every face (of
    `only`, if given) that has at least one sample point inside another
    same-direction coplanar face."""
    bm.faces.ensure_lookup_table()
    bm.normal_update()
    bvh = BVHTree.FromBMesh(bm)
    faces = bm.faces
    found = {}
    for f in (faces if only is None else [x for x in only if x.is_valid]):
        if f.calc_area() < 1e-8:
            continue
        n = f.normal
        pts = face_samples(f)
        covered, covers = 0, set()
        for p in pts:
            for co, _nn, idx, _dist in bvh.find_nearest_range(p, plane_tol):
                if idx == f.index:
                    continue
                g = faces[idx]
                if g.normal.dot(n) < same_dir:
                    continue
                d = co - p
                if (d - n * d.dot(n)).length < 1e-6:  # p projects INTO g, not onto its edge
                    covered += 1
                    covers.add(g)
                    break
        if covered:
            found[f] = (covered / len(pts), covers)
    return found


def _bbox(verts, pad=0.0):
    cos = [v.co for v in verts]
    lo = tuple(min(c[i] for c in cos) - pad for i in range(3))
    hi = tuple(max(c[i] for c in cos) + pad for i in range(3))
    return lo, hi


def _bbox_overlap(a, b):
    return all(a[0][i] <= b[1][i] and b[0][i] <= a[1][i] for i in range(3))


def _inside_any(point, faces):
    return any(g.is_valid and intersect_face_point(g, point) for g in faces)


def _remove_pass(bm, plane_tol, only=None):
    found = find_overlaps(bm, plane_tol, only=only)
    cut, area = 0, 0.0
    for f in sorted(found, key=lambda x: -x.calc_area()):
        if not f.is_valid:
            continue
        covers = [g for g in found[f][1] if g.is_valid]
        if not covers:
            continue
        n = f.normal.copy()
        if not any(_inside_any(p, covers) for p in face_samples(f)):
            continue
        origin = f.verts[0].co.copy()
        keep = set(covers)

        def in_plane(x):
            # bisect_plane also returns faces around the split edges: keep only
            # the pieces of f (same plane, same direction), never a cover
            return (x.is_valid and x not in keep and x.normal.dot(n) > 0.999
                    and abs((x.calc_center_median() - origin).dot(n)) < plane_tol)

        # Cut f along every edge line of its covers: each piece then lies wholly
        # inside or wholly outside each cover - also along f's own border, which
        # the (inset) samples do not see.
        pieces = [f]
        for g in covers:
            glo, ghi = _bbox(g.verts, 1e-4)
            for e in g.edges:
                a, b = e.verts[0].co.copy(), e.verts[1].co.copy()
                no = n.cross(b - a)
                if no.length < 1e-9:
                    continue
                # only pieces near g need its outline: big faces under many
                # small ones would otherwise be cut by every line of every cover
                near = [p for p in pieces if p.is_valid and _bbox_overlap(_bbox(p.verts), (glo, ghi))]
                if not near:
                    continue
                near_set = set(near)
                far = [p for p in pieces if p.is_valid and p not in near_set]
                geom = list({x for p in near for x in (p, *p.edges, *p.verts)})
                res = bmesh.ops.bisect_plane(bm, geom=geom, dist=1e-6,
                                             plane_co=a, plane_no=no.normalized())
                pieces = far + [x for x in res["geom"]
                                if isinstance(x, bmesh.types.BMFace) and in_plane(x)]
        drop = [p for p in pieces if in_plane(p) and _inside_any(p.calc_center_median(), covers)]
        if drop:
            cut += 1
            area += sum(p.calc_area() for p in drop)
            bmesh.ops.delete(bm, geom=drop, context='FACES_ONLY')
    return cut, area


def remove_coplanar_overlaps(bm, plane_tol=2e-4, passes=3, only=None):
    """Remove the part of every face that lies on another same-direction face.

    Faces are handled from the largest: each is cut along the outlines of the
    faces covering it and the pieces lying on them are deleted (a face covered
    completely disappears). The covering faces stay whole, so the surface keeps
    exactly one layer - and a small face drawn onto a wall (a sign, a door, a
    panel) stays visible while the wall gets a hole under it. Cuts split the
    shared edges, so neighbours get the new vertices too (no T-junctions).
    A few passes catch overlaps the first sampling missed. With `only`, just
    those faces are cut (where they lie on others), the others stay whole.
    Returns (faces changed, area removed).
    """
    total_cut, total_area = 0, 0.0
    for _ in range(passes):
        cut, area = _remove_pass(bm, plane_tol, only)
        if only is not None:
            break
        total_cut += cut
        total_area += area
        if not cut:
            break
    bmesh.ops.delete(bm, geom=[v for v in bm.verts if not v.link_faces], context='VERTS')
    bm.faces.ensure_lookup_table()
    return total_cut, total_area


def overlapping_faces(bm, plane_tol=2e-4, min_area=1e-3):
    """Indices of faces that still lie on another same-direction face over at
    least `min_area` (10 cm2; smaller slivers do not flicker visibly), for the
    test. Reversed two-sided twins point the other way: not counted."""
    return sorted(f.index for f, (share, _c) in find_overlaps(bm, plane_tol).items()
                  if share * f.calc_area() >= min_area)
