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

"""Automatic test of a repaired model ("Correct" / "To check").

The result is "photographed" from outside with parallel rays and compared
with a reference: the same model united WITHOUT orientation and WITHOUT
removing hidden faces (so nothing visible is missing in it).

* back hit - a ray from outside lands on the back side of a result face, i.e.
  the face shows red in Face Orientation. Faces with a reversed twin
  (two-sided faces) and the underside of the base (points down on purpose)
  do not count.
* hole - a ray hits the reference but passes the result (or lands more than
  1 cm deeper): a visible face was deleted.

Only views from above the horizon are used: views from below enter the open
bottom of a model and see things nobody can see. A hit counts only when four
rays shifted by 2 cm hit at a similar depth ("solid" view), so views through
hairline cracks between SketchUp faces are ignored. The view directions differ
from the ones the repair uses, so the test does not just confirm itself.

This is the same test as tools/batch_verify.py (metrics *_from_above_solid).
"""

import math

import bmesh
from mathutils import Vector
from mathutils.bvhtree import BVHTree

from .geometry.build import exact_area

# 24 azimuths x 4 elevations above the horizon + one view from the top
VIEWS = [Vector((math.cos(a), math.sin(a), h)).normalized()
         for a in [k * math.pi / 12 + 0.29 for k in range(24)]
         for h in (0.05, 0.2, 0.45, 1.2)] + [Vector((0.01, 0.0, 1.0))]

VERDICT_CORRECT = "CORRECT"
VERDICT_TO_CHECK = "TO_CHECK"


def _bbox(bm):
    cos = [v.co for v in bm.verts]
    lo = Vector((min(c.x for c in cos), min(c.y for c in cos), min(c.z for c in cos)))
    hi = Vector((max(c.x for c in cos), max(c.y for c in cos), max(c.z for c in cos)))
    return lo, hi


def _position_key(face):
    return frozenset(tuple(round(c, 4) for c in v.co) for v in face.verts)


def topology_counts(bm):
    """Duplicate faces (same vertices), zero-area faces, edges whose two faces
    disagree on winding (invisible, reported only)."""
    keys, dup = set(), 0
    for f in bm.faces:
        k = frozenset(v.index for v in f.verts)
        dup += k in keys
        keys.add(k)
    zero = sum(1 for f in bm.faces if exact_area(f) < 1e-9)
    incons = sum(1 for e in bm.edges if len(e.link_faces) == 2 and not e.is_contiguous)
    return {"duplicate_faces": dup, "zero_area": zero, "inconsistent_edges": incons}


def verify_bmesh(bm, ref_bm, grid=120, probe=0.02, depth_tol=0.25, progress=None):
    """Compare the result `bm` with the reference `ref_bm` (same coordinates).

    Returns a dict of metrics. `grid` x `grid` rays are cast per view
    (97 views); 120 is what the batch tests used.
    """
    bm.faces.ensure_lookup_table()
    bm.normal_update()
    ref_bm.normal_update()
    bvh = BVHTree.FromBMesh(bm)
    rbvh = BVHTree.FromBMesh(ref_bm)
    lo, hi = _bbox(ref_bm)
    center, size = (lo + hi) / 2, max((hi - lo).length, 1e-3)
    far = size * 4.0

    by_pos = {}
    for f in bm.faces:
        by_pos.setdefault(_position_key(f), []).append(f.index)
    has_twin = {i for group in by_pos.values() if len(group) > 1 for i in group}
    base = {f.index for f in bm.faces
            if f.normal.z < -0.999 and all(abs(v.co.z - lo.z) < 0.02 for v in f.verts)}
    normals = [f.normal.copy() for f in bm.faces]

    def solid(tree, origin, d, u, w, dist):
        ok = 0
        for off in (u * probe, -u * probe, w * probe, -w * probe):
            hit = tree.ray_cast(origin + off, -d, far)
            if hit[0] is not None and abs(hit[3] - dist) < depth_tol:
                ok += 1
        return ok >= 3

    rays_hit = back = back_base = holes = 0
    back_faces, front_faces, hole_faces = {}, {}, {}
    step = 1.0 / grid
    for n, d in enumerate(VIEWS):
        cam = center + d * size * 2.0
        u = d.orthogonal().normalized()
        w = d.cross(u).normalized()
        nd = -d
        for a in range(grid):
            ua = cam + u * (((a + 0.5) * step - 0.5) * size)
            for b in range(grid):
                o = ua + w * (((b + 0.5) * step - 0.5) * size)
                _loc, _n, idx, dist = bvh.ray_cast(o, nd, far)
                _rloc, _rn, ridx, rdist = rbvh.ray_cast(o, nd, far)
                if ridx is not None and (idx is None or dist - rdist > 0.01):
                    if solid(rbvh, o, d, u, w, rdist):
                        holes += 1
                        hole_faces[ridx] = hole_faces.get(ridx, 0) + 1
                if idx is None:
                    continue
                rays_hit += 1
                if normals[idx].dot(d) >= 0.0:
                    front_faces[idx] = front_faces.get(idx, 0) + 1
                elif idx not in has_twin and solid(bvh, o, d, u, w, dist):
                    if idx in base:
                        back_base += 1
                    else:
                        back += 1
                        back_faces[idx] = back_faces.get(idx, 0) + 1
        if progress is not None:
            progress((n + 1) / len(VIEWS))

    worst = sorted(back_faces.items(), key=lambda x: -x[1])
    metrics = {
        "rays_hit": rays_hit,
        "back_hits": back,
        "back_hits_base": back_base,
        "faces_seen_back": len(back_faces),
        "hole_rays": holes,
        "faces_missing": len(hole_faces),
        "max_back_one_face": worst[0][1] if worst else 0,
        "back_face_ids": [i for i, _c in worst[:200]],
        "worst_back": [{"face": i, "hits": c, "front": front_faces.get(i, 0),
                        "area": round(bm.faces[i].calc_area(), 4)} for i, c in worst[:5]],
        "grid": grid,
    }
    metrics.update(topology_counts(bm))
    return metrics


def mesh_bmesh(mesh):
    bm = bmesh.new()
    bm.from_mesh(mesh)
    bm.verts.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    return bm


def judge(metrics, max_back_hits=0, max_holes=0, allow_inconsistent=True):
    """(verdict, reasons) from the metrics of verify_bmesh.

    Strict by default: on the 146 test houses every model with even one
    solid back hit or hole ray turned out to have a real (if small) defect in
    nearly all cases, so 0 / 0 matches a manual review best. No duplicate and
    no zero-area faces either. Inconsistent edges are invisible and only
    reported.
    """
    reasons = []
    if metrics["back_hits"] > max_back_hits:
        reasons.append(f"{metrics['back_hits']} back hits on {metrics['faces_seen_back']} face(s)")
    if metrics["hole_rays"] > max_holes:
        reasons.append(f"{metrics['hole_rays']} hole rays ({metrics['faces_missing']} missing face(s))")
    if metrics["duplicate_faces"]:
        reasons.append(f"{metrics['duplicate_faces']} duplicate face(s)")
    if metrics["zero_area"]:
        reasons.append(f"{metrics['zero_area']} zero-area face(s)")
    if not allow_inconsistent and metrics["inconsistent_edges"]:
        reasons.append(f"{metrics['inconsistent_edges']} inconsistent edge(s)")
    return (VERDICT_TO_CHECK if reasons else VERDICT_CORRECT), reasons
