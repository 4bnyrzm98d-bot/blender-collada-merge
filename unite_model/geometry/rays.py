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


"""Ray casting helpers: free directions around a face and views from outside."""

import math
from collections import defaultdict

from mathutils import Vector


def hemisphere_dirs(count):
    """Evenly spread directions on the +Z hemisphere (Fibonacci spiral)."""
    golden = math.pi * (3.0 - math.sqrt(5.0))
    dirs = []
    for i in range(count):
        z = 1.0 - (i + 0.5) / count
        r = math.sqrt(max(0.0, 1.0 - z * z))
        t = golden * i
        dirs.append(Vector((r * math.cos(t), r * math.sin(t), z)))
    return dirs


def free_directions(bvh, origin, normal, dirs, far, eps, ground=None):
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


def outside_views(count, upper_only=True):
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


def outside_hits(bm, bvh, center, size, views, grid, ground=None,
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
