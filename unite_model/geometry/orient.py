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


"""Outward orientation of faces, removal of hidden faces, two-sided faces."""

from collections import defaultdict, deque

import bmesh
from mathutils import Vector
from mathutils.bvhtree import BVHTree

from .build import weld
from .rays import free_directions, hemisphere_dirs, outside_hits, outside_views
from ..tags import TWO_SIDED_LAYER


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
    (views through hairline cracks do not count, see outside_hits).

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
    dirs = hemisphere_dirs(ray_count)

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
        front = free_directions(bvh, c, n, dirs, far, eps, ground)
        back = free_directions(bvh, c, -n, dirs, far, eps, ground)
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
    dense = hemisphere_dirs(enclosed_ray_count)
    for idx in list(enclosed):
        f = bm.faces[idx]
        c = f.calc_center_median()
        n = f.normal
        front = free_directions(bvh, c, n, dense, far, eps, ground)
        back = free_directions(bvh, c, -n, dense, far, eps, ground)
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
    seen = outside_hits(bm, bvh, center, size,
                         outside_views(view_count, upper_only=on_ground),
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
