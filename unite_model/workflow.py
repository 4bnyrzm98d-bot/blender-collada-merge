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

"""Repair of one model with the panel settings: placement, verification, verdict."""

import time
from contextlib import contextmanager

import bpy
from mathutils import Vector

from .pipeline import source_objects, unite_bmesh, unite_collection
from .scene import place_beside, top_level
from .tags import (RESULT_TAG, VERDICT_KEY, VERDICT_REASON_KEY, is_generated)
from .verify import VERDICT_TO_CHECK, judge, mesh_bmesh, verify_bmesh


def _pipeline_kwargs(s):
    """Arguments of unite_bmesh / unite_collection from the panel settings."""
    return dict(
        merge_distance=s.merge_distance,
        fix_normals=s.fix_normals,
        remove_enclosed=s.remove_enclosed,
        on_ground=s.on_ground,
        two_sided=s.two_sided,
        remove_unrelated=s.remove_unrelated,
        keep_buildings=s.keep_buildings,
        keep_distance=s.keep_distance,
        keep_removed=s.keep_removed,
        remove_ground=s.remove_ground,
        close_gap=s.gap_width if s.close_gaps else 0.0,
        fill_holes=s.fill_holes,
    )


@contextmanager
def at_home(context, collection):
    """Temporarily put a model moved by "Arrange" back to its imported position.

    The repair and the test then do not depend on the layout (float precision
    far from the origin) and the result mesh keeps the original coordinates.
    Yields True when the model had been moved.
    """
    arranged = [(o, o.location.copy()) for o in top_level(collection) if "unite_home" in o]
    if arranged:
        for o, _loc in arranged:
            o.location = o["unite_home"]
        context.view_layer.update()
    try:
        yield bool(arranged)
    finally:
        if arranged:
            for o, loc in arranged:
                if o.name in bpy.data.objects:
                    o.location = loc
            context.view_layer.update()


def set_verdict(collection, obj, verdict, reasons, metrics=None):
    """Store the test result on the model collection and on its result."""
    reason = "; ".join(reasons)
    for target in (collection, obj):
        if target is None:
            continue
        target[VERDICT_KEY] = verdict
        target[VERDICT_REASON_KEY] = reason
    if obj is not None and metrics is not None:
        for key in ("back_hits", "hole_rays", "faces_seen_back", "faces_missing",
                    "duplicate_faces", "zero_area", "inconsistent_edges", "rays_hit"):
            obj["unite_test_" + key] = metrics[key]
        obj["unite_test_back_faces"] = metrics["back_face_ids"]


def test_result(collection, obj, s, progress=None):
    """Test the result `obj` of `collection` against a reference built from the
    source objects at their current position (call inside at_home).

    Returns (verdict, reasons, metrics) and stores them (set_verdict).
    """
    t0 = time.time()
    kwargs = _pipeline_kwargs(s)
    kwargs.update(fix_normals=False, remove_enclosed=False, two_sided=False, keep_removed=False)
    ref_bm = bm = None
    try:
        ref_bm, _materials, _stats, _removed = unite_bmesh(collection, **kwargs)
        if ref_bm is None:
            raise RuntimeError("source objects are missing (deleted?)")
        bm = mesh_bmesh(obj.data)  # mesh coordinates = imported world coordinates
        metrics = verify_bmesh(bm, ref_bm, grid=s.test_grid, progress=progress)
    except Exception as exc:
        set_verdict(collection, obj, VERDICT_TO_CHECK, [f"test failed: {exc}"])
        return VERDICT_TO_CHECK, [f"test failed: {exc}"], None
    finally:
        for b in (ref_bm, bm):
            if b is not None:
                b.free()
    verdict, reasons = judge(metrics, max_back_hits=s.test_max_back_hits,
                             max_holes=s.test_max_holes)
    metrics["seconds"] = round(time.time() - t0, 2)
    set_verdict(collection, obj, verdict, reasons, metrics)
    return verdict, reasons, metrics


def result_of(collection):
    return next((o for o in collection.objects if o.get(RESULT_TAG)), None)


def test_collection(context, collection, s, progress=None):
    """Re-test an already repaired model (button "Test Models")."""
    obj = result_of(collection)
    if obj is None:
        set_verdict(collection, None, VERDICT_TO_CHECK, ["not repaired"])
        return VERDICT_TO_CHECK, ["not repaired"], None
    if not source_objects(collection):
        reasons = ["cannot test: source objects were deleted"]
        set_verdict(collection, obj, VERDICT_TO_CHECK, reasons)
        return VERDICT_TO_CHECK, reasons, None
    with at_home(context, collection):
        return test_result(collection, obj, s, progress)


def repair_collection(context, collection, s, progress=None):
    """Run the pipeline with the panel settings `s`, test and place the result.

    The test runs before the source objects are hidden or deleted, because
    the reference is built from them.
    """
    with at_home(context, collection) as arranged:
        obj, stats = unite_collection(collection, smooth_angle=s.smooth_angle,
                                      **_pipeline_kwargs(s))
        if obj is not None and s.test_after_unite:
            verdict, reasons, metrics = test_result(collection, obj, s, progress)
            stats["test"] = verdict
            stats["test_reason"] = "; ".join(reasons)
            if metrics is not None:
                stats["test_back_hits"] = metrics["back_hits"]
                stats["test_hole_rays"] = metrics["hole_rays"]
                stats["test_seconds"] = metrics["seconds"]
    if obj is None:
        set_verdict(collection, None, VERDICT_TO_CHECK,
                    [f"repair failed: {stats.get('error', 'unknown error')}"])
        return None, stats
    removed = bpy.data.objects.get(stats.get("removed_object", ""))
    offset = Vector(collection.get("unite_offset", (0.0, 0.0, 0.0)))
    if arranged:
        for o in (obj, removed):
            if o is not None:
                o.location = offset
        context.view_layer.update()
    if collection.get("unite_source"):
        obj["unite_source"] = collection["unite_source"]
    if collection.get("unite_subdir") is not None:
        obj["unite_subdir"] = collection["unite_subdir"]
    others = [o for o in collection.all_objects if o is not obj and not is_generated(o)]
    if s.place_beside and s.source_action != 'DELETE':
        meshes = [o for o in others if o.type == 'MESH']
        if meshes:
            stats["offset_x"] = round(place_beside(obj, meshes, s.gap, context.scene), 4)
    if removed is not None:
        removed.location = obj.location
    if s.source_action == 'HIDE':
        for o in others:
            o.hide_set(True)
            o.hide_render = True
    elif s.source_action == 'DELETE':
        for o in others:
            bpy.data.objects.remove(o, do_unlink=True)
    for key, value in stats.items():
        if isinstance(value, (int, float, str)):
            obj["unite_" + key] = value
    context.view_layer.update()  # later placements read matrix_world
    return obj, stats
