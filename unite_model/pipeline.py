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


"""The repair pipeline: collection in, one united object out (no UI, no scene layout)."""

import math
from collections import defaultdict

import bmesh
import bpy
from mathutils import Vector

from .geometry.build import exact_area, build_bmesh, remove_back_faces, remove_degenerate, weld
from .geometry.fill import close_gaps, fill_missing_faces
from .geometry.orient import make_two_sided, orient_outward
from .geometry.unrelated import describe_parts, find_unrelated
from .tags import FLOOR_LAYER, REMOVED_TAG, RESULT_TAG, TWO_SIDED_LAYER, is_generated


def _collection_cameras(collection):
    """(origin, forward) of cameras imported with the model (SketchUp view)."""
    cams = []
    for o in collection.all_objects:
        if o.type == 'CAMERA':
            mw = o.matrix_world
            forward = (mw.to_3x3() @ Vector((0.0, 0.0, -1.0))).normalized()
            cams.append((mw.translation.copy(), forward))
    return cams


def set_area_weighted_normals(me, smooth_angle):
    """Shading normals weighted by face area (like the Weighted Normal
    modifier). With plain averaging a large flat roof that shares a vertex
    with a few small faces of a curved gable gets a tilted normal there and
    shows a dark smear across the whole roof. Weighted by area, a big face
    keeps its own normal, while curves made of similar small faces stay
    smooth. Faces meeting at more than `smooth_angle` stay sharp.
    """
    cos_limit = math.cos(smooth_angle)
    normals = [p.normal.copy() for p in me.polygons]
    areas = [p.area for p in me.polygons]
    vert_faces = defaultdict(list)
    for p in me.polygons:
        for v in p.vertices:
            vert_faces[v].append(p.index)
    loop_normals = [None] * len(me.loops)
    for p in me.polygons:
        n0 = normals[p.index]
        for li in p.loop_indices:
            v = me.loops[li].vertex_index
            acc = Vector()
            for g in vert_faces[v]:
                if normals[g].dot(n0) >= cos_limit:
                    acc += normals[g] * areas[g]
            loop_normals[li] = acc.normalized() if acc.length > 1e-12 else n0
    me.normals_split_custom_set(loop_normals)


def _bm_to_object(bm, materials, name, collection, smooth_angle):
    """Turn a bmesh into a new object; keeps only materials still in use."""
    zero = [f for f in bm.faces if exact_area(f) < 1e-9]
    if zero:  # zero-area faces (float32) are invisible and break exporters
        bmesh.ops.delete(bm, geom=zero, context='FACES_ONLY')
    used = sorted({f.material_index for f in bm.faces})
    remap = {old: new for new, old in enumerate(used)}
    for f in bm.faces:
        f.material_index = remap[f.material_index]
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    for old in used:
        me.materials.append(materials[old])
    me.validate()
    if smooth_angle is not None and me.polygons:
        me.shade_smooth()
        me.set_sharp_from_angle(angle=smooth_angle)
        set_area_weighted_normals(me, smooth_angle)
    me.update()
    obj = bpy.data.objects.new(name, me)
    collection.objects.link(obj)
    return obj


def _base_name(collection):
    name = collection.name
    for ext in (".dae", ".zae", ".DAE"):
        if ext in name:
            name = name.rsplit(ext, 1)[0] + name.rsplit(ext, 1)[1]
    return name


def remove_previous_results(collection):
    """Delete objects an earlier run created in this collection."""
    for o in list(collection.objects):
        if is_generated(o):
            me = o.data
            bpy.data.objects.remove(o, do_unlink=True)
            if me is not None and me.users == 0:
                bpy.data.meshes.remove(me)


def source_objects(collection):
    """The imported mesh objects of a model (results of earlier runs excluded)."""
    return [o for o in collection.all_objects
            if o.type == 'MESH' and not is_generated(o)]


def unite_bmesh(collection, merge_distance=1e-4, fix_normals=True,
                remove_enclosed=True, on_ground=True, two_sided=True, ray_count=48,
                remove_unrelated=True, keep_buildings='MAIN', keep_distance=1.0,
                keep_removed=True, remove_ground=True, close_gap=0.4, fill_holes=True):
    """Run the geometry pipeline on all meshes of `collection`.

    Nothing in the scene changes. Returns (bm, materials, stats, removed_bm):
    the united mesh in world space, the material of each material index, the
    statistics, and the faces removed as unrelated (None unless keep_removed).
    The caller frees both bmeshes. Returns (None, None, stats, None) when the
    collection has no meshes.
    """
    sources = source_objects(collection)
    if not sources:
        return None, None, {"error": "no mesh objects"}, None

    bm, materials, skipped = build_bmesh(sources)
    stats = {
        "source_objects": len(sources),
        "faces_in": len(bm.faces),
        "verts_in": len(bm.verts),
        "invalid_faces_skipped": skipped,
    }

    removed, back = remove_back_faces(bm, merge_distance)
    stats["back_faces_removed"] = removed
    stats["back_material"] = (materials[back].name
                              if back is not None and materials[back] else None)
    stats["verts_welded"] = weld(bm, merge_distance)
    stats["degenerate_removed"] = remove_degenerate(bm, merge_distance)

    # Unrelated geometry goes before orientation: a backdrop blocks the rays
    # and a map underlay would define the ground level.
    removed_bm = None
    stats["unrelated_parts"] = 0
    stats["unrelated_faces"] = 0
    stats["unrelated_area"] = 0.0
    if remove_unrelated and bm.faces:
        doomed, info = find_unrelated(
            bm, _collection_cameras(collection), keep_distance=keep_distance,
            keep_buildings=keep_buildings, remove_ground=remove_ground)
        stats["main_building_by"] = info["main_by"]
        stats["buildings_found"] = info["buildings"]
        stats["unrelated_parts"] = len(info["parts"])
        stats["unrelated_faces"] = len(doomed)
        stats["unrelated_area"] = round(sum(p["area"] for p in info["parts"]), 2)
        stats["unrelated_summary"] = describe_parts(info["parts"])
        stats["unrelated_details"] = info["parts"]
        if doomed:
            bm.faces.ensure_lookup_table()
            if keep_removed:
                bm.faces.index_update()
                removed_bm = bm.copy()
                removed_bm.faces.index_update()
                removed_bm.faces.ensure_lookup_table()
                bmesh.ops.delete(removed_bm, geom=[f for f in removed_bm.faces
                                                   if f.index not in doomed],
                                 context='FACES')
            bmesh.ops.delete(bm, geom=[bm.faces[i] for i in doomed], context='FACES')

    if close_gap and bm.faces:
        count, area = close_gaps(bm, close_gap, merge_distance)
        stats["gaps_closed"] = count
        stats["gap_area"] = round(area, 2)

    if fill_holes and bm.faces:
        count, area = fill_missing_faces(bm)
        stats["holes_filled"] = count
        stats["hole_area"] = round(area, 2)

    if fix_normals and bm.faces:
        stats.update(orient_outward(bm, ray_count, remove_enclosed, on_ground))
    floor_layer = bm.faces.layers.int.get(FLOOR_LAYER)
    if floor_layer is not None:
        # a filled floor closes the house from below: it always points down
        up = [f for f in bm.faces if f[floor_layer] and f.normal.z > 0.0]
        if up:
            bmesh.ops.reverse_faces(bm, faces=up)
            bm.normal_update()
        bm.faces.layers.int.remove(floor_layer)

    stats["degenerate_removed"] += remove_degenerate(bm, merge_distance)

    # Last geometry step: nothing may weld after the twins exist.
    if two_sided:
        stats["two_sided_faces"] = make_two_sided(bm)
    else:
        layer = bm.faces.layers.int.get(TWO_SIDED_LAYER)
        if layer is not None:
            bm.faces.layers.int.remove(layer)
    return bm, materials, stats, removed_bm


def unite_collection(collection, merge_distance=1e-4, fix_normals=True,
                     remove_enclosed=True, on_ground=True, two_sided=True,
                     smooth_angle=math.radians(30.0), ray_count=48,
                     remove_unrelated=True, keep_buildings='MAIN',
                     keep_distance=1.0, keep_removed=True, remove_ground=True,
                     close_gap=0.4, fill_holes=True):
    """Create one mesh object from all meshes in `collection`.

    The source objects are not modified. Faces that do not belong to the
    house (remove_unrelated) go to a hidden object "<name>_removed" when
    keep_removed is set, so they can be reviewed. Returns (new_object, stats).
    """
    bm, materials, stats, removed_bm = unite_bmesh(
        collection, merge_distance=merge_distance, fix_normals=fix_normals,
        remove_enclosed=remove_enclosed, on_ground=on_ground, two_sided=two_sided,
        ray_count=ray_count, remove_unrelated=remove_unrelated,
        keep_buildings=keep_buildings, keep_distance=keep_distance,
        keep_removed=keep_removed, remove_ground=remove_ground,
        close_gap=close_gap, fill_holes=fill_holes)
    if bm is None:
        return None, stats

    remove_previous_results(collection)
    name = _base_name(collection) + "_united"
    obj = _bm_to_object(bm, materials, name, collection, smooth_angle)
    bm.free()
    obj[RESULT_TAG] = True

    if removed_bm is not None:
        rem = _bm_to_object(removed_bm, materials, _base_name(collection) + "_removed",
                            collection, None)
        removed_bm.free()
        rem[REMOVED_TAG] = True
        rem.hide_render = True
        try:
            rem.hide_set(True)
        except RuntimeError:  # collection excluded from the view layer
            pass
        obj["unite_removed_object"] = rem.name
        stats["removed_object"] = rem.name

    stats["faces_out"] = len(obj.data.polygons)
    stats["verts_out"] = len(obj.data.vertices)
    stats["materials_out"] = len(obj.data.materials)
    return obj, stats
