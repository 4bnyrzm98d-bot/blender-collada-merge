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


"""Collections and layout of several models in one scene."""

import math
import os

import bpy
from mathutils import Vector

from .tags import REMOVED_TAG, RESULT_TAG, is_generated


def _world_bbox(obj):
    """(min, max) of the object's bounding box in world space."""
    pts = [obj.matrix_world @ Vector(c) for c in obj.bound_box]
    lo = Vector((min(p.x for p in pts), min(p.y for p in pts), min(p.z for p in pts)))
    hi = Vector((max(p.x for p in pts), max(p.y for p in pts), max(p.z for p in pts)))
    return lo, hi


def _boxes_overlap(a_lo, a_hi, b_lo, b_hi):
    return (a_lo.x < b_hi.x and b_lo.x < a_hi.x and
            a_lo.y < b_hi.y and b_lo.y < a_hi.y)


def place_beside(obj, sources, gap, scene):
    """Move `obj` along +X next to its source model, onto free space.

    The step is the model width plus `gap`; it keeps stepping while the new
    position would overlap another visible mesh in the scene (footprint check).
    """
    src_boxes = [_world_bbox(o) for o in sources]
    lo = Vector((min(b[0].x for b in src_boxes), min(b[0].y for b in src_boxes), 0.0))
    hi = Vector((max(b[1].x for b in src_boxes), max(b[1].y for b in src_boxes), 0.0))
    step = (hi.x - lo.x) + gap
    obstacles = [_world_bbox(o) for o in scene.objects
                 if o.type == 'MESH' and o is not obj and o.visible_get()]
    shift = step
    for _ in range(50):
        new_lo = lo + Vector((shift, 0.0, 0.0))
        new_hi = hi + Vector((shift, 0.0, 0.0))
        if not any(_boxes_overlap(new_lo, new_hi, b_lo, b_hi) for b_lo, b_hi in obstacles):
            break
        shift += step
    obj.location.x += shift
    return shift


def top_level(coll):
    members = set(coll.objects)
    return [o for o in coll.objects if o.parent is None or o.parent not in members]


def folder_collection(context, root_dir, rel_dir, cache):
    """Collection for a folder of the import (nested like the folders).

    root_dir itself becomes the top collection; a model in root/a/b goes into
    the collection 'b' inside 'a' inside the top one. Reuses a collection made
    by an earlier import of the same folder.
    """
    key = os.path.normpath(os.path.join(root_dir, rel_dir))
    if key in cache:
        return cache[key]
    if rel_dir in ("", "."):
        parent = context.scene.collection
        label = os.path.basename(os.path.normpath(root_dir)) or "Import"
    else:
        parent = folder_collection(context, root_dir, os.path.dirname(rel_dir), cache)
        label = os.path.basename(rel_dir)
    coll = next((c for c in parent.children if c.get("unite_folder") == key), None)
    if coll is None:
        coll = bpy.data.collections.new(label)
        coll["unite_folder"] = key
        parent.children.link(coll)
    cache[key] = coll
    return coll


def move_collection(coll, new_parent, scene):
    if coll.name in new_parent.children:
        return
    for parent in [scene.collection] + list(bpy.data.collections):
        if coll.name in parent.children:
            parent.children.unlink(coll)
    new_parent.children.link(coll)


def _model_box(coll, reserve_result):
    """XY footprint of an imported model, its repaired result included."""
    boxes = [_world_bbox(o) for o in coll.objects
             if o.type == 'MESH' and not o.get(REMOVED_TAG)]
    if not boxes:
        return None
    lo = Vector((min(b[0].x for b in boxes), min(b[0].y for b in boxes), 0.0))
    hi = Vector((max(b[1].x for b in boxes), max(b[1].y for b in boxes), 0.0))
    if reserve_result:  # room for the result that "place beside" will add later
        hi.x += (hi.x - lo.x) + 2.0
    return lo, hi


def shift_model(coll, delta):
    """Move a whole imported model; only top-level objects are moved, so the
    geometry and the internal hierarchy stay exactly as imported."""
    for o in top_level(coll):
        # exact imported position, used by the repair (results are rebuilt)
        if "unite_home" not in o and not is_generated(o):
            o["unite_home"] = tuple(o.location)
        o.location += delta
    coll["unite_offset"] = Vector(coll.get("unite_offset", (0.0, 0.0, 0.0))) + delta


LAYOUT_KEY = "unite_layout"


ROW_LENGTH = 8


def has_result(coll):
    return any(o.get(RESULT_TAG) for o in coll.objects)


def _free_row_top(context, exclude, spacing):
    """Y where a new row can start: below every mesh not in `exclude`."""
    boxes = [_world_bbox(o) for o in context.scene.objects
             if o.type == 'MESH' and not any(o.name in c.objects for c in exclude)]
    return min((b[0].y for b in boxes), default=0.0) - (spacing if boxes else 0.0)


def append_model(context, coll, spacing=10.0, reserve_result=False):
    """Place one newly imported model next to the previous one.

    Imports one by one fill rows of ROW_LENGTH models; a new row starts below
    everything in the scene. The row state lives in scene[LAYOUT_KEY].
    """
    scene = context.scene
    box = _model_box(coll, reserve_result and not has_result(coll))
    if box is None:
        return
    lo, hi = box
    state = scene.get(LAYOUT_KEY)
    state = dict(state) if state else None
    if not state or state["count"] >= ROW_LENGTH:
        state = {"top": _free_row_top(context, [coll], spacing),
                 "x": 0.0, "depth": 0.0, "count": 0}
    shift_model(coll, Vector((state["x"] - lo.x, state["top"] - hi.y, 0.0)))
    state["x"] += (hi.x - lo.x) + spacing
    state["depth"] = max(state["depth"], hi.y - lo.y)
    state["count"] += 1
    scene[LAYOUT_KEY] = state
    context.view_layer.update()


def _layout_groups(groups, x0, y, spacing, reserve_result):
    """Place `groups` (lists of model collections) as blocks of rows starting
    at (x0, y) and going along -Y. Returns the largest x used."""
    max_x = x0
    for group in groups:
        items = [(c, _model_box(c, reserve_result and not has_result(c))) for c in group]
        items = [(c, b) for c, b in items if b is not None]
        if not items:
            continue
        cols = max(1, math.ceil(math.sqrt(len(items))))
        for r in range(0, len(items), cols):
            row = items[r:r + cols]
            depth = max(b[1].y - b[0].y for _c, b in row)
            x = x0
            for coll, (lo, hi) in row:
                shift_model(coll, Vector((x - lo.x, y - hi.y, 0.0)))
                x += (hi.x - lo.x) + spacing
            max_x = max(max_x, x - spacing)
            y -= depth + spacing
        y -= spacing  # extra gap between folders
    return max_x


def arrange_sections(context, sections, spacing=10.0, reserve_result=False):
    """Lay models out in sections side by side (e.g. Correct | To check).

    `sections` is a list of sections, a section a list of groups (one per
    folder), a group a list of model collections. Every folder becomes a block
    of rows; the blocks of a section go along -Y, the next section starts to
    the right of the previous one. Everything starts below the rest of the
    scene.
    """
    scene = context.scene
    placed = [c for sec in sections for g in sec for c in g]
    y = _free_row_top(context, placed, spacing)
    x0 = 0.0
    for section in sections:
        if not any(section):
            continue
        x0 = _layout_groups(section, x0, y, spacing, reserve_result) + 4.0 * spacing
    if LAYOUT_KEY in scene:  # the next single import starts a new row
        del scene[LAYOUT_KEY]
    # matrix_world of moved objects is only refreshed by a depsgraph update;
    # the repair reads it right after this
    context.view_layer.update()


def arrange_models(context, groups, spacing=10.0, reserve_result=False):
    """Lay imported models out in a grid instead of on top of each other.

    `groups` is a list of lists of collections (one list per folder). Every
    folder becomes a block of rows; blocks follow each other along -Y, below
    whatever else is in the scene.
    """
    arrange_sections(context, [groups], spacing, reserve_result)


def scene_model_sections(context):
    """Every imported model in the scene: one section per top-level collection
    (e.g. "Correct", "To check", an imported folder), in it one group per
    parent collection, in Outliner order."""
    sections = []
    for top in context.scene.collection.children:
        groups = {}

        def walk(parent):
            for child in parent.children:
                if child.get("unite_source"):
                    groups.setdefault(parent.name, []).append(child)
                else:
                    walk(child)
        if top.get("unite_source"):
            groups[context.scene.collection.name] = [top]
        else:
            walk(top)
        if groups:
            sections.append([sorted(g, key=lambda c: c.get("unite_source", c.name))
                             for g in groups.values()])
    return sections


def _has_own_meshes(coll):
    return any(o.type == 'MESH' and not is_generated(o) for o in coll.objects)


def _model_collections(root):
    """Collections below `root` (itself included) that hold a model's meshes.

    One import = one collection; folder collections only hold collections."""
    found = [root] if _has_own_meshes(root) else []
    for child in root.children:
        found += _model_collections(child)
    return found


def target_collections(context, scope):
    if scope == 'ACTIVE':
        coll = context.view_layer.active_layer_collection.collection
        if coll is context.scene.collection:
            return []
        return _model_collections(coll)  # a folder collection: every model in it
    return [c for child in context.scene.collection.children
            for c in _model_collections(child)]


def find_layer_collection(layer, coll):
    if layer.collection is coll:
        return layer
    for child in layer.children:
        found = find_layer_collection(child, coll)
        if found is not None:
            return found
    return None
