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

"""Sort the models in the scene into the collections "Correct" and "To check"
by the result of the automatic test, and pick results for export by verdict."""

import bpy

from .scene import arrange_models, move_collection, target_collections
from .tags import RESULT_TAG, SORT_KEY, VERDICT_KEY
from .verify import VERDICT_CORRECT, VERDICT_TO_CHECK

SORT_COLLECTIONS = {
    VERDICT_CORRECT: ("Correct", 'COLOR_04'),     # green
    VERDICT_TO_CHECK: ("To check", 'COLOR_01'),   # red
}


def verdict_collection(context, verdict):
    """The top-level collection for `verdict`, created on first use."""
    for coll in context.scene.collection.children:
        if coll.get(SORT_KEY) == verdict:
            return coll
    name, color = SORT_COLLECTIONS[verdict]
    coll = bpy.data.collections.new(name)
    coll[SORT_KEY] = verdict
    coll.color_tag = color
    context.scene.collection.children.link(coll)
    return coll


def model_verdict(coll):
    """CORRECT, TO_CHECK or None (not tested yet)."""
    verdict = coll.get(VERDICT_KEY)
    if verdict in SORT_COLLECTIONS:
        return verdict
    for o in coll.objects:
        if o.get(RESULT_TAG) and o.get(VERDICT_KEY) in SORT_COLLECTIONS:
            return o[VERDICT_KEY]
    return None


def _remove_empty_folders(context):
    """Delete folder collections of the import that the sorting emptied."""
    def walk(parent):
        for child in list(parent.children):
            walk(child)
            if child.get("unite_folder") and not child.all_objects and not child.children:
                bpy.data.collections.remove(child)
    walk(context.scene.collection)


def sort_scene(context, arrange=True, reserve_result=False, spacing=10.0):
    """Move every tested model collection into "Correct" or "To check".

    Untested models stay where they are. With `arrange` the two groups are
    laid out as two blocks (Correct first). Returns {verdict: count, None: count}.
    """
    groups = {VERDICT_CORRECT: [], VERDICT_TO_CHECK: []}
    untested = 0
    for coll in target_collections(context, 'ALL'):
        verdict = model_verdict(coll)
        if verdict is None:
            untested += 1
            continue
        groups[verdict].append(coll)
    for verdict, colls in groups.items():
        if not colls:
            continue
        parent = verdict_collection(context, verdict)
        for coll in sorted(colls, key=lambda c: c.get("unite_source", c.name)):
            move_collection(coll, parent, context.scene)
    _remove_empty_folders(context)
    blocks = [sorted(groups[v], key=lambda c: c.get("unite_source", c.name))
              for v in (VERDICT_CORRECT, VERDICT_TO_CHECK) if groups[v]]
    if arrange and blocks:
        arrange_models(context, blocks, spacing=spacing, reserve_result=reserve_result)
    return {VERDICT_CORRECT: len(groups[VERDICT_CORRECT]),
            VERDICT_TO_CHECK: len(groups[VERDICT_TO_CHECK]), None: untested}


def results_for_export(context, scope):
    """Repaired objects to export: SELECTED, ALL, CORRECT or TO_CHECK.

    CORRECT / TO_CHECK go by the test result stored on the object, so it does
    not matter where the model sits in the Outliner. Untested results belong to
    neither.
    """
    objs = [o for o in context.view_layer.objects if o.get(RESULT_TAG)]
    if scope == 'SELECTED':
        return [o for o in objs if o.select_get()]
    if scope in (VERDICT_CORRECT, VERDICT_TO_CHECK):
        return [o for o in objs if o.get(VERDICT_KEY) == scope]
    return objs
