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

import os

import bpy

from .scene import arrange_sections, move_collection, target_collections
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


FOLDER_KEY = "unite_sort_folder"   # folder path a sorted folder collection stands for
LABEL_KEY = "unite_sort_label"     # the name it should have


def model_folder(coll):
    """Import folder of a model as a list of names: the subfolder of a folder
    import ("1.NAM", or "a/b" -> ["a", "b"]), otherwise the folder of the file."""
    rel = coll.get("unite_subdir") or ""
    parts = [p for p in rel.replace("\\", "/").split("/") if p not in ("", ".")]
    if not parts and coll.get("unite_source"):
        name = os.path.basename(os.path.dirname(coll["unite_source"]))
        parts = [name] if name else []
    return parts


def _folder_label(name, verdict):
    # collection names are unique in a .blend: the same folder in "To check"
    # needs a different name than in "Correct"
    return name if verdict == VERDICT_CORRECT else f"{name} (to check)"


def folder_collection_in(root, parts, verdict):
    """Nested collections for the folder `parts` inside the verdict collection."""
    parent = root
    for k in range(len(parts)):
        key = "/".join(parts[:k + 1])
        coll = next((c for c in parent.children if c.get(FOLDER_KEY) == key), None)
        if coll is None:
            label = _folder_label(parts[k], verdict)
            coll = bpy.data.collections.new(label)
            coll[FOLDER_KEY] = key
            coll[LABEL_KEY] = label
            coll.color_tag = root.color_tag
            parent.children.link(coll)
        parent = coll
    return parent


def _remove_empty_folders(context):
    """Delete folder collections (of the import or of the sorting) left empty."""
    def walk(parent):
        for child in list(parent.children):
            walk(child)
            if ((child.get("unite_folder") or child.get(FOLDER_KEY))
                    and not child.all_objects and not child.children):
                bpy.data.collections.remove(child)
    walk(context.scene.collection)


def _restore_labels():
    """Give sorted folder collections their intended names once the import
    folder collections of the same name are gone (Blender adds ".001" while
    a name is taken)."""
    for coll in bpy.data.collections:
        label = coll.get(LABEL_KEY)
        if label and coll.name != label and bpy.data.collections.get(label) is None:
            coll.name = label


def _order_children(parent):
    """Outliner order = link order: relink the children alphabetically."""
    children = sorted(parent.children, key=lambda c: c.name.lower())
    if [c.name for c in children] == [c.name for c in parent.children]:
        return
    for c in children:
        parent.children.unlink(c)
    for c in children:
        parent.children.link(c)


def sort_scene(context, arrange=True, reserve_result=False, spacing=10.0):
    """Move every tested model collection into "Correct" or "To check", each
    time into a collection named after its import folder.

    Untested models stay where they are. With `arrange` the models are laid
    out as two sections side by side (Correct left, To check right), one block
    per folder. Returns {verdict: count, None: count}.
    """
    found = {VERDICT_CORRECT: [], VERDICT_TO_CHECK: []}
    untested = 0
    for coll in target_collections(context, 'ALL'):
        verdict = model_verdict(coll)
        if verdict is None:
            untested += 1
            continue
        found[verdict].append(coll)
    sections = []
    for verdict in (VERDICT_CORRECT, VERDICT_TO_CHECK):
        colls = found[verdict]
        if not colls:
            continue
        root = verdict_collection(context, verdict)
        groups = {}
        for coll in sorted(colls, key=lambda c: (model_folder(c), c.get("unite_source", c.name))):
            parent = folder_collection_in(root, model_folder(coll), verdict)
            move_collection(coll, parent, context.scene)
            groups.setdefault(parent.name, []).append(coll)
        sections.append(list(groups.values()))
    _remove_empty_folders(context)
    _restore_labels()
    for verdict in (VERDICT_CORRECT, VERDICT_TO_CHECK):
        if found[verdict]:
            root = verdict_collection(context, verdict)
            _order_children(root)
            for folder in root.children:
                if folder.get(FOLDER_KEY):
                    _order_children(folder)
    if arrange and sections:
        arrange_sections(context, sections, spacing=spacing, reserve_result=reserve_result)
    return {VERDICT_CORRECT: len(found[VERDICT_CORRECT]),
            VERDICT_TO_CHECK: len(found[VERDICT_TO_CHECK]), None: untested}


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
