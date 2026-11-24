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

"""Operators (buttons) of the add-on."""

import os

import bmesh
import bpy
from bpy.props import CollectionProperty, EnumProperty, StringProperty
from bpy_extras.io_utils import ImportHelper

from ..batch import process_dae_file, write_report
from ..io_collada import collada_importer_available, import_dae, list_dae_files
from ..io_gltf import export_glb, export_name, export_subdir, verdict_folder
from ..scene import (append_model, arrange_models, find_layer_collection, folder_collection,
                     move_collection, scene_model_groups, target_collections)
from ..sorting import results_for_export, sort_scene
from ..tags import REMOVED_TAG, RESULT_TAG
from ..verify import VERDICT_CORRECT, VERDICT_TO_CHECK
from ..workflow import repair_collection, test_collection
from .properties import get_settings

IMPORTER_MISSING = "COLLADA importer not found: install the 'Collada Support' extension"

SCOPE_ITEMS = [
    ('ACTIVE', "Active", "Only the active collection in the Outliner (a folder "
                         "collection: every model in it)"),
    ('ALL', "All", "Every model in the scene"),
]


def _object_mode(context):
    if context.mode != 'OBJECT':
        bpy.ops.object.mode_set(mode='OBJECT')


class _Progress:
    """Mouse-cursor progress for long loops (models x share of one model)."""

    def __init__(self, context, total):
        self.wm = context.window_manager
        self.total = max(1, total)
        self.done = 0
        self.wm.progress_begin(0, 1000)

    def model(self, share):
        self.wm.progress_update(int(1000 * (self.done + share) / self.total))

    def next(self):
        self.done += 1
        self.model(0.0)

    def end(self):
        self.wm.progress_end()


def _select_results(context, results):
    if not results:
        return
    for o in context.selected_objects:
        o.select_set(False)
    for o in results:
        if o.name in context.view_layer.objects:
            o.select_set(True)
    context.view_layer.objects.active = results[-1]


def _repair_all(context, colls, s):
    """Repair (and test) `colls`; sort the scene afterwards when the test ran.
    Returns (results, counts or None)."""
    results = []
    progress = _Progress(context, len(colls))
    try:
        for coll in colls:
            obj, _stats = repair_collection(context, coll, s, progress=progress.model)
            if obj is not None:
                results.append(obj)
            progress.next()
    finally:
        progress.end()
    counts = None
    if s.test_after_unite and s.sort_after_test:
        counts = sort_scene(context, arrange=True, reserve_result=s.place_beside)
    return results, counts


def _verdict_text(results, counts):
    if counts is not None:
        return f"Correct {counts[VERDICT_CORRECT]}, to check {counts[VERDICT_TO_CHECK]}"
    correct = sum(1 for o in results if o.get("unite_verdict") == VERDICT_CORRECT)
    tested = sum(1 for o in results if o.get("unite_verdict"))
    return f"correct {correct} of {tested} tested" if tested else ""


class UNITE_OT_import_dae(bpy.types.Operator, ImportHelper):
    """Import COLLADA (.dae) files with the Collada Support importer; select files, or a folder to import every .dae in it"""
    bl_idname = "unite_model.import_dae"
    bl_label = "Import COLLADA"
    bl_options = {'REGISTER', 'UNDO'}

    filename_ext = ".dae"
    filter_glob: StringProperty(default="*.dae;*.zae", options={'HIDDEN'})
    files: CollectionProperty(type=bpy.types.OperatorFileListElement,
                              options={'HIDDEN', 'SKIP_SAVE'})
    directory: StringProperty(subtype='DIR_PATH', options={'HIDDEN', 'SKIP_SAVE'})

    def execute(self, context):
        if not collada_importer_available():
            self.report({'ERROR'}, IMPORTER_MISSING)
            return {'CANCELLED'}
        s = get_settings(context)
        names = [f.name for f in self.files if f.name]
        folder_import = not names
        if names:
            paths = [os.path.join(self.directory, n) for n in names]
        else:
            paths = list_dae_files(self.directory, s.import_subfolders)
        if not paths:
            self.report({'WARNING'}, "No .dae files selected")
            return {'CANCELLED'}
        _object_mode(context)
        done, failed, last = 0, [], None
        groups, folders = {}, {}
        for path in paths:
            try:
                coll = import_dae(context, path)
            except Exception as exc:
                failed.append(f"{os.path.relpath(path, self.directory)} ({exc})")
                continue
            done += 1
            last = coll
            rel_dir = os.path.dirname(os.path.relpath(path, self.directory))
            coll["unite_subdir"] = rel_dir
            if folder_import and s.group_by_folder:
                move_collection(coll, folder_collection(context, self.directory, rel_dir,
                                                        folders), context.scene)
            groups.setdefault(rel_dir, []).append(coll)
        imported = [c for g in groups.values() for c in g]
        if s.arrange_imports and len(imported) > 1:
            arrange_models(context, [groups[k] for k in sorted(groups)],
                           reserve_result=s.place_beside)
        elif s.arrange_imports and imported:
            append_model(context, imported[0], reserve_result=s.place_beside)
        results, counts = [], None
        if s.repair_after_import and imported:
            results, counts = _repair_all(context, imported, s)
            _select_results(context, results)
        if last is not None and counts is None:  # the Repair section then works on the new models
            top = folders.get(os.path.normpath(self.directory)) if folders else None
            layer = find_layer_collection(context.view_layer.layer_collection, top or last)
            if layer is not None:
                context.view_layer.active_layer_collection = layer
        msg = f"Imported {done} file(s)"
        if s.repair_after_import:
            msg += f", repaired {len(results)}"
            verdict = _verdict_text(results, counts)
            if verdict:
                msg += f" ({verdict})"
        if failed:
            msg += f"; failed: {', '.join(failed[:3])}"
            self.report({'WARNING'}, msg)
        else:
            self.report({'INFO'}, msg)
        return {'FINISHED'} if done else {'CANCELLED'}


class UNITE_OT_arrange(bpy.types.Operator):
    """Lay out every imported model in the scene in a grid (one block per folder collection, or Correct / To check), so no model overlaps another. Only whole models move; exports keep the original coordinates"""
    bl_idname = "unite_model.arrange"
    bl_label = "Arrange Scene"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        _object_mode(context)
        groups = scene_model_groups(context)
        count = sum(len(g) for g in groups)
        if not count:
            self.report({'WARNING'}, "No imported models in the scene")
            return {'CANCELLED'}
        arrange_models(context, groups, reserve_result=get_settings(context).place_beside)
        self.report({'INFO'}, f"Arranged {count} model(s)")
        return {'FINISHED'}


class UNITE_OT_collection(bpy.types.Operator):
    """Join all meshes of a collection into one object, remove unrelated geometry and double-sided back faces, fix normals; then test the result"""
    bl_idname = "unite_model.collection"
    bl_label = "Unite Collection"
    bl_options = {'REGISTER', 'UNDO'}

    scope: EnumProperty(name="Collections", items=SCOPE_ITEMS, default='ACTIVE')

    def execute(self, context):
        _object_mode(context)
        targets = target_collections(context, self.scope)
        if not targets:
            self.report({'WARNING'}, "No collection with meshes selected")
            return {'CANCELLED'}
        s = get_settings(context)
        results, counts = _repair_all(context, targets, s)
        _select_results(context, results)
        msg = f"Repaired {len(results)} of {len(targets)} model(s)"
        verdict = _verdict_text(results, counts)
        if verdict:
            msg += f": {verdict}"
        self.report({'INFO'}, msg)
        return {'FINISHED'} if results else {'CANCELLED'}


class UNITE_OT_test(bpy.types.Operator):
    """Test already repaired models again (rays from outside) and mark them Correct or To check. Needs the source objects (not deleted)"""
    bl_idname = "unite_model.test"
    bl_label = "Test Models"
    bl_options = {'REGISTER', 'UNDO'}

    scope: EnumProperty(name="Collections", items=SCOPE_ITEMS, default='ALL')

    def execute(self, context):
        _object_mode(context)
        targets = [c for c in target_collections(context, self.scope)
                   if any(o.get(RESULT_TAG) for o in c.objects)]
        if not targets:
            self.report({'WARNING'}, "No repaired models to test")
            return {'CANCELLED'}
        s = get_settings(context)
        verdicts = []
        progress = _Progress(context, len(targets))
        try:
            for coll in targets:
                verdict, _reasons, _metrics = test_collection(context, coll, s, progress.model)
                verdicts.append(verdict)
                progress.next()
        finally:
            progress.end()
        if s.sort_after_test:
            sort_scene(context, arrange=True, reserve_result=s.place_beside)
        correct = verdicts.count(VERDICT_CORRECT)
        self.report({'INFO'}, f"Tested {len(verdicts)} model(s): correct {correct}, "
                              f"to check {len(verdicts) - correct}")
        return {'FINISHED'}


class UNITE_OT_sort(bpy.types.Operator):
    """Move the tested models into the collections 'Correct' and 'To check' and lay them out as two blocks"""
    bl_idname = "unite_model.sort"
    bl_label = "Sort Correct / To check"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        _object_mode(context)
        counts = sort_scene(context, arrange=True,
                            reserve_result=get_settings(context).place_beside)
        if not counts[VERDICT_CORRECT] and not counts[VERDICT_TO_CHECK]:
            self.report({'WARNING'}, "No tested models in the scene (run Test Models)")
            return {'CANCELLED'}
        msg = f"Correct {counts[VERDICT_CORRECT]}, to check {counts[VERDICT_TO_CHECK]}"
        if counts[None]:
            msg += f", not tested {counts[None]}"
        self.report({'INFO'}, msg)
        return {'FINISHED'}


class UNITE_OT_select_problems(bpy.types.Operator):
    """Edit the active repaired model with the faces the test saw from the back selected"""
    bl_idname = "unite_model.select_problems"
    bl_label = "Select Problem Faces"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        obj = context.active_object
        return obj is not None and obj.get(RESULT_TAG) and obj.type == 'MESH'

    def execute(self, context):
        obj = context.active_object
        ids = set(obj.get("unite_test_back_faces", []))
        if not ids:
            self.report({'INFO'}, "The test saw no face from the back"
                        + ("" if obj.get("unite_verdict") else " (model not tested)"))
            return {'CANCELLED'}
        _object_mode(context)
        for p in obj.data.polygons:
            p.select = p.index in ids
        for e in obj.data.edges:
            e.select = False
        for v in obj.data.vertices:
            v.select = False
        for p in obj.data.polygons:
            if p.select:
                for vi in p.vertices:
                    obj.data.vertices[vi].select = True
        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        context.view_layer.objects.active = obj
        bpy.ops.object.mode_set(mode='EDIT')
        bm = bmesh.from_edit_mesh(obj.data)
        bm.select_mode = {'FACE'}
        bm.select_flush_mode()
        bmesh.update_edit_mesh(obj.data)
        context.tool_settings.mesh_select_mode = (False, False, True)
        self.report({'INFO'}, f"{len(ids)} face(s) seen from the back are selected")
        return {'FINISHED'}


class UNITE_OT_toggle_removed(bpy.types.Operator):
    """Show or hide the geometry that was removed as unrelated"""
    bl_idname = "unite_model.toggle_removed"
    bl_label = "Show Removed Parts"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        objs = [o for o in context.view_layer.objects if o.get(REMOVED_TAG)]
        if not objs:
            self.report({'INFO'}, "No removed parts in the scene")
            return {'CANCELLED'}
        show = any(o.hide_get() for o in objs)
        for o in objs:
            o.hide_set(not show)
        self.report({'INFO'}, ("Showing" if show else "Hiding") + f" {len(objs)} object(s)")
        return {'FINISHED'}


class UNITE_OT_check_view(bpy.types.Operator):
    """Toggle Face Orientation together with Backface Culling in this 3D view.
With culling, only the side a face points to is drawn: a wrong normal shows as a hole, red as a wrong side; two-sided faces show correctly from both sides"""
    bl_idname = "unite_model.check_view"
    bl_label = "Normals Check View"

    @classmethod
    def poll(cls, context):
        return context.space_data is not None and context.space_data.type == 'VIEW_3D'

    def execute(self, context):
        space = context.space_data
        on = not (space.overlay.show_face_orientation and space.shading.show_backface_culling)
        space.overlay.show_face_orientation = on
        space.shading.show_backface_culling = on
        return {'FINISHED'}


class UNITE_OT_export_glb(bpy.types.Operator):
    """Export repaired models as glTF 2.0 binary (.glb), one file per model"""
    bl_idname = "unite_model.export_glb"
    bl_label = "Export GLB"
    bl_options = {'REGISTER'}

    scope: EnumProperty(
        name="Export",
        items=[
            ('CORRECT', "Correct", "Repaired models that passed the test"),
            ('TO_CHECK', "To check", "Repaired models that did not pass the test"),
            ('ALL', "All", "Every repaired model in the scene"),
            ('SELECTED', "Selected", "Repaired models that are selected"),
        ],
        default='ALL',
    )

    def execute(self, context):
        s = get_settings(context)
        out_dir = bpy.path.abspath(s.export_dir)
        if not s.export_dir:
            self.report({'ERROR'}, "Choose the export folder first")
            return {'CANCELLED'}
        objs = results_for_export(context, self.scope)
        if not objs:
            what = {'CORRECT': "correct", 'TO_CHECK': "'to check'",
                    'SELECTED': "selected"}.get(self.scope, "repaired")
            self.report({'WARNING'}, f"No {what} models to export")
            return {'CANCELLED'}
        written, errors, used = [], 0, set()
        for obj in objs:
            sub = export_subdir(obj)
            if s.export_verdict_folders:
                sub = os.path.join(verdict_folder(obj), sub)
            name = export_name(obj)
            base, k = name, 2
            while os.path.join(sub, name).lower() in used:
                name = f"{base}_{k}"
                k += 1
            used.add(os.path.join(sub, name).lower())
            try:
                written.append(export_glb(context, obj, os.path.join(out_dir, sub, name + ".glb"), s))
            except Exception as exc:
                errors += 1
                self.report({'ERROR'}, f"{obj.name}: {exc}")
        self.report({'WARNING'} if errors else {'INFO'},
                    f"Exported {len(written)} file(s) to {out_dir}"
                    + (f", {errors} failed" if errors else ""))
        return {'FINISHED'} if written else {'CANCELLED'}


class UNITE_OT_batch(bpy.types.Operator):
    """Import every .dae in the input folder, repair and test it and export it as .glb into the export folder; writes unite_report.csv (Esc cancels)"""
    bl_idname = "unite_model.batch"
    bl_label = "Run Batch"

    _timer = None

    def execute(self, context):
        s = get_settings(context)
        if not collada_importer_available():
            self.report({'ERROR'}, IMPORTER_MISSING)
            return {'CANCELLED'}
        self.in_dir = bpy.path.abspath(s.batch_input_dir)
        self.out_dir = bpy.path.abspath(s.export_dir)
        if not s.batch_input_dir or not os.path.isdir(self.in_dir):
            self.report({'ERROR'}, "Choose an existing input folder")
            return {'CANCELLED'}
        if not s.export_dir:
            self.report({'ERROR'}, "Choose the export folder")
            return {'CANCELLED'}
        self.files = list_dae_files(self.in_dir, s.batch_subfolders)
        if not self.files:
            self.report({'WARNING'}, "No .dae files in the input folder")
            return {'CANCELLED'}
        _object_mode(context)
        os.makedirs(self.out_dir, exist_ok=True)
        self.rows = []
        self.index = 0
        wm = context.window_manager
        self._timer = wm.event_timer_add(0.05, window=context.window)
        wm.modal_handler_add(self)
        wm.progress_begin(0, len(self.files))
        return {'RUNNING_MODAL'}

    def _finish(self, context, cancelled=False):
        wm = context.window_manager
        wm.event_timer_remove(self._timer)
        wm.progress_end()
        context.workspace.status_text_set(None)
        report = write_report(self.rows, self.out_dir)
        bad = sum(1 for r in self.rows if r["status"] != "ok")
        correct = sum(1 for r in self.rows if r.get("test") == VERDICT_CORRECT)
        to_check = sum(1 for r in self.rows if r.get("test") == VERDICT_TO_CHECK)
        msg = (f"{'Cancelled' if cancelled else 'Done'}: {len(self.rows) - bad} exported "
               f"(correct {correct}, to check {to_check}), {bad} failed. Report: {report}")
        s = get_settings(context)
        if s.batch_keep_in_scene and s.test_after_unite and s.sort_after_test:
            sort_scene(context, arrange=True, reserve_result=s.place_beside)
        self.report({'WARNING'} if bad or cancelled else {'INFO'}, msg)
        return {'CANCELLED'} if cancelled else {'FINISHED'}

    def modal(self, context, event):
        if event.type == 'ESC':
            return self._finish(context, cancelled=True)
        if event.type != 'TIMER':
            return {'PASS_THROUGH'}
        if self.index >= len(self.files):
            return self._finish(context)
        s = get_settings(context)
        path = self.files[self.index]
        rel = os.path.relpath(path, self.in_dir)
        context.workspace.status_text_set(
            f"Unite batch {self.index + 1}/{len(self.files)}: {rel}   (Esc to cancel)")
        self.rows.append(process_dae_file(context, path, self.out_dir, os.path.splitext(rel)[0],
                                          s, keep_in_scene=s.batch_keep_in_scene))
        self.index += 1
        context.window_manager.progress_update(self.index)
        return {'RUNNING_MODAL'}


classes = (
    UNITE_OT_import_dae,
    UNITE_OT_arrange,
    UNITE_OT_collection,
    UNITE_OT_test,
    UNITE_OT_sort,
    UNITE_OT_select_problems,
    UNITE_OT_toggle_removed,
    UNITE_OT_check_view,
    UNITE_OT_export_glb,
    UNITE_OT_batch,
)
