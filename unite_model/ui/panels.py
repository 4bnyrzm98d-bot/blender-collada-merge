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

"""Sidebar panels in the 3D view (category 'Unite')."""

import bpy

from ..io_collada import collada_importer_available
from ..tags import RESULT_TAG, VERDICT_KEY, VERDICT_REASON_KEY
from ..verify import VERDICT_CORRECT
from .operators import (UNITE_OT_arrange, UNITE_OT_batch, UNITE_OT_check_view,
                        UNITE_OT_collection, UNITE_OT_export_glb, UNITE_OT_import_dae,
                        UNITE_OT_select_problems, UNITE_OT_sort, UNITE_OT_test,
                        UNITE_OT_toggle_removed)
from .properties import get_settings


class _SubPanel:
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Unite"
    bl_parent_id = "UNITE_PT_main"


class UNITE_PT_main(bpy.types.Panel):
    bl_label = "Unite Model"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Unite"

    def draw(self, context):
        layout = self.layout
        obj = context.active_object
        if obj is None or not obj.get(RESULT_TAG):
            layout.label(text="Import a .dae, then repair, test and export", icon='INFO')
            return
        box = layout.box()
        box.label(text=obj.name, icon='MESH_DATA')
        col = box.column(align=True)
        col.label(text=f"Objects {obj.get('unite_source_objects', '?')} -> 1, "
                       f"faces {obj.get('unite_faces_in', '?')} -> {obj.get('unite_faces_out', '?')}")
        col.label(text=f"Flipped {obj.get('unite_flipped', 0)}, "
                       f"unrelated parts removed {obj.get('unite_unrelated_parts', 0)}")
        verdict = obj.get(VERDICT_KEY)
        if verdict is None:
            col.label(text="Not tested", icon='QUESTION')
            return
        ok = verdict == VERDICT_CORRECT
        row = col.row()
        row.alert = not ok
        row.label(text="Test: Correct" if ok else "Test: To check",
                  icon='CHECKMARK' if ok else 'ERROR')
        if "unite_test_back_hits" in obj:
            col.label(text=f"Back hits {obj['unite_test_back_hits']}, "
                           f"from below {obj.get('unite_test_bottom_back_hits', 0)}, "
                           f"hole rays {obj['unite_test_hole_rays']}")
        reason = obj.get(VERDICT_REASON_KEY)
        if reason:
            for part in reason.split("; ")[:4]:
                col.label(text=part)
        if obj.get("unite_test_back_faces"):
            box.operator(UNITE_OT_select_problems.bl_idname, icon='RESTRICT_SELECT_OFF')


class UNITE_PT_import(_SubPanel, bpy.types.Panel):
    bl_label = "1  Import COLLADA"

    def draw(self, context):
        layout = self.layout
        s = get_settings(context)
        if not collada_importer_available():
            box = layout.box()
            box.alert = True
            box.label(text="COLLADA importer missing", icon='ERROR')
            box.label(text="Install the 'Collada Support' extension")
        layout.operator(UNITE_OT_import_dae.bl_idname, text="Import .dae...", icon='IMPORT')
        col = layout.column(align=True)
        col.prop(s, "repair_after_import")
        col.prop(s, "import_subfolders")
        col.prop(s, "group_by_folder")
        col.prop(s, "arrange_imports")
        layout.operator(UNITE_OT_arrange.bl_idname, icon='MOD_ARRAY')


class UNITE_PT_repair(_SubPanel, bpy.types.Panel):
    bl_label = "2  Repair"

    def draw(self, context):
        layout = self.layout
        s = get_settings(context)
        coll = context.view_layer.active_layer_collection.collection
        layout.label(text=f"Active: {coll.name}", icon='OUTLINER_COLLECTION')
        col = layout.column(align=True)
        op = col.operator(UNITE_OT_collection.bl_idname, text="Unite Active Collection")
        op.scope = 'ACTIVE'
        op = col.operator(UNITE_OT_collection.bl_idname, text="Unite All Collections")
        op.scope = 'ALL'
        row = layout.row(align=True)
        row.operator(UNITE_OT_toggle_removed.bl_idname, icon='HIDE_OFF')
        row.operator(UNITE_OT_check_view.bl_idname, text="Check Normals", icon='NORMALS_FACE')

        box = layout.box()
        box.prop(s, "show_repair_options",
                 icon='TRIA_DOWN' if s.show_repair_options else 'TRIA_RIGHT', emboss=False)
        if s.show_repair_options:
            col = box.column()
            col.prop(s, "remove_unrelated")
            sub = col.column()
            sub.active = s.remove_unrelated
            sub.prop(s, "keep_buildings")
            sub.prop(s, "keep_distance")
            sub.prop(s, "remove_ground")
            sub.prop(s, "keep_removed")
            col.separator()
            col.prop(s, "merge_distance")
            col.prop(s, "close_gaps")
            sub = col.column()
            sub.active = s.close_gaps
            sub.prop(s, "gap_width")
            col.prop(s, "fill_holes")
            col.prop(s, "fix_normals")
            col.prop(s, "remove_enclosed")
            col.prop(s, "two_sided")
            col.prop(s, "on_ground")
            col.prop(s, "smooth_angle")
            col.separator()
            col.prop(s, "source_action")
            col.prop(s, "place_beside")
            sub = col.column()
            sub.active = s.place_beside
            sub.prop(s, "gap")


class UNITE_PT_test(_SubPanel, bpy.types.Panel):
    bl_label = "3  Test"

    def draw(self, context):
        layout = self.layout
        s = get_settings(context)
        col = layout.column(align=True)
        col.prop(s, "test_after_unite")
        col.prop(s, "sort_after_test")
        col = layout.column(align=True)
        op = col.operator(UNITE_OT_test.bl_idname, text="Test All Models", icon='VIEWZOOM')
        op.scope = 'ALL'
        op = col.operator(UNITE_OT_test.bl_idname, text="Test Active Collection")
        op.scope = 'ACTIVE'
        layout.operator(UNITE_OT_sort.bl_idname, icon='OUTLINER_COLLECTION')
        box = layout.box()
        box.prop(s, "show_test_options",
                 icon='TRIA_DOWN' if s.show_test_options else 'TRIA_RIGHT', emboss=False)
        if s.show_test_options:
            col = box.column()
            col.prop(s, "test_grid")
            col.prop(s, "test_max_back_hits")
            col.prop(s, "test_max_bottom_hits")
            col.prop(s, "test_max_holes")


class UNITE_PT_export(_SubPanel, bpy.types.Panel):
    bl_label = "4  Export glTF 2.0 (.glb)"

    def draw(self, context):
        layout = self.layout
        s = get_settings(context)
        layout.prop(s, "export_dir")
        col = layout.column(align=True)
        col.prop(s, "export_verdict_folders")
        row = col.row()
        row.prop(s, "export_materials")
        row.prop(s, "export_yup")
        col = layout.column(align=True)
        row = col.row(align=True)
        op = row.operator(UNITE_OT_export_glb.bl_idname, text="Export Correct", icon='CHECKMARK')
        op.scope = 'CORRECT'
        op = row.operator(UNITE_OT_export_glb.bl_idname, text="Export To check", icon='ERROR')
        op.scope = 'TO_CHECK'
        row = col.row(align=True)
        op = row.operator(UNITE_OT_export_glb.bl_idname, text="Export All", icon='EXPORT')
        op.scope = 'ALL'
        op = row.operator(UNITE_OT_export_glb.bl_idname, text="Export Selected",
                          icon='RESTRICT_SELECT_OFF')
        op.scope = 'SELECTED'


class UNITE_PT_batch(_SubPanel, bpy.types.Panel):
    bl_label = "Batch: Folder to GLB"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        s = get_settings(context)
        layout.prop(s, "batch_input_dir")
        layout.label(text="Output: export folder (section 4)")
        col = layout.column(align=True)
        col.prop(s, "batch_subfolders")
        col.prop(s, "batch_keep_in_scene")
        layout.operator(UNITE_OT_batch.bl_idname, icon='FILE_REFRESH')


classes = (
    UNITE_PT_main,
    UNITE_PT_import,
    UNITE_PT_repair,
    UNITE_PT_test,
    UNITE_PT_export,
    UNITE_PT_batch,
)
