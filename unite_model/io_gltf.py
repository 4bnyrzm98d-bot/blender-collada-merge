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


"""glTF 2.0 (.glb) export of repaired objects."""

import os

import bpy
from mathutils import Matrix

from .tags import VERDICT_KEY


def export_name(obj):
    source = obj.get("unite_source")
    if source:
        name = os.path.splitext(os.path.basename(source))[0]
    else:
        name = obj.name
        if name.endswith("_united"):
            name = name[:-len("_united")]
    return bpy.path.clean_name(name).strip("_") or "model"


VERDICT_FOLDERS = {"CORRECT": "Correct", "TO_CHECK": "To_check"}


def verdict_folder(obj):
    """Export subfolder for the test result of `obj` (Correct / To_check / Untested)."""
    return VERDICT_FOLDERS.get(obj.get(VERDICT_KEY), "Untested")


def export_subdir(obj):
    """Folder path below the export folder: the import's subfolder ('' if none)."""
    return obj.get("unite_subdir", "")


def export_glb(context, obj, filepath, s):
    """Export one repaired object as binary glTF 2.0 (.glb).

    A temporary object that shares the mesh is exported at the original
    coordinates (the offset used to show the result beside the original is
    ignored), alone, without cameras/lights. The repaired object itself is not
    touched. Everything is looked up by name afterwards because the glTF
    exporter may use undo internally, which invalidates Python references.
    """
    view_layer = context.view_layer
    selected = [o.name for o in view_layer.objects if o.select_get()]
    active = view_layer.objects.active.name if view_layer.objects.active else None
    tmp = bpy.data.objects.new(os.path.splitext(os.path.basename(filepath))[0], obj.data)
    tmp.matrix_world = Matrix.Identity(4)
    context.scene.collection.objects.link(tmp)
    tmp_name = tmp.name
    try:
        for o in view_layer.objects:
            if o.select_get():
                o.select_set(False)
        tmp.select_set(True)
        view_layer.objects.active = tmp
        os.makedirs(os.path.dirname(filepath) or ".", exist_ok=True)
        result = bpy.ops.export_scene.gltf(
            filepath=filepath,
            export_format='GLB',
            use_selection=True,
            use_active_scene=True,  # selections in other scenes must not leak in
            export_apply=True,
            export_yup=s.export_yup,
            export_materials='EXPORT' if s.export_materials else 'NONE',
            export_normals=True,
            export_texcoords=True,
            export_cameras=False,
            export_lights=False,
            export_animations=False,
            export_extras=False,
        )
        if 'FINISHED' not in result:
            raise RuntimeError(f"glTF export failed: {filepath}")
    finally:
        tmp = bpy.data.objects.get(tmp_name)
        if tmp is not None:
            bpy.data.objects.remove(tmp, do_unlink=True)
        for name in selected:
            o = view_layer.objects.get(name)
            if o is not None:
                o.select_set(True)
        view_layer.objects.active = view_layer.objects.get(active) if active else None
    return filepath
