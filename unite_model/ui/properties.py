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


"""Settings shown in the panel (Scene.unite_model)."""

import math

import bpy
from bpy.props import BoolProperty, EnumProperty, FloatProperty, IntProperty, StringProperty


class UniteModelSettings(bpy.types.PropertyGroup):
    # import
    repair_after_import: BoolProperty(
        name="Repair After Import", default=True,
        description="Run the repair on every imported model right away",
    )
    import_subfolders: BoolProperty(
        name="Include Subfolders", default=False,
        description="When a folder is imported, also import .dae files in its subfolders",
    )
    arrange_imports: BoolProperty(
        name="Arrange Models", default=True,
        description="Place every imported model (and its result) in its own spot of a "
                    "grid instead of on top of the others: a folder import as one block per "
                    "folder, single imports one after another in rows. Only whole models "
                    "are moved; the export still uses the original coordinates",
    )
    group_by_folder: BoolProperty(
        name="Collections by Folder", default=True,
        description="When a folder is imported, put the models into collections named and "
                    "nested like the folders",
    )
    # repair
    merge_distance: FloatProperty(
        name="Merge Distance", default=0.0001, min=0.0, soft_max=0.01,
        precision=5, unit='LENGTH',
        description="Vertices closer than this are welded. Keep it well below the shortest edge",
    )
    remove_unrelated: BoolProperty(
        name="Remove Unrelated Geometry", default=True,
        description="Delete map underlays, photo backdrops, stray parts and other "
                    "geometry that does not belong to the house",
    )
    keep_buildings: EnumProperty(
        name="Buildings",
        items=[
            ('MAIN', "Main Only", "Keep only the house the file is about (chosen by the "
                                  "saved SketchUp camera, otherwise the largest)"),
            ('ALL', "All", "Keep every building found in the file"),
        ],
        default='MAIN',
    )
    keep_distance: FloatProperty(
        name="Keep Distance", default=1.0, min=0.0, soft_max=10.0, unit='LENGTH',
        description="Detached parts closer than this to the house are kept (doors, signs, "
                    "railings); farther ones are removed",
    )
    remove_ground: BoolProperty(
        name="Remove Ground Strips", default=True,
        description="Delete flat faces at ground level that are attached to the house "
                    "but not covered by it (pavement, driveway, yard)",
    )
    keep_removed: BoolProperty(
        name="Keep Removed Parts", default=True,
        description="Store removed geometry in a hidden '<name>_removed' object for review",
    )
    close_gaps: BoolProperty(
        name="Close Slits", default=True,
        description="Fill narrow slits the model leaves open (e.g. between a facade and "
                    "the cornice). SketchUp hides them by drawing both sides of faces",
    )
    gap_width: FloatProperty(
        name="Max Slit Width", default=0.4, min=0.01, soft_max=1.0, unit='LENGTH',
        description="Wider openings (windows, doors) stay open",
    )
    fill_holes: BoolProperty(
        name="Fill Missing Faces", default=True,
        description="Close holes where a face of a closed shape is missing (a gable, "
                    "the floor, a chimney top). Windows cut into walls stay open",
    )
    fix_normals: BoolProperty(
        name="Normals Outward", default=True,
        description="Orient every face so it points to the outside of the model",
    )
    remove_enclosed: BoolProperty(
        name="Remove Hidden Inner Faces", default=True,
        description="Delete faces that are completely hidden inside the model "
                    "(never deleted when visible from outside)",
    )
    two_sided: BoolProperty(
        name="Free-Standing Faces Two-Sided", default=True,
        description="Faces open to the outside on both sides (free-standing walls, "
                    "canopies, fences) get a reversed copy so they look right from both sides",
    )
    on_ground: BoolProperty(
        name="Stands on Ground", default=True,
        description="The model stands on terrain at its lowest point: it is never "
                    "seen from below and its base faces point down",
    )
    smooth_angle: FloatProperty(
        name="Auto Smooth Angle", default=math.radians(30.0),
        min=0.0, max=math.pi, subtype='ANGLE',
        description="Edges sharper than this stay sharp",
    )
    source_action: EnumProperty(
        name="Source Objects",
        items=[
            ('KEEP', "Keep", "Leave the original objects visible"),
            ('HIDE', "Hide", "Keep the original objects but hide them"),
            ('DELETE', "Delete", "Delete the original objects and their empties"),
        ],
        default='KEEP',
    )
    place_beside: BoolProperty(
        name="Place Beside Original", default=True,
        description="Move the result next to the original model so both can be compared",
    )
    gap: FloatProperty(
        name="Gap", default=2.0, min=0.0, unit='LENGTH',
        description="Free space between the original and the result",
    )
    show_repair_options: BoolProperty(name="Options", default=False)
    # test
    test_after_unite: BoolProperty(
        name="Test After Unite", default=True,
        description="Check every repaired model with rays from outside (faces seen from "
                    "the back, visible faces that were deleted, duplicate faces) and mark "
                    "it Correct or To check",
    )
    sort_after_test: BoolProperty(
        name="Sort into Correct / To check", default=True,
        description="After the test, move the models into the collections 'Correct' and "
                    "'To check' and lay them out as two blocks",
    )
    test_grid: IntProperty(
        name="Test Resolution", default=120, min=20, soft_max=200,
        description="Rays per side of each of the 97 test views (120 x 120 like the batch "
                    "tests). Lower is faster but can miss small faces",
    )
    test_max_back_hits: IntProperty(
        name="Max Back Hits", default=0, min=0,
        description="Allowed rays from outside that land on the back of a face (red in Face "
                    "Orientation). 0 = strict; at the default resolution a model has about "
                    "200 000 - 500 000 hits",
    )
    test_max_holes: IntProperty(
        name="Max Hole Rays", default=0, min=0,
        description="Allowed rays that pass where the original model has a visible face",
    )
    show_test_options: BoolProperty(name="Test Options", default=False)
    # export
    export_dir: StringProperty(
        name="Folder", subtype='DIR_PATH', default="",
        description="Folder for the exported .glb files",
    )
    export_verdict_folders: BoolProperty(
        name="Correct / To_check Subfolders", default=False,
        description="Put the files into the subfolders 'Correct' and 'To_check' of the "
                    "export folder (by the test result); the import subfolders are kept "
                    "below them",
    )
    export_materials: BoolProperty(
        name="Materials", default=True,
        description="Export materials and textures",
    )
    export_yup: BoolProperty(
        name="+Y Up", default=True,
        description="glTF convention: +Y is up (Blender's +Z is converted)",
    )
    # batch
    batch_input_dir: StringProperty(
        name="Input", subtype='DIR_PATH', default="",
        description="Folder with .dae files to import, repair and export",
    )
    batch_subfolders: BoolProperty(
        name="Include Subfolders", default=True,
        description="Process .dae files in subfolders too; the folder structure is "
                    "kept in the output folder",
    )
    batch_keep_in_scene: BoolProperty(
        name="Keep in Scene", default=False,
        description="Leave imported and repaired models in the scene (uses a lot of "
                    "memory for many files)",
    )


def get_settings(context):
    return context.scene.unite_model
