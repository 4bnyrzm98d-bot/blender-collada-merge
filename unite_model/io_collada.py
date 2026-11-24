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


"""COLLADA import through the Collada Support extension."""

import glob
import os

import bpy


def collada_importer_available():
    """True when an operator bpy.ops.import_scene.collada is registered.

    hasattr(bpy.ops.import_scene, "collada") is always True: bpy.ops returns
    a wrapper for any name, so it cannot detect a missing importer.
    """
    try:
        bpy.ops.import_scene.collada.get_rna_type()
        return True
    except Exception:
        return False


def import_dae(context, filepath):
    """Import one COLLADA file with the Collada Support importer.

    Returns the collection that holds the imported objects (the importer
    creates one per file; if it does not, the new objects are moved into a
    new collection named after the file).
    """
    if not collada_importer_available():
        raise RuntimeError("COLLADA importer not found: install the "
                           "'Collada Support' extension (Blender 5.x)")
    before_colls = set(bpy.data.collections)
    before_objs = set(bpy.data.objects)
    result = bpy.ops.import_scene.collada(filepath=filepath)
    if 'FINISHED' not in result:
        raise RuntimeError(f"import failed: {os.path.basename(filepath)}")
    new_colls = [c for c in bpy.data.collections if c not in before_colls]
    if new_colls:
        coll = max(new_colls, key=lambda c: len(c.all_objects))
    else:
        coll = bpy.data.collections.new(os.path.basename(filepath))
        context.scene.collection.children.link(coll)
        for o in bpy.data.objects:
            if o not in before_objs:
                for c in list(o.users_collection):
                    c.objects.unlink(o)
                coll.objects.link(o)
    coll["unite_source"] = filepath
    return coll


def list_dae_files(directory, recursive):
    pattern = os.path.join(directory, "**", "*") if recursive else os.path.join(directory, "*")
    return sorted(p for p in glob.glob(pattern, recursive=recursive)
                  if p.lower().endswith((".dae", ".zae")) and os.path.isfile(p))
