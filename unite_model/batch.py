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


"""Batch processing of a folder: import, repair, export, CSV report."""

import csv
import os
import time

import bpy

from .io_collada import import_dae
from .io_gltf import export_glb, verdict_folder
from .tags import VERDICT_KEY, VERDICT_REASON_KEY
from .workflow import repair_collection


def _snapshot_data():
    return (set(bpy.data.meshes), set(bpy.data.materials), set(bpy.data.images),
            set(bpy.data.cameras), set(bpy.data.lights))


def _remove_new_data(snapshot):
    """Free datablocks created since `snapshot` that nothing uses any more."""
    for coll, before in zip((bpy.data.meshes, bpy.data.materials, bpy.data.images,
                             bpy.data.cameras, bpy.data.lights), snapshot):
        for block in [b for b in coll if b not in before]:
            if block.users == 0:
                coll.remove(block)


def remove_collection(coll):
    for o in list(coll.all_objects):
        bpy.data.objects.remove(o, do_unlink=True)
    for child in list(coll.children_recursive):
        bpy.data.collections.remove(child)
    bpy.data.collections.remove(coll)


def process_dae_file(context, filepath, out_dir, rel_stem, s, keep_in_scene=False):
    """Batch step: import -> repair (-> test) -> export one file.

    The file goes to out_dir/rel_stem.glb, or below the subfolder of its test
    result (Correct / To_check) with "Correct / To_check Subfolders".
    Returns a report row.
    """
    row = {"file": filepath, "glb": "", "status": "ok", "error": ""}
    snapshot = _snapshot_data()
    coll = None
    t0 = time.time()
    try:
        coll = import_dae(context, filepath)
        obj, stats = repair_collection(context, coll, s)
        if obj is None:
            raise RuntimeError(stats.get("error", "nothing to repair"))
        row["test"] = obj.get(VERDICT_KEY, "")
        row["test_reason"] = obj.get(VERDICT_REASON_KEY, "")
        if s.export_verdict_folders:
            out_path = os.path.join(out_dir, verdict_folder(obj), rel_stem + ".glb")
        else:
            out_path = os.path.join(out_dir, rel_stem + ".glb")
        export_glb(context, obj, out_path, s)
        row["glb"] = out_path
        for key in ("test_back_hits", "test_hole_rays", "test_seconds","source_objects", "faces_in", "faces_out", "flipped",
                    "back_faces_removed", "enclosed_removed", "two_sided_faces",
                    "unrelated_parts", "unrelated_faces", "unrelated_area",
                    "buildings_found", "main_building_by", "unrelated_summary"):
            row[key] = stats.get(key, "")
    except Exception as exc:  # report and continue with the next file
        row["status"] = "error"
        row["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        if coll is not None and not keep_in_scene:
            remove_collection(coll)
            _remove_new_data(snapshot)
    row["seconds"] = round(time.time() - t0, 2)
    return row


REPORT_COLUMNS = ["file", "status", "test", "test_reason", "test_back_hits",
                  "test_hole_rays", "glb", "source_objects", "faces_in", "faces_out",
                  "flipped", "back_faces_removed", "enclosed_removed", "two_sided_faces",
                  "unrelated_parts", "unrelated_faces", "unrelated_area",
                  "buildings_found", "main_building_by", "unrelated_summary",
                  "test_seconds", "seconds", "error"]


def write_report(rows, out_dir):
    path = os.path.join(out_dir, "unite_report.csv")
    with open(path, "w", newline="", encoding="utf-8-sig") as fh:
        w = csv.DictWriter(fh, fieldnames=REPORT_COLUMNS, extrasaction="ignore", delimiter=";")
        w.writeheader()
        for r in rows:
            w.writerow(r)
    return path
