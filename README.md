# blender-collada-merge
Blender 5.x add-on: import COLLADA (.dae) house models, merge the split pieces into one object, fix normals, test the result and export glTF 2.0 (.glb).

## Unite Model add-on (`unite_model/`)

Install the `unite_model` folder as a Blender extension (Blender 4.2+, tested on 5.2). The folder
with `blender_manifest.toml` is the extension; the repository root is not. The panel is in
**3D View → Sidebar → Unite**.

Requires a COLLADA importer that registers `bpy.ops.import_scene.collada`
(on Blender 5.x the *Collada Support* extension from extensions.blender.org). If it is missing,
the Import section shows a red warning.

### Workflow (panel sections)

1. **Import COLLADA** – single `.dae` files or a whole folder with subfolders. Each folder gets
   its own collection and models are laid out in a grid, so imports never land on top of each
   other. *Arrange Scene* re-arranges everything already in the scene.
2. **Repair** – for every model collection:
   - all pieces are copied into one world-space mesh; the back face of every SketchUp
     double-sided pair is removed *before* welding (0.1 mm); degenerate and zero-area faces go;
   - unrelated geometry (loose sheets, far parts, neighbouring buildings, ground strips) is
     moved to a hidden `<name>_removed` object;
   - narrow slits (up to 0.4 m) are bridged, missing planar faces (e.g. floors) are filled;
   - faces are oriented outward (ray test with a crack-leak probe); hidden inner faces are removed;
   - coplanar faces lying on top of each other (the same surface twice with another
     triangulation, a panel drawn onto a wall) are cut so only one layer remains – no z-fighting;
     a small face on a bigger one stays, the bigger one gets a hole under it;
   - free-standing faces get a reversed twin, faces in niches / under eaves a 1 cm offset twin
     with rims; faces that point up with nothing below them (passage floors, steps, cornice
     tops, eaves without soffit) get a 1 cm offset back face closed by a rim, so the bottom of
     the model is never red; shading uses area-weighted custom normals.
3. **Test** – right after the repair (or with *Test All Models* later) the result is
   "photographed" with rays from 97 directions above the horizon and compared with a reference
   (the same model without orientation and without removed faces). A model is **Correct** when
   - no ray from outside lands on the back of a face (red in Face Orientation) – oblique views
     from above and the sides, plus vertical rays from below for the bottom of the model,
   - no ray passes where the original has a visible face (a deleted face),
   - there are no duplicate and no zero-area faces and no faces lying on another face pointing the
     same way (z-fighting, over 10 cm²).

   Rays through hairline cracks, the underside of the base and two-sided faces do not count.

   Otherwise it is **To check**, with the reason shown in the panel; *Select Problem Faces*
   opens the model in Edit Mode with the faces seen from the back selected. The models are then
   moved into the collections **Correct** (green) and **To check** (red), inside them into a
   collection per import folder (`1.NAM`, `1.NAM (to check)`, …), and laid out as two sections
   side by side, one block per folder. Limits and ray resolution are under *Test Options*.
4. **Export glTF 2.0 (.glb)** – *Export Correct*, *Export To check*, *Export All* or
   *Export Selected* into the chosen folder; import subfolders are mirrored. With
   *Correct / To_check Subfolders* the files go to `Correct/…` and `To_check/…`.

**Batch: Folder to GLB** does import → repair → test → export for a whole folder and writes
`unite_report.csv` (with the test result of every file).

*Check Normals* toggles Face Orientation and Backface Culling for a quick visual check.
Original objects are never modified.

### Code layout

```
unite_model/
├── __init__.py          register/unregister, public API for scripts and tools
├── blender_manifest.toml
├── tags.py              names of custom properties and bmesh layers
├── geometry/            pure bmesh steps, no operators, no scene
│   ├── build.py         meshes → one bmesh, back faces, welding, slivers
│   ├── unrelated.py     parts that do not belong to the house
│   ├── fill.py          narrow slits, missing planar faces
│   ├── overlap.py       coplanar faces on top of each other (z-fighting)
│   ├── orient.py        outward normals, hidden faces, two-sided faces
│   └── rays.py          ray casting helpers
├── pipeline.py          collection → united bmesh / object (unite_bmesh, unite_collection)
├── verify.py            automatic test of a result → Correct / To check
├── workflow.py          repair + test of one model with the panel settings
├── scene.py             model collections, placement and grid layout
├── sorting.py           Correct / To check collections, export selection
├── io_collada.py        COLLADA import (Collada Support)
├── io_gltf.py           glTF 2.0 (.glb) export
├── batch.py             folder → GLB batch with CSV report
└── ui/
    ├── properties.py    settings (Scene.unite_model)
    ├── operators.py     buttons
    └── panels.py        sidebar panels
```

Dependencies go one way: `geometry` ← `pipeline` ← `verify`/`workflow` ← `sorting`/`batch` ← `ui`.
Only `ui` defines Blender classes.

Scripting example:

```python
import bpy, unite_model  # or bl_ext.<repo>.unite_model when installed as extension
obj, stats = unite_model.unite_collection(bpy.data.collections["42.dae"])
```

## Test tools (`tools/`)

Scripts for background Blender processes (never run inside an interactive session). They take
the path of the `unite_model` folder (or its `__init__.py`).

| Script | Purpose |
|---|---|
| `analyze_scene.py` | read-only analysis of an imported scene (double-sided pairs, weld effects) |
| `verify_result.py` | ray-test a united object against its source |
| `batch_runner.py` | batch: import, unite and verify a list of `.dae` files |
| `batch_verify.py` | batch with GLB export/re-import check, crack-filtered ("solid") metrics and per-face diagnostics |
| `launch_batch.py` | start several background workers in parallel |
| `summarize_batch.py` | summarize the JSONL results |

Test models are not part of this repository.

## License

GPL-3.0-or-later.
