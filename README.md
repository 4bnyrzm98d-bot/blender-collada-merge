# blender-collada-merge
Blender 5.x add-on: import COLLADA (.dae) house models, merge the split pieces into one object and fix normals.

## Unite Model add-on (`unite_model/`)

Install the folder as a Blender extension (Blender 4.2+, tested on 5.x). The panel is in
**3D View → Sidebar → Unite Model**.

Requires a COLLADA importer that registers `bpy.ops.import_scene.collada`
(on Blender 5.x the *Collada Support* extension).

What it does for every imported collection:

1. **Import** – single `.dae` files or a whole folder with subfolders. Each folder gets its own
   collection and models are laid out in a grid (rows of 8), so imports never land on top of
   each other. *Arrange Scene* re-arranges everything already in the scene.
2. **Unite** – all pieces are copied into one world-space mesh; the back face of every
   SketchUp double-sided pair is removed *before* welding (0.1 mm), degenerate and zero-area
   faces are removed.
3. **Remove unrelated geometry** – loose sheets, far parts, neighbouring buildings and stray
   ground strips that are not part of the house.
4. **Close gaps / fill missing faces** – narrow slits (up to 0.4 m) are bridged, missing planar
   faces (e.g. floors) are filled; filled floors point down.
5. **Orient outward** – ray-based inside/outside test with a crack-leak probe (views through
   hairline cracks are ignored); hidden enclosed faces are removed (tiny ones are kept).
6. **Two-sided faces** – free-standing faces get a coincident reversed twin; faces in niches and
   under eaves get a 1 cm offset twin with rims (like Solidify) to avoid z-fighting.
7. **Normals** – smooth by angle with area-weighted custom normals.
8. **Export** – GLB (glTF 2.0) into a chosen folder, mirroring the source subfolders.

*Check Normals* toggles Face Orientation and Backface Culling for a quick visual check.
Original objects are never modified.

## Test tools (`tools/`)

Scripts for background Blender processes (never run inside an interactive session):

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
