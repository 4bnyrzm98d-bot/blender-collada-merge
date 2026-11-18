# Start the batch test in background Blender processes.
# Run through the Blender MCP tool (execute_blender_code) after setting the
# four constants. The user's open Blender session is not touched: each worker
# is a separate `blender --background` process that imports and processes the
# models one by one. Models are only read from disk; results (numbers only)
# go to OUT_DIR as out_<k>.jsonl. Poll progress with summarize_batch.py.

import bpy, glob, os, subprocess, tempfile

MODELS_DIR = None        # folder searched recursively for *.dae
PLUGIN_PATH = None       # the plugin's __init__.py
RUNNER_PATH = None       # tools/batch_runner.py from this repository
WORKERS = 4
OUT_DIR = os.path.join(tempfile.gettempdir(), "unite_test")

os.makedirs(OUT_DIR, exist_ok=True)
files = set(glob.glob(os.path.join(MODELS_DIR, "**", "*.dae"), recursive=True))
files |= set(glob.glob(os.path.join(MODELS_DIR, "**", "*.DAE"), recursive=True))
# biggest first so the slow ones do not end up alone at the end
files = sorted(files, key=lambda p: -os.path.getsize(p))

pids = []
for k in range(WORKERS):
    lst = os.path.join(OUT_DIR, f"list_{k}.txt")
    out = os.path.join(OUT_DIR, f"out_{k}.jsonl")
    with open(lst, "w", encoding="utf-8") as fh:
        fh.write("\n".join(files[k::WORKERS]) + "\n")
    if os.path.exists(out):
        os.remove(out)
    log = open(os.path.join(OUT_DIR, f"log_{k}.txt"), "w")
    p = subprocess.Popen([bpy.app.binary_path, "--background", "--python", RUNNER_PATH,
                          "--", PLUGIN_PATH, lst, out],
                         stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    pids.append(p.pid)

result = {"files": len(files), "pids": pids, "out_dir": OUT_DIR}
