# Progress and summary of a batch test started with launch_batch.py.
# Run through the Blender MCP tool (execute_blender_code); set PIDS to the
# list launch_batch.py returned (or leave empty to skip the "still running"
# check). Reads only the JSONL result files.
#
# A model is flagged when any of these hold:
#   back hits from views above the horizon > 0.001 % of rays
#   hole rays from views above the horizon > 2
#   inconsistent_edges > 0, duplicate_faces > 0, zero_area > 0
# A few single back hits on big faces are grazing-angle noise and stay below
# the threshold. Views from below are listed as "below_hints" only.

import os, json, glob, tempfile

OUT_DIR = os.path.join(tempfile.gettempdir(), "unite_test")
PIDS = []

records = []
for path in sorted(glob.glob(os.path.join(OUT_DIR, "out_*.jsonl"))):
    with open(path, encoding="utf-8") as fh:
        records += [json.loads(l) for l in fh if l.strip()]

expected = 0
for path in glob.glob(os.path.join(OUT_DIR, "list_*.txt")):
    with open(path, encoding="utf-8") as fh:
        expected += sum(1 for l in fh if l.strip())

alive = []
for pid in PIDS:
    try:
        os.kill(pid, 0)
        alive.append(pid)
    except OSError:
        pass

flagged, errors, below_hints, ok = [], [], [], 0
for r in records:
    name = os.path.relpath(r["file"], os.path.commonpath([x["file"] for x in records])) if len(records) > 1 else r["file"]
    if r["status"] == "error":
        errors.append({"file": name, "error": r["error"].strip().splitlines()[-1][:200]})
        continue
    if r["status"] != "ok":
        errors.append({"file": name, "error": r["status"]})
        continue
    v = r["verify"]
    reasons = []
    # Pass/fail uses views from above the horizon (unambiguous); below-horizon
    # numbers are reported as hints only (views from below can enter open model bottoms).
    back_above = v.get("back_hits_from_above", v["back_hits"])
    holes_above = v.get("hole_rays_from_above", v["hole_rays"])
    if back_above > 1e-5 * max(1, v["rays_hit"]):
        reasons.append(f"back from above {back_above}/{v['rays_hit']}")
    if holes_above > 2:
        reasons.append(f"holes from above {holes_above}")
    for key in ("inconsistent_edges", "duplicate_faces", "zero_area"):
        if v[key]:
            reasons.append(f"{key} {v[key]}")
    if reasons:
        flagged.append({"file": name, "reasons": reasons, "worst_back": v["worst_back"][:3],
                        "objects": r["mesh_objects"], "faces_out": r["stats"]["faces_out"]})
    else:
        ok += 1
    below_back = v["back_hits"] - back_above
    below_holes = v["hole_rays"] - holes_above
    if below_back > 100 or below_holes > 20:
        below_hints.append({"file": name, "back_from_below": below_back, "holes_from_below": below_holes})

times = [r.get("unite_s", 0) for r in records if r["status"] == "ok"]
result = {
    "done": len(records), "expected": expected, "still_running": alive,
    "clean": ok, "flagged": len(flagged), "errors": len(errors),
    "unite_seconds_avg": round(sum(times) / len(times), 2) if times else None,
    "unite_seconds_max": max(times) if times else None,
    "flagged_models": flagged, "error_models": errors,
    "below_hints": below_hints,
}
