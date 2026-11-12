"""Check prepared input assets; this is not a planner/SoC correctness test."""
import argparse
import hashlib
import json
import math
import os
from datetime import datetime
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
UPSTREAM = ROOT / "upstream/vamp-vcc"
COMMIT = "96a0d556e9677bb91c755aed7d268fa2772035b9"
EXCLUDED_FILES = {"manifest.json", "reports/package_verification_report.json"}


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024*1024), b""):
            h.update(block)
    return h.hexdigest()


def inventory():
    result = []
    for directory, dirs, files in os.walk(ROOT):
        dirs[:] = [d for d in dirs if d not in {".git", ".venv", "__pycache__"}
                   and not (Path(directory) == ROOT / "build" and d in {"zig_global_cache", "zig_local_cache", "tmp"})]
        for name in files:
            path = Path(directory) / name
            rel = path.relative_to(ROOT).as_posix()
            if rel not in EXCLUDED_FILES:
                result.append({"path": rel, "bytes": path.stat().st_size, "sha256": digest(path)})
    return sorted(result, key=lambda row: row["path"])


def require(condition, message):
    if not condition:
        raise ValueError(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write-manifest", action="store_true", help="Record a new intentional asset version, then verify it")
    args = parser.parse_args()
    entries = inventory()
    manifest_path = ROOT / "manifest.json"
    if args.write_manifest:
        manifest_path.write_text(json.dumps({"prepared_date": datetime.now().date().isoformat(), "timezone": "Asia/Taipei",
            "source_repository": "https://github.com/nycu-caslab/vamp-vcc",
            "source_commit": COMMIT, "excluded": [".git", ".venv", "__pycache__", "build/zig_global_cache", "build/zig_local_cache", "build/tmp", *sorted(EXCLUDED_FILES)],
            "files": entries}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    manifest = read_json(manifest_path)
    require(manifest["files"] == entries, "Manifest mismatch: files were added, removed, or changed")
    git_commit = subprocess.check_output(["git", "-C", str(UPSTREAM), "rev-parse", "HEAD"], text=True).strip()
    require(git_commit == COMMIT, "Unexpected upstream commit")
    dirty = subprocess.check_output(["git", "-C", str(UPSTREAM), "status", "--porcelain"], text=True)
    require(not dirty.strip(), "Upstream tracked/untracked files changed")

    index = read_json(ROOT / "data/panda_mbm/index.json")
    require(len(index) == 700, "Expected 700 Panda inputs")
    require(len({(p['scenario'], p['source_index']) for p in index}) == 700, "Duplicate input key")
    counts = {}
    for entry in index:
        path = ROOT / entry["problem_path"]
        p = read_json(path)
        request = read_json(path.parent / "source_request.json")
        scene = read_json(path.parent / "source_scene.json")
        names = p["joint_names"]
        start = dict(zip(request["start_state"]["joint_state"]["name"], request["start_state"]["joint_state"]["position"], strict=True))
        require(p["start"] == [start[j] for j in names], f"Start mismatch: {path}")
        require(len(p["goals"]) == len(request["goal_constraints"]), f"Goal count mismatch: {path}")
        for goal, constraint in zip(p["goals"], request["goal_constraints"], strict=True):
            original = {x["joint_name"]: x["position"] for x in constraint["joint_constraints"]}
            require(goal == [original[j] for j in names], f"Goal mismatch: {path}")
        require(all(len(q) == 7 and all(math.isfinite(x) for x in q) for q in [p["start"], *p["goals"]]), "Invalid joint vector")
        require(sum(len(o["primitives"]) for o in scene["world"]["collision_objects"]) == len(p["primitives"]), "Dropped primitive")
        require(p["validation_status"] == "not_run_no_start_goal_collision_check", "Invalid validation claim")
        counts[p["scenario"]] = counts.get(p["scenario"], 0) + 1
    require(len(counts) == 7 and set(counts.values()) == {100}, "Scenario count mismatch")
    raw_count = len(list((ROOT / "data/panda_mbm/raw_yaml").rglob("*.yaml")))
    require(raw_count == 1400, "Expected 1400 original YAML files")

    mesh_refs = 0
    model_dir = UPSTREAM / "resources/panda"
    for urdf in model_dir.glob("*.urdf"):
        model = ET.parse(urdf).getroot()
        for mesh in model.findall(".//mesh"):
            name = mesh.attrib["filename"]
            if name.startswith("package://"):
                name = name[len("package://"):]
            target = model_dir / name
            require(target.is_file(), f"Missing mesh: {target}")
            mesh_refs += 1
    ET.parse(model_dir / "panda.srdf")
    examples = []
    for case in sorted((ROOT / "examples").iterdir()):
        metadata = read_json(case / "metadata.json")
        count = 0
        with (case / "pointcloud.xyz").open(encoding="ascii") as stream:
            for line in stream:
                p = list(map(float, line.split()))
                require(len(p) == 3 and all(math.isfinite(v) for v in p), f"Invalid XYZ: {case}")
                count += 1
        with (case / "pointcloud.ply").open(encoding="ascii") as stream:
            header = []
            for line in stream:
                header.append(line.strip())
                if line.strip() == "end_header":
                    break
            require(f"element vertex {count}" in header, "PLY header count mismatch")
            with (case / "pointcloud.xyz").open(encoding="ascii") as xyz:
                for xyz_line, ply_line in zip(xyz, stream, strict=True):
                    require(xyz_line == ply_line, "PLY and XYZ differ")
        with (case / "point_sources.csv").open(encoding="utf-8") as stream:
            source_count = sum(1 for _ in stream)-1
        require(count == source_count == metadata["point_count"], "Example count mismatch")
        examples.append({"name": case.name, "point_count": count})
    require(len(examples) == 3, "Expected three examples")
    report = {"status": "PASS_DATA_INTEGRITY_ONLY", "upstream_commit": git_commit,
              "manifest_file_count": len(entries), "manifest_total_bytes": sum(x["bytes"] for x in entries),
              "panda_problem_count": len(index), "raw_yaml_count": raw_count,
              "scenario_counts": counts, "resolved_panda_mesh_references": mesh_refs,
              "examples": examples, "soc_or_path_validation": "NOT_CHECKED_BY_THIS_TOOL; see runs/user_restored_v1/summary.json and runs/user_restored_regression_v1/summary.json"}
    path = ROOT / "reports/package_verification_report.json"
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
