"""Prepare inspectable Panda MBM inputs; does not run a planner or validate paths."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import random
import re
import tarfile
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
UPSTREAM = ROOT / "upstream" / "vamp-vcc"
COMMIT = "96a0d556e9677bb91c755aed7d268fa2772035b9"
JOINTS = [f"panda_joint{i}" for i in range(1, 8)]


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def pose_matrix(pose):
    x, y, z, w = map(float, pose.get("orientation", [0, 0, 0, 1]))
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm == 0:
        raise ValueError("Zero quaternion")
    x, y, z, w = [v / norm for v in (x, y, z, w)]
    px, py, pz = map(float, pose.get("position", [0, 0, 0]))
    return [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w), px],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w), py],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y), pz],
            [0., 0., 0., 1.]]


def multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def normalized_problem(scene, request, scenario, source_index):
    start_state = request["start_state"]["joint_state"]
    start_map = dict(zip(start_state["name"], start_state["position"], strict=True))
    goals = []
    for constraint in request["goal_constraints"]:
        goal_map = {item["joint_name"]: item["position"] for item in constraint["joint_constraints"]}
        goals.append([float(goal_map[j]) for j in JOINTS])
    primitives = []
    for obj in scene["world"]["collision_objects"]:
        parent_pose = pose_matrix(obj.get("pose", {}))
        for i, (shape, pose) in enumerate(zip(obj["primitives"], obj["primitive_poses"], strict=True)):
            kind = shape["type"]
            dimensions = list(map(float, shape["dimensions"]))
            expected = {"box": 3, "cylinder": 2, "sphere": 1}
            if kind not in expected or len(dimensions) != expected[kind] or min(dimensions) <= 0:
                raise ValueError(f"Unsupported primitive: {shape}")
            primitives.append({"object_id": obj["id"], "primitive_index": i,
                               "type": kind, "dimensions": dimensions,
                               "world_from_local": multiply(parent_pose, pose_matrix(pose))})
    return {"schema": "soc-panda-input-v1", "robot": "panda", "scenario": scenario,
            "source_index": source_index, "units": {"length": "m", "joint_angle": "rad"},
            "joint_names": JOINTS, "start": [float(start_map[j]) for j in JOINTS],
            "goals": goals, "primitives": primitives,
            "validation_status": "not_run_no_start_goal_collision_check",
            "coordinate_note": "MBM scene world coordinates; fixed_frame_transforms retained in source_scene.json. No camera simulation.",
            "dimension_order": {"box": ["full_x", "full_y", "full_z"],
                                "cylinder": ["height", "radius"], "sphere": ["radius"]}}


def weighted_index(rng, weights):
    target = rng.random() * sum(weights)
    cumulative = 0.
    for i, weight in enumerate(weights):
        cumulative += weight
        if target < cumulative:
            return i
    return len(weights) - 1


def sample_local(rng, primitive):
    dims = primitive["dimensions"]
    kind = primitive["type"]
    if kind == "box":
        dx, dy, dz = dims
        side = weighted_index(rng, [dy*dz, dy*dz, dx*dz, dx*dz, dx*dy, dx*dy])
        p = [rng.uniform(-d/2, d/2) for d in dims]
        axis = side // 2
        p[axis] = dims[axis] / 2 * (1 if side % 2 == 0 else -1)
        return p
    if kind == "cylinder":
        height, radius = dims
        side = weighted_index(rng, [math.pi*radius**2, 2*math.pi*radius*height, math.pi*radius**2])
        angle = rng.uniform(-math.pi, math.pi)
        rho = radius if side == 1 else radius * math.sqrt(rng.random())
        z = rng.uniform(-height/2, height/2) if side == 1 else (-height/2 if side == 0 else height/2)
        return [rho*math.cos(angle), rho*math.sin(angle), z]
    z = rng.uniform(-1, 1)
    angle = rng.uniform(-math.pi, math.pi)
    rho = math.sqrt(max(0., 1-z*z))
    return [dims[0]*rho*math.cos(angle), dims[0]*rho*math.sin(angle), dims[0]*z]


def surface_residual(p, primitive):
    t = primitive["world_from_local"]
    d = [p[i]-t[i][3] for i in range(3)]
    local = [sum(t[i][j]*d[i] for i in range(3)) for j in range(3)]
    dims = primitive["dimensions"]
    if primitive["type"] == "box":
        half = [v/2 for v in dims]
        outside = max(0., *(abs(v)-h for v, h in zip(local, half)))
        return max(outside, min(abs(abs(v)-h) for v, h in zip(local, half)))
    if primitive["type"] == "cylinder":
        height, radius = dims
        rho = math.hypot(local[0], local[1])
        outside = max(0., rho-radius, abs(local[2])-height/2)
        return max(outside, min(abs(rho-radius), abs(abs(local[2])-height/2)))
    return abs(math.sqrt(sum(v*v for v in local))-dims[0])


def export_cloud(problem, out_dir, samples_per_primitive, seed):
    rng = random.Random(seed)
    out_dir.mkdir(parents=True, exist_ok=True)
    write_json(out_dir / "problem.json", problem)
    count = samples_per_primitive * len(problem["primitives"])
    bounds_min = [math.inf]*3
    bounds_max = [-math.inf]*3
    max_residual = 0.
    with (out_dir / "pointcloud.xyz").open("w", encoding="ascii", newline="\n") as xyz, \
         (out_dir / "pointcloud.ply").open("w", encoding="ascii", newline="\n") as ply, \
         (out_dir / "point_sources.csv").open("w", encoding="utf-8", newline="") as source_file:
        ply.write(f"ply\nformat ascii 1.0\ncomment synthetic MBM surface samples; units m\nelement vertex {count}\nproperty double x\nproperty double y\nproperty double z\nend_header\n")
        writer = csv.writer(source_file)
        writer.writerow(["point_index", "primitive_index", "object_id"])
        point_index = 0
        for primitive_index, primitive in enumerate(problem["primitives"]):
            t = primitive["world_from_local"]
            for _ in range(samples_per_primitive):
                local = sample_local(rng, primitive)
                p = [sum(t[i][j]*local[j] for j in range(3))+t[i][3] for i in range(3)]
                if not all(math.isfinite(v) for v in p):
                    raise ValueError("Non-finite point")
                for i in range(3):
                    bounds_min[i] = min(bounds_min[i], p[i])
                    bounds_max[i] = max(bounds_max[i], p[i])
                max_residual = max(max_residual, surface_residual(p, primitive))
                line = " ".join(format(v, ".17g") for v in p) + "\n"
                xyz.write(line)
                ply.write(line)
                writer.writerow([point_index, primitive_index, primitive["object_id"]])
                point_index += 1
    if max_residual > 1e-10:
        raise ValueError(f"Surface sampling residual: {max_residual}")
    metadata = {"point_count": count, "samples_per_primitive": samples_per_primitive,
                "seed": seed, "aabb_min_m": bounds_min, "aabb_max_m": bounds_max,
                "max_surface_residual_m": max_residual,
                "sampling": "uniform surface-area sampling; Python random.Random; custom exporter, not byte-identical to upstream NumPy sampler",
                "occlusion": False, "sensor_noise": False, "filtered": False,
                "planner_or_soc_executed": False}
    write_json(out_dir / "metadata.json", metadata)
    return metadata


def export_model_summary():
    model = ET.parse(UPSTREAM / "resources/panda/panda_spherized.urdf").getroot()
    joints = []
    for joint in model.findall("joint"):
        limit = joint.find("limit")
        joints.append({"name": joint.attrib["name"], "type": joint.attrib["type"],
                       "parent": joint.find("parent").attrib["link"],
                       "child": joint.find("child").attrib["link"],
                       "origin": joint.find("origin").attrib if joint.find("origin") is not None else {},
                       "axis": joint.find("axis").attrib if joint.find("axis") is not None else {},
                       "limit": limit.attrib if limit is not None else {}})
    spheres = []
    for link in model.findall("link"):
        for collision in link.findall("collision"):
            sphere = collision.find("geometry/sphere")
            if sphere is not None:
                origin = collision.find("origin")
                spheres.append({"link": link.attrib["name"], "radius_m": float(sphere.attrib["radius"]),
                                "origin_in_link": origin.attrib if origin is not None else {}})
    write_json(ROOT / "data/panda_model_summary.json",
               {"source": "upstream/vamp-vcc/resources/panda/panda_spherized.urdf",
                "planning_joint_names": JOINTS, "joints": joints, "collision_spheres": spheres,
                "note": "URDF summary only; fixed-joint/finger settings and self-collision rules must follow upstream robot code and SRDF. Not an FK implementation."})
    return len(spheres)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples-per-primitive", type=int, default=10000)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    if args.samples_per_primitive <= 0:
        parser.error("samples-per-primitive must be positive")
    archive = UPSTREAM / "resources/panda/problems.tar.bz2"
    pairs = {}
    with tarfile.open(archive, "r:bz2") as tar:
        for member in tar.getmembers():
            if not member.isfile():
                continue
            match = re.fullmatch(r"problems/([a-z_]+)_panda/(scene|request)(\d+)\.yaml", member.name)
            if not match:
                raise ValueError(f"Unexpected archive entry: {member.name}")
            scenario, kind, source_index = match[1], match[2], int(match[3])
            payload = tar.extractfile(member).read()
            # Only fixed, validated path components are written; tar.extractall is not used.
            raw = ROOT / "data/panda_mbm/raw_yaml" / scenario / Path(member.name).name
            raw.parent.mkdir(parents=True, exist_ok=True)
            raw.write_bytes(payload)
            bucket = pairs.setdefault((scenario, source_index), {})
            if kind in bucket:
                raise ValueError(f"Duplicate pair member: {member.name}")
            bucket[kind] = yaml.safe_load(payload)
    counts, index, examples = {}, [], []
    for (scenario, source_index), pair in sorted(pairs.items()):
        if set(pair) != {"scene", "request"}:
            raise ValueError(f"Unpaired scene/request: {scenario} {source_index}")
        problem = normalized_problem(pair["scene"], pair["request"], scenario, source_index)
        out = ROOT / "data/panda_mbm/problems" / scenario / f"{source_index:04d}"
        write_json(out / "problem.json", problem)
        write_json(out / "source_scene.json", pair["scene"])
        write_json(out / "source_request.json", pair["request"])
        counts[scenario] = counts.get(scenario, 0) + 1
        index.append({"scenario": scenario, "source_index": source_index,
                      "problem_path": (out / "problem.json").relative_to(ROOT).as_posix(),
                      "primitive_count": len(problem["primitives"]),
                      "start_goal_collision_validated": False})
        if scenario in {"box", "bookshelf_thin", "cage"} and counts[scenario] == 1:
            name = f"panda_{scenario}_{source_index:04d}"
            stats = export_cloud(problem, ROOT / "examples" / name, args.samples_per_primitive, args.seed)
            examples.append({"name": name, **stats})
    if len(pairs) != 700 or len(counts) != 7 or any(n != 100 for n in counts.values()):
        raise ValueError(f"Unexpected dataset count: {counts}")
    write_json(ROOT / "data/panda_mbm/index.json", index)
    spheres = export_model_summary()
    report = {"source_commit": COMMIT, "source_archive_sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
              "problem_count": len(pairs), "raw_yaml_count": 2*len(pairs), "scenario_counts": counts,
              "urdf_collision_sphere_count": spheres, "examples": examples,
              "checks_performed": ["700 complete scene/request pairs", "7 arm joints present in every start and goal",
                                   "positive supported primitive dimensions", "finite sample coordinates",
                                   "sample points lie on their source primitive surfaces within 1e-10 m"],
              "not_performed": ["VAMP installation or execution", "start/goal collision validity",
                                "SoC functional verification", "path planning or path validation", "timing or power measurement"]}
    write_json(ROOT / "reports/data_preparation_report.json", report)
    print(json.dumps({"problems": len(pairs), "scenarios": counts,
                      "examples": [{"name": e["name"], "points": e["point_count"]} for e in examples]}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
