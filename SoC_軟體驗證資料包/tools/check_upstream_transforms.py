"""Optional independent transform comparison; requires NumPy, not the VAMP extension."""
import importlib.util
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
module_path = ROOT / "upstream/vamp-vcc/src/vamp/transformations.py"
spec = importlib.util.spec_from_file_location("upstream_transform_reference", module_path)
reference = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reference)


def pose(p):
    return reference.concatenate_matrices(reference.translation_matrix(p.get("position", [0, 0, 0])),
                                          reference.quaternion_matrix(p.get("orientation", [0, 0, 0, 1])))


def main():
    index = json.loads((ROOT / "data/panda_mbm/index.json").read_text(encoding="utf-8"))
    maximum = 0.
    comparisons = 0
    for entry in index:
        path = ROOT / entry["problem_path"]
        normalized = json.loads(path.read_text(encoding="utf-8"))
        scene = json.loads((path.parent / "source_scene.json").read_text(encoding="utf-8"))
        flattened = []
        for obj in scene["world"]["collision_objects"]:
            flattened.extend(pose(obj.get("pose", {})) @ pose(p) for p in obj["primitive_poses"])
        for expected, primitive in zip(flattened, normalized["primitives"], strict=True):
            error = float(np.max(np.abs(expected - np.asarray(primitive["world_from_local"]))))
            maximum = max(maximum, error)
            comparisons += 1
    if maximum > 1e-12:
        raise ValueError(f"Coordinate transform discrepancy: {maximum}")
    result = {"status": "PASS_TRANSFORM_CONVERSION_ONLY", "primitive_transforms_compared": comparisons,
              "maximum_matrix_element_difference": maximum,
              "reference": "upstream src/vamp/transformations.py float64",
              "numpy_version": np.__version__, "fk_or_soc_validation": "NOT_RUN"}
    (ROOT / "reports/transform_reference_report.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
