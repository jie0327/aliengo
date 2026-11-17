"""Independent PyKDL comparison of the lesson's pure-Python URDF FK."""
import importlib.util
import json
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import PyKDL as kdl

root = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("course_urdf_fk", root / "examples/lesson04/urdf_fk.py")
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)
urdf = root / ".ros1/aliengo.urdf"
model = module.UrdfModel(urdf)
elements = ET.parse(urdf).getroot().findall("joint")
by_child = {joint.find("child").get("link"): joint for joint in elements}
maximum = 0.0
checks = 0


def vec(text):
    return [float(part) for part in text.split()]


for leg in ("FR", "FL", "RR", "RL"):
    chain_elements = []
    child = leg + "_foot"
    while child != "trunk":
        item = by_child[child]
        chain_elements.insert(0, item)
        child = item.find("parent").get("link")
    chain = kdl.Chain()
    for element in chain_elements:
        origin = element.find("origin")
        xyz = vec(origin.get("xyz", "0 0 0")) if origin is not None else [0, 0, 0]
        rpy = vec(origin.get("rpy", "0 0 0")) if origin is not None else [0, 0, 0]
        frame = kdl.Frame(kdl.Rotation.RPY(*rpy), kdl.Vector(*xyz))
        if element.get("type") == "fixed":
            # Noetic's PyKDL exposes the fixed joint enum as "None".
            joint = kdl.Joint(element.get("name"), getattr(kdl.Joint, "None"))
        else:
            local_axis = kdl.Vector(*vec(element.find("axis").get("xyz")))
            joint = kdl.Joint(element.get("name"), frame.p, frame.M * local_axis, kdl.Joint.RotAxis)
        chain.addSegment(kdl.Segment(element.find("child").get("link"), joint, frame))
    solver = kdl.ChainFkSolverPos_recursive(chain)
    for angles in ((0, 0, 0), (0, 0.67, -1.3), (0.1, 1.3, -2.4), (0.2, 1.1, -1.8)):
        array = kdl.JntArray(3)
        for index, angle in enumerate(angles):
            array[index] = angle
        reference = kdl.Frame()
        assert solver.JntToCart(array, reference) >= 0
        positions = {f"{leg}_{part}_joint": angle for part, angle in zip(("hip", "thigh", "calf"), angles)}
        # Zero-angle case is an algebraic reference; calf zero is outside its limits.
        result = model.forward("trunk", leg + "_foot", positions, check_limits=angles != (0, 0, 0))
        errors = [abs(result[i][j] - reference.M[i, j]) for i in range(3) for j in range(3)]
        errors += [abs(result[i][3] - reference.p[i]) for i in range(3)]
        maximum = max(maximum, *errors)
        checks += 1
assert maximum < 1e-12, maximum
print(json.dumps({"checks": checks, "legs": 4, "poses_per_leg": 4,
                  "maximum_transform_error": maximum, "result": "PASS"}, indent=2))
