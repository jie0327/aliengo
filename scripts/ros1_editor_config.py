#!/usr/bin/env python3
"""Generate host editor commands from the ROS container's build database."""

import json
import os
from pathlib import Path
import shlex


ROOT = Path(__file__).resolve().parents[1]
ROS = ROOT / ".ros1"
EDITOR = ROS / "editor"
SYSROOT = EDITOR / "sysroot"
SOURCE_SUFFIXES = {".c", ".cc", ".cpp"}
REFERENCE_PACKAGES = {
    "livox_laser_simulation": "references/livox_laser_simulation",
    "realsense_gazebo_plugin": "references/realsense_gazebo_plugin",
    "aliengo_description": "references/unitree_ros/robots/aliengo_description",
    "unitree_legged_msgs": "references/unitree_ros/unitree_ros_to_real/unitree_legged_msgs",
    "unitree_legged_control": "references/unitree_ros/unitree_legged_control",
    "unitree_controller": "references/unitree_ros/unitree_controller",
    "unitree_gazebo": "references/unitree_ros/unitree_gazebo",
    "unitree_guide": "references/unitree_guide/unitree_guide",
}


def host_path(value):
    """Normalize container paths before moving them into the exported sysroot."""
    path = os.path.normpath(value)
    for source, target in (
        ("/work", ROS),
        ("/opt/ros/noetic", SYSROOT / "opt/ros/noetic"),
        ("/usr/include", SYSROOT / "usr/include"),
    ):
        if path == source or path.startswith(source + "/"):
            return str(target) + path[len(source):]
    return path


def host_argument(argument):
    if argument.startswith("-D"):
        return argument.replace('\\"', '"')
    if argument.startswith("-I") and len(argument) > 2:
        return "-I" + host_path(argument[2:])
    if argument.startswith("/"):
        return host_path(argument)
    return argument


def build_entry(entry):
    arguments = [host_argument(arg) for arg in shlex.split(entry["command"])]
    arguments[0] = "/usr/bin/g++"
    arguments.insert(1, "--sysroot=" + str(SYSROOT))
    if not any(arg.startswith("-std=") for arg in arguments):
        # GCC 9 in the Noetic container defaults to GNU C++14.
        arguments.append("-std=gnu++14")
    return {
        "directory": host_path(entry["directory"]),
        "file": host_path(entry["file"]),
        "arguments": arguments,
    }


def source_files(directory):
    return sorted(path for path in directory.rglob("*") if path.suffix in SOURCE_SUFFIXES)


def example_entry(source, includes, ros=False):
    is_c = source.suffix == ".c"
    arguments = [
        "/usr/bin/gcc" if is_c else "/usr/bin/g++",
        "--sysroot=" + str(SYSROOT),
        "-std=gnu11" if is_c else "-std=c++11",
    ]
    if ros:
        arguments += [
            "-DROSCONSOLE_BACKEND_LOG4CXX",
            "-DROS_BUILD_SHARED_LIBS=1",
            '-DROS_PACKAGE_NAME="unitree_legged_real"',
        ]
    arguments += ["-I" + str(include) for include in includes]
    arguments += ["-c", str(source)]
    return {"directory": str(ROOT), "file": str(source), "arguments": arguments}


def main():
    container_entries = json.loads((ROS / "build/compile_commands.json").read_text())
    entries = []
    for container_entry in container_entries:
        # googletest belongs to the container toolchain, outside this workspace.
        if not container_entry["file"].startswith("/work/src/"):
            continue
        entry = build_entry(container_entry)
        entries.append(entry)
        package = Path(container_entry["file"]).relative_to("/work/src").parts[0]
        original = str(ROS / "src" / package)
        reference = str(ROOT / REFERENCE_PACKAGES[package])
        entries.append({
            "directory": entry["directory"],
            "file": entry["file"].replace(original, reference, 1),
            "arguments": [arg.replace(original, reference) for arg in entry["arguments"]],
        })

    sdk = ROOT / "references/unitree_legged_sdk"
    for source in source_files(sdk / "examples"):
        entries.append(example_entry(source, [sdk / "include"]))

    for guide in (ROS / "src/unitree_guide", ROOT / "references/unitree_guide/unitree_guide"):
        sdk = guide / "library/unitree_legged_sdk-3.8.0"
        for source in source_files(sdk / "example"):
            entries.append(example_entry(source, [sdk / "include"]))

    actuator = ROOT / "references/unitree_guide/unitree_actuator_sdk"
    for source in source_files(actuator / "src"):
        entries.append(example_entry(source, [actuator / "include"]))
    motor = actuator / "unitree_motor_ctrl"
    for source in source_files(motor / "src"):
        entries.append(example_entry(source, [motor / "include/unitree_motor_ctrl"]))

    real = ROOT / "references/unitree_ros/unitree_ros_to_real/unitree_legged_real"
    real_includes = [
        real / "include",
        ROOT / "references/unitree_guide/unitree_guide/library/unitree_legged_sdk-3.8.0/include",
        ROS / "devel/include",
        SYSROOT / "opt/ros/noetic/include",
        SYSROOT / "opt/ros/noetic/include/xmlrpcpp",
        SYSROOT / "usr/include/eigen3",
    ]
    for source in source_files(real / "src"):
        entries.append(example_entry(source, real_includes, ros=True))

    entries.sort(key=lambda entry: entry["file"])
    EDITOR.mkdir(parents=True, exist_ok=True)
    destination = EDITOR / "compile_commands.json"
    destination.write_text(json.dumps(entries, indent=2) + "\n")
    print(f"Wrote {len(entries)} editor commands to {destination}")


if __name__ == "__main__":
    main()
