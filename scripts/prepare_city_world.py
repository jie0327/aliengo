#!/usr/bin/env python3
"""保留 CitySim 的米制街区布局，生成供 AlienGo 看点云的静态世界。"""
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def main():
    source = ROOT / "references/citysim/worlds/simple_city.world"
    tree = ET.parse(source)
    world = tree.getroot().find("world")
    excluded = {"actor", "person_standing", "prius_hybrid_sensors", "ocean"}
    for item in list(world):
        uris = [element.text.strip() for element in item.findall(".//uri") if element.text]
        if item.tag == "actor" or any(
                uri.startswith("model://") and uri[8:].split("/")[0] in excluded
                for uri in uris):
            world.remove(item)
            continue
        if item.tag in {"model", "include"}:
            static = item.find("static")
            if static is None:
                static = ET.SubElement(item, "static")
            static.text = "true"
    for parent in world.iter():
        for child in list(parent):
            if child.tag == "plugin":
                parent.remove(child)

    # 切换环境不切换已验证的 ODE 积分和接触求解配置。
    for physics in world.findall("physics"):
        world.remove(physics)
    physics = deepcopy(ET.parse(ROOT / ".ros1/src/unitree_gazebo/worlds/earth.world")
                       .getroot().find("world/physics"))
    gravity = physics.find("gravity")
    if gravity is not None:
        physics.remove(gravity)
    world.insert(0, physics)

    gui = world.find("gui")
    if gui is None:
        gui = ET.SubElement(world, "gui", {"fullscreen": "0"})
    camera = gui.find("camera")
    if camera is None:
        camera = ET.SubElement(gui, "camera", {"name": "user_camera"})
    pose = camera.find("pose")
    if pose is None:
        pose = ET.SubElement(camera, "pose")
    pose.text = "-62 -40 17 0 0.5 0.72"
    destination = ROOT / "ros1/worlds/city.world"
    destination.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(destination, encoding="utf-8", xml_declaration=True)
    print(destination)


if __name__ == "__main__":
    main()
