#!/usr/bin/env python3
"""把 Fuel Depot 的原始模型适配为 Gazebo Classic 静态仓库。"""
from copy import deepcopy
import os
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def write_xml(tree, path):
    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8", xml_declaration=True)


def main():
    source = ROOT / "references/fuel_models/Depot"
    destination = ROOT / "ros1/worlds/models/Depot"
    destination.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(source / "model.sdf")
    model = tree.getroot().find("model")
    model.find("static").text = "true"
    # 风扇停在初始位置；保留各 link 和 visual 的相对位姿。
    for child in list(model):
        if child.tag in {"joint", "plugin"}:
            model.remove(child)
    # Classic 可直接读取 DAE 中的 albedo/UV；不要再用 SDF fallback 覆盖它。
    for visual in model.findall(".//visual"):
        for material in visual.findall("material"):
            visual.remove(material)
    for uri in model.findall(".//uri"):
        uri.text = "model://Depot/" + uri.text.strip()
    # 原模型两盏点光源的衰减系数全为 0；Classic 的光照公式会除以 0。
    for light in model.findall(".//light"):
        attenuation = light.find("attenuation")
        attenuation.find("constant").text = "0.5"
        attenuation.find("linear").text = "0.01"
        attenuation.find("quadratic").text = "0.002"

    # 完整 FLOOR 同时包含底层和夹层。用它替换原 plane，避免双地面接触。
    main_link = model.find("link[@name='main']")
    main_link.remove(main_link.find("collision"))

    for link in model.findall("link"):
        for visual in link.findall("visual"):
            # 原模型只有地面碰撞。CPU Ray 必须能命中墙、柱、箱子等网格。
            collision = ET.SubElement(link, "collision", {
                "name": visual.get("name") + "_collision"})
            pose = visual.find("pose")
            if pose is not None:
                collision.append(deepcopy(pose))
            collision.append(deepcopy(visual.find("geometry")))
        # 与当前平地环境使用相同的脚底摩擦参数。
        for collision in link.findall("collision"):
            surface = ET.SubElement(collision, "surface")
            contact = ET.SubElement(surface, "contact")
            ET.SubElement(contact, "collide_bitmask").text = "0xffff"
            ode = ET.SubElement(ET.SubElement(surface, "friction"), "ode")
            ET.SubElement(ode, "mu").text = "100"
            ET.SubElement(ode, "mu2").text = "50"
    write_xml(tree, destination / "model.sdf")
    config = ET.parse(source / "model.config")
    config.getroot().find("sdf").set("version", "1.6")
    write_xml(config, destination / "model.config")
    for directory in ("meshes", "materials"):
        target = destination / directory
        if not target.is_symlink():
            target.symlink_to(os.path.relpath(source / directory, destination),
                              target_is_directory=True)

    world_tree = ET.ElementTree(ET.Element("sdf", {"version": "1.6"}))
    world = ET.SubElement(world_tree.getroot(), "world", {"name": "default"})
    physics = deepcopy(ET.parse(ROOT / ".ros1/src/unitree_gazebo/worlds/earth.world")
                       .getroot().find("world/physics"))
    physics.remove(physics.find("gravity"))
    world.append(physics)
    ET.SubElement(world, "gravity").text = "0 0 -9.81"
    scene = ET.SubElement(world, "scene")
    ET.SubElement(scene, "ambient").text = "0.4 0.4 0.4 1"
    for name in ("sun", "Depot"):
        ET.SubElement(ET.SubElement(world, "include"), "uri").text = "model://" + name
    camera = ET.SubElement(ET.SubElement(world, "gui"), "camera", {"name": "user_camera"})
    ET.SubElement(camera, "pose").text = "-8 -2 2 0 0.35 0.6"
    write_xml(world_tree, ROOT / "ros1/worlds/depot.world")
    print("已生成 Depot Classic 模型与 ros1/worlds/depot.world")


if __name__ == "__main__":
    main()
