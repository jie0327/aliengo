"""Experimental URDF-to-MJCF conversion, CPU only, no mesh rendering."""
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

JOINT_NAMES = tuple(f'{leg}_{joint}_joint' for leg in ('FR', 'FL', 'RR', 'RL')
                    for joint in ('hip', 'thigh', 'calf'))


def convert(urdf_path: Path, output_path: Path) -> None:
    root = ET.parse(urdf_path).getroot()
    if root.tag != 'robot' or root.get('name') != 'aliengo':
        raise ValueError('Expected the expanded official AlienGo URDF')
    if root.find("link[@name='base']") is None or root.find("link[@name='ground_root']") is not None:
        raise ValueError('Expected a free AlienGo base without an existing ground_root')
    for link in root.findall('link'):
        for element in link.findall('visual'):
            link.remove(element)
    for element in list(root):
        if element.tag in ('gazebo', 'transmission', 'material'):
            root.remove(element)
    ET.SubElement(root, 'link', {'name': 'ground_root'})
    joint = ET.SubElement(root, 'joint', {'name': 'base_floating', 'type': 'floating'})
    ET.SubElement(joint, 'parent', {'link': 'ground_root'})
    ET.SubElement(joint, 'child', {'link': 'base'})
    with tempfile.TemporaryDirectory(prefix='aliengo-urdf-') as folder:
        clean = Path(folder) / 'floating.urdf'
        generated = Path(folder) / 'imported.xml'
        ET.ElementTree(root).write(clean, encoding='unicode')
        source = mujoco.MjModel.from_xml_path(str(clean))
        if source.nq != 19 or source.nv != 18 or source.njnt != 13:
            raise ValueError('Unexpected AlienGo floating-base or joint configuration')
        names = tuple(mujoco.mj_id2name(source, mujoco.mjtObj.mjOBJ_JOINT, i)
                      for i in range(1, source.njnt))
        if names != JOINT_NAMES:
            raise ValueError(f'Unexpected joint order: {names}')
        base_id = mujoco.mj_name2id(source, mujoco.mjtObj.mjOBJ_BODY, 'base')
        if base_id < 0 or source.body_mass[base_id] <= 0:
            raise ValueError('Base inertia unavailable')
        mujoco.mj_saveLastXML(str(generated), source)
        converted = ET.parse(generated)
        mjroot = converted.getroot()
        world = mjroot.find('worldbody')
        base = world.find("body[@name='base']")
        if base is None:
            raise ValueError('MuJoCo conversion did not preserve the expected base')
        # SaveLastXML may omit merged-base inertia and rederive it from collision
        # volumes. Preserve the importer-computed mass, COM and inertia explicitly.
        existing = base.find('inertial')
        if existing is not None:
            base.remove(existing)
        def vector(values):
            return ' '.join(f'{float(v):.17g}' for v in values)
        ET.SubElement(base, 'inertial', {
            'pos': vector(source.body_ipos[base_id]),
            'quat': vector(source.body_iquat[base_id]),
            'mass': f'{source.body_mass[base_id]:.17g}',
            'diaginertia': vector(source.body_inertia[base_id]),
        })
        ET.SubElement(world, 'geom', {'name': 'floor', 'type': 'plane',
                                      'size': '10 10 .1', 'friction': '.8 .005 .0001'})
        option = mjroot.find('option')
        if option is None:
            option = ET.SubElement(mjroot, 'option')
        option.set('timestep', '.002')
        option.set('integrator', 'implicitfast')
        actuators = ET.SubElement(mjroot, 'actuator')
        for name in JOINT_NAMES:
            # Limits are taken from the imported URDF, not a real-robot contract.
            idx = mujoco.mj_name2id(source, mujoco.mjtObj.mjOBJ_JOINT, name)
            lo, hi = source.jnt_actfrcrange[idx]
            if not lo < 0 < hi:
                raise ValueError(f'Missing effort limits for {name}')
            ET.SubElement(actuators, 'motor', {'name': name + '_motor', 'joint': name,
                                               'ctrllimited': 'true',
                                               'ctrlrange': f'{lo:.17g} {hi:.17g}'})
        converted.write(output_path, encoding='unicode', xml_declaration=True)
        check = mujoco.MjModel.from_xml_path(str(output_path))
        if check.nu != 12 or not np.isclose(check.body_mass.sum(), source.body_mass.sum(), rtol=1e-8):
            raise ValueError('Converted mass or actuator count does not match source')
