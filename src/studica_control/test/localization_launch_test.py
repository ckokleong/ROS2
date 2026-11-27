import copy
import importlib.util
import pathlib
import tempfile
import unittest
import xml.etree.ElementTree as ET

import yaml
from launch import LaunchContext

path = pathlib.Path(__file__).parent.parent / 'launch' / 'localization.launch.py'
spec = importlib.util.spec_from_file_location('localization_launch', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class DescriptionTest(unittest.TestCase):
    def setUp(self):
        self.control = {
            'odometry': {'enabled': True, 'base_frame': 'base_footprint', 'odom_frame': 'odom'},
            'imu': {'enabled': True, 'gyro_variance': 0.01, 'frame_id': 'imu_link'},
            'frames': {
                'base_link_height_m': 0.15,
                'sensors': ['imu_link', 'laser_link'],
                'imu_link': {'xyz': [0.1, 0.0, 0.03], 'rpy': [0.0, 0.0, 0.2]},
                'laser_link': {'xyz': [0.2, 0.0, 0.4], 'rpy': [0.0, 0.0, 0.0]},
            },
        }

    def test_fixed_tree_and_mounts(self):
        robot = ET.fromstring(module.build_description(self.control))
        joints = {joint.attrib['name']: joint for joint in robot.findall('joint')}
        self.assertEqual(joints['footprint_to_base'].find('parent').attrib['link'], 'base_footprint')
        self.assertEqual(joints['footprint_to_base'].find('child').attrib['link'], 'base_link')
        self.assertEqual(joints['footprint_to_base'].find('origin').attrib['xyz'], '0.0 0.0 0.15')
        laser = joints['base_to_laser_link']
        self.assertEqual(laser.find('parent').attrib['link'], 'base_link')
        self.assertEqual(laser.find('origin').attrib['xyz'], '0.2 0.0 0.4')
        self.assertEqual(joints['base_to_imu_link'].find('origin').attrib['rpy'], '0.0 0.0 0.2')

    def test_invalid_configuration_rejected(self):
        for mutation in ('duplicate', 'missing_imu', 'nan'):
            control = copy.deepcopy(self.control)
            if mutation == 'duplicate':
                control['frames']['sensors'].append('imu_link')
            elif mutation == 'missing_imu':
                control['frames']['sensors'].remove('imu_link')
            else:
                control['frames']['imu_link']['xyz'][0] = float('nan')
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                module.build_description(control)

    def test_launch_uses_ekf_configuration(self):
        template = path.parent.parent / 'config' / 'params_template.yaml'
        with template.open(encoding='utf-8') as f:
            config = yaml.safe_load(f)
        config['control_server']['ros__parameters']['odometry']['enabled'] = True
        imu = config['control_server']['ros__parameters']['imu']
        imu['enabled'] = True
        imu['gyro_variance'] = 0.01
        with tempfile.NamedTemporaryFile(mode='w+', suffix='.yaml') as file:
            yaml.safe_dump(config, file)
            file.flush()
            context = LaunchContext()
            context.launch_configurations['params_file'] = file.name
            self.assertEqual(len(module.build_nodes(context)), 2)
            config['ekf_filter_node']['ros__parameters']['twist0_config'][7] = False
            file.seek(0)
            file.truncate()
            yaml.safe_dump(config, file)
            file.flush()
            with self.assertRaises(ValueError):
                module.build_nodes(context)


if __name__ == '__main__':
    unittest.main()
