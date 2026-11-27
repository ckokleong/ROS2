import math
import os
import re
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build_description(control):
    if not control.get('odometry', {}).get('enabled', False):
        raise ValueError('odometry.enabled must be true in the selected params file')
    if control.get('use_sim_time', False):
        raise ValueError('localization currently supports hardware system time only')
    frames = control['frames']
    odometry = control['odometry']
    imu = control['imu']
    gyro_variance = imu.get('gyro_variance', -1.0)
    if (not imu.get('enabled', False) or not isinstance(gyro_variance, (int, float)) or
            not math.isfinite(gyro_variance) or gyro_variance <= 0.0):
        raise ValueError('localization requires an enabled IMU with positive gyro_variance')
    base = odometry.get('base_frame', 'base_footprint')
    odom = odometry.get('odom_frame', 'odom')
    sensors = frames['sensors']
    height = frames['base_link_height_m']
    valid_name = lambda name: isinstance(name, str) and re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', name)
    if (not valid_name(base) or base in ('base_link', 'map') or
            not valid_name(odom) or odom in (base, 'base_link', 'map') or
            not isinstance(sensors, list) or not sensors or
            not isinstance(height, (int, float)) or not math.isfinite(height)):
        raise ValueError('invalid odometry frame names or frames configuration')
    if not all(valid_name(name) for name in sensors) or len(sensors) != len(set(sensors)):
        raise ValueError('sensor frame names must be valid and unique')
    if imu['frame_id'] not in sensors:
        raise ValueError('imu.frame_id must be listed in frames.sensors')

    robot = ET.Element('robot', {'name': 'studica_robot'})
    ET.SubElement(robot, 'link', {'name': base})
    ET.SubElement(robot, 'link', {'name': 'base_link'})

    def fixed_joint(name, parent, child, xyz, rpy):
        joint = ET.SubElement(robot, 'joint', {'name': name, 'type': 'fixed'})
        ET.SubElement(joint, 'parent', {'link': parent})
        ET.SubElement(joint, 'child', {'link': child})
        ET.SubElement(joint, 'origin', {
            'xyz': ' '.join(str(v) for v in xyz),
            'rpy': ' '.join(str(v) for v in rpy),
        })

    fixed_joint('footprint_to_base', base, 'base_link', [0.0, 0.0, height], [0.0, 0.0, 0.0])
    for name in sensors:
        if name in (base, 'base_link', odom, 'map'):
            raise ValueError('invalid sensor frame name')
        sensor = frames[name]
        xyz, rpy = sensor['xyz'], sensor['rpy']
        if any(not isinstance(v, list) or len(v) != 3 or
               any(not isinstance(n, (int, float)) or not math.isfinite(n) for n in v)
               for v in (xyz, rpy)):
            raise ValueError(f'frame {name} requires finite xyz and rpy triples')
        ET.SubElement(robot, 'link', {'name': name})
        fixed_joint('base_to_' + name, 'base_link', name, xyz, rpy)

    return ET.tostring(robot, encoding='unicode')


def build_nodes(context):
    params_file = LaunchConfiguration('params_file').perform(context)
    with open(params_file, encoding='utf-8') as config:
        params = yaml.safe_load(config)
    control = params['control_server']['ros__parameters']
    description = build_description(control)
    odometry = control['odometry']
    imu = control['imu']
    base = odometry.get('base_frame', 'base_footprint')
    odom = odometry.get('odom_frame', 'odom')
    ekf = params['ekf_filter_node']['ros__parameters']
    if ekf.get('two_d_mode') is not True or ekf.get('publish_tf') is not True:
        raise ValueError('EKF must enable two_d_mode and publish_tf')
    for key, selected in [('twist0_config', (6, 7, 11)), ('imu0_config', (11,))]:
        values = ekf.get(key)
        expected = [i in selected for i in range(15)]
        if not isinstance(values, list) or values != expected:
            raise ValueError(f'{key} must select only the documented measurements')
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}], output='screen'),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             parameters=[params_file, {
                 'odom_frame': odom,
                 'world_frame': odom,
                 'base_link_frame': base,
                 'twist0': odometry.get('wheel_twist_topic', '/wheel_twist'),
                 'imu0': imu['topic'],
             }],
             remappings=[('odometry/filtered', '/odom')], output='screen'),
    ]


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('studica_control'), 'config', 'params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Same params file used by studica_launch.py'),
        OpaqueFunction(function=build_nodes),
    ])
