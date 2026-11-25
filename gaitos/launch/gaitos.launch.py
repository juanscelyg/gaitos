# Copyright 2026 Juan S. Cely G.
# Licensed under the Apache License, Version 2.0 (see LICENSE)
"""gaitos node of one robot, with robot_state_publisher (and RViz).

    ros2 launch gaitos gaitos.launch.py robot:=go2 params:="a.yaml,b.yaml" [rviz:=true]

Parameters, in order: gaitos/params/gaitos.yaml, the robot file
(gaitos_description/robots/<robot>_description/config/<robot>.yaml), the files of `params`
and the URDF / SRDF of the robot (robot_description, robot_description_semantic).
The namespace is the robot name (`namespace` to change it).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def description(robot, variant=''):
    """The URDF and SRDF of a robot; `variant` picks urdf/<robot>_<variant>.urdf (e.g. r1_dex3)
    and srdf/<robot>_<variant>.srdf when the variant has its own (other poses)"""
    base = os.path.join(get_package_share_directory('gaitos_description'), 'robots',
                        f'{robot}_description')
    with open(os.path.join(base, 'urdf', f'{robot}_{variant}.urdf' if variant else f'{robot}.urdf')) as f:
        urdf = f.read()
    srdf_path = os.path.join(base, 'srdf', f'{robot}_{variant}.srdf')
    if not variant or not os.path.exists(srdf_path):
        srdf_path = os.path.join(base, 'srdf', f'{robot}.srdf')
    with open(srdf_path) as f:
        srdf = f.read()
    return base, urdf, srdf


def gaitos_nodes(robot, namespace, extra_params, use_sim_time=False, rviz=False, overrides=None,
                 state_publisher=True, variant=''):
    """Nodes of one robot: gaitos_node, robot_state_publisher (unless another stack publishes
    the robot state) and optionally RViz"""
    base, urdf, srdf = description(robot, variant)
    params = [os.path.join(get_package_share_directory('gaitos'), 'params', 'gaitos.yaml'),
              os.path.join(base, 'config', f'{robot}.yaml')]
    params += [p for p in extra_params if p]
    params.append({'robot_description': urdf, 'robot_description_semantic': srdf,
                   'use_sim_time': use_sim_time, **(overrides or {})})
    nodes = [
        Node(package='gaitos_system', executable='gaitos_node', name='gaitos',
             namespace=namespace, parameters=params, output='screen', emulate_tty=True),
    ]
    if state_publisher:
        nodes.append(Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            namespace=namespace, output='log',
            parameters=[{'robot_description': urdf, 'use_sim_time': use_sim_time}]))
    if rviz:
        nodes.append(Node(
            package='rviz2', executable='rviz2', output='log', namespace=namespace,
            arguments=['-d', os.path.join(get_package_share_directory('gaitos_description'),
                                          'rviz', 'gaitos.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/robot_description', f'/{namespace}/robot_description')]))
    return nodes


def launch_setup(context):
    robot = LaunchConfiguration('robot').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context) or robot
    params = [p.strip() for p in LaunchConfiguration('params').perform(context).split(',')]
    return gaitos_nodes(robot, namespace, params,
                        LaunchConfiguration('use_sim_time').perform(context) == 'true',
                        LaunchConfiguration('rviz').perform(context) == 'true')


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='go2', description='go2, r1, nao'),
        DeclareLaunchArgument('namespace', default_value='', description='default: robot'),
        DeclareLaunchArgument('params', default_value='', description='extra parameter files, comma separated'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
