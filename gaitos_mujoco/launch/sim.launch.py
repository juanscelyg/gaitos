# Copyright 2026 Juan S. Cely G.
# Licensed under the Apache License, Version 2.0 (see LICENSE)
"""A robot in MuJoCo: the gaitos node runs the simulation in lockstep and publishes /clock.

    ros2 launch gaitos_mujoco sim.launch.py robot:=go2 [rviz:=true]
    ros2 service call /go2/mode/stand_up std_srvs/srv/Trigger

model:=<variant> picks another model of the robot: params/<robot>_<variant>_mujoco.yaml and
urdf/<robot>_<variant>.urdf (robot:=r1 model:=dex3, the R1 with Dex3-1 hands).
"""
import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def gaitos_launch():
    """The gaitos launch module (gaitos_nodes, description)"""
    path = os.path.join(get_package_share_directory("gaitos"), "launch", "gaitos.launch.py")
    spec = importlib.util.spec_from_file_location("gaitos_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def launch_setup(context):
    robot = LaunchConfiguration('robot').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context) or robot
    variant = LaunchConfiguration('model').perform(context)
    extra = [p.strip() for p in LaunchConfiguration('params').perform(context).split(',')]
    share = os.path.join(get_package_share_directory('gaitos_mujoco'), 'params')
    params = [os.path.join(share, f'{robot}_mujoco.yaml')]
    if variant:
        params.append(os.path.join(share, f'{robot}_{variant}_mujoco.yaml'))
        if not os.path.exists(params[-1]):
            raise RuntimeError(f'no model {variant} for {robot}: {params[-1]} not found')
    params += extra
    return gaitos_launch().gaitos_nodes(
        robot, namespace, params, use_sim_time=True,
        rviz=LaunchConfiguration('rviz').perform(context) == 'true',
        overrides={'publish_clock': True,
                   'real_time_factor': float(LaunchConfiguration('real_time_factor').perform(context))},
        state_publisher=LaunchConfiguration('robot_state_publisher').perform(context) == 'true',
        variant=variant)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='go2', description='go2, r1, nao'),
        DeclareLaunchArgument('model', default_value='',
                              description='model variant: dex3 (robot:=r1, Dex3-1 hands)'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('params', default_value='', description='extra parameter files'),
        DeclareLaunchArgument('real_time_factor', default_value='1.0'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('robot_state_publisher', default_value='true',
                              description='false when another stack publishes the robot state'),
        OpaqueFunction(function=launch_setup),
    ])
