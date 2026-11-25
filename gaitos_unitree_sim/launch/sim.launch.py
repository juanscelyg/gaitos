# Copyright 2026 Juan S. Cely G.
# Licensed under the Apache License, Version 2.0 (see LICENSE)
"""A simulated Unitree robot that speaks the DDS interface of the real one.

    ros2 launch gaitos_unitree_sim sim.launch.py [robot:=r1|go2] [simulator:=mujoco|gazebo]
                                                 [headless:=true] [bringup:=true]
                                                 [arm_sdk:=true] [dex3:=true] [model:=dex3]

gaitos runs the simulator in the namespace `gaitos` (its own mode / status topics) and, through
its emulator extension, publishes and serves what the robot does:
 - r1 (UnitreeHgEmulator): /lowstate, /odommodestate, /secondary_imu, /lf/bmsstate,
   /arm/action/state, /api/sport (FSM, SetVelocity) and /api/arm. The robot state (TF,
   joint_states, robot_description) is left to unitree_r1: bringup:=true also starts its
   r1_core (r1_state, robot_state_publisher, r1_locomotion, r1_gesture) if it is installed.
   arm_sdk:=true also takes the upper body commands of /arm_sdk (params/r1_arm_sdk.yaml), and
   dex3:=true emulates the Dex3-1 hands, kinematic (params/r1_dex3.yaml). model:=dex3 (MuJoCo
   only) simulates the R1 with its Dex3-1 hands (gaitos_mujoco model:=dex3): their mass and
   look; the fingers of the model do not follow /dex3.
 - go2 (UnitreeGoEmulator): /lowstate, /lf/lowstate, /sportmodestate, /lf/sportmodestate and
   /api/sport (StandUp, BalanceStand, Move, Euler...); gaitos also publishes its own state
   (TF, joint_states with robot_state_publisher) in /gaitos.

Use a ROS_DOMAIN_ID of its own: these are the topic names of the real robot.
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context):
    robot = LaunchConfiguration('robot').perform(context)
    simulator = LaunchConfiguration('simulator').perform(context)
    if simulator not in ('mujoco', 'gazebo'):
        raise RuntimeError(f"simulator must be mujoco or gazebo, not '{simulator}'")
    model = LaunchConfiguration('model').perform(context)
    if model and simulator != 'mujoco':
        raise RuntimeError(f'model:={model} is only for simulator:=mujoco')
    emulator = os.path.join(get_package_share_directory('gaitos_unitree_sim'), 'params',
                            f'{robot}_unitree_sim.yaml')
    extra = [p.strip() for p in LaunchConfiguration('params').perform(context).split(',') if p.strip()]
    # Optional R1 interfaces, before the user's files so that those still override them
    for flag, file in (('dex3', 'r1_dex3.yaml'), ('arm_sdk', 'r1_arm_sdk.yaml')):
        if LaunchConfiguration(flag).perform(context) == 'true':
            if robot != 'r1':
                raise RuntimeError(f'{flag}:=true is only for robot:=r1')
            extra.insert(0, os.path.join(get_package_share_directory('gaitos_unitree_sim'),
                                         'params', file))
    # The R1 state is published by unitree_r1; the Go2 one by gaitos itself
    state_by_gaitos = robot != 'r1'
    args = {'robot': robot, 'namespace': LaunchConfiguration('namespace').perform(context),
            'params': ','.join([emulator] + extra),
            'robot_state_publisher': 'true' if state_by_gaitos else 'false'}
    if model:
        args['model'] = model
    if simulator == 'gazebo':
        args['headless'] = LaunchConfiguration('headless').perform(context)
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(f'gaitos_{simulator}'), 'launch', 'sim.launch.py')),
        launch_arguments=args.items())]
    if LaunchConfiguration('bringup').perform(context) == 'true' and robot == 'r1':
        try:
            core = os.path.join(get_package_share_directory('r1_bringup'), 'launch', 'r1_core.launch.py')
            actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(core)))
        except PackageNotFoundError:
            actions.append(LogInfo(msg='bringup:=true but r1_bringup (unitree_r1) is not installed'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='r1', description='r1 or go2'),
        DeclareLaunchArgument('simulator', default_value='mujoco', description='mujoco or gazebo'),
        DeclareLaunchArgument('namespace', default_value='gaitos',
                              description='of the gaitos node; not r1, which unitree_r1 uses'),
        DeclareLaunchArgument('params', default_value='', description='extra parameter files'),
        DeclareLaunchArgument('headless', default_value='false', description='Gazebo only'),
        DeclareLaunchArgument('bringup', default_value='false',
                              description='r1: also start r1_core of unitree_r1'),
        DeclareLaunchArgument('arm_sdk', default_value='false',
                              description='r1: emulate arm_sdk (upper body commands, FSM 816)'),
        DeclareLaunchArgument('dex3', default_value='false',
                              description='r1: emulate the Dex3-1 hands (kinematic)'),
        DeclareLaunchArgument('model', default_value='',
                              description='mujoco: model variant, dex3 (R1 with Dex3-1 hands)'),
        OpaqueFunction(function=launch_setup),
    ])
