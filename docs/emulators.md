# Vendor interfaces in simulation

[← gaitos](../README.md#documentation)

The simulated robot with the interface of the real one, so programs written for the real robot run against MuJoCo or Gazebo unchanged. They are node extensions (`gaitos::NodeExtension`, see [ROS 2 interfaces](interfaces.md)).

## Unitree robots in simulation (R1 and Go2)

The robot in MuJoCo or Gazebo with the DDS interface of the real one, so a ROS 2 program
written for the real robot (with the unitree_ros2 messages: unitree_r1 for the R1, a
SportClient over `/api/sport` for the Go2) runs on it unchanged. gaitos stands for Unitree's
controller: its modes are the FSM / sport modes and its walking the built-in gait.

```bash
export ROS_DOMAIN_ID=<free domain>   # these are the topic names of the real robot
ros2 launch gaitos_unitree_sim sim.launch.py robot:=r1|go2 [simulator:=mujoco|gazebo] [headless:=true] [bringup:=true]
```

The gaitos node runs in the namespace `gaitos` (`/gaitos/mode`, `/gaitos/status`,
`/gaitos/ground_truth`).

**R1** (`UnitreeHgEmulator`). `bringup:=true` also starts `r1_core` of unitree_r1 (r1_state,
robot_state_publisher, r1_locomotion, r1_gesture); gaitos leaves the robot state (TF,
joint_states, robot_description) to it (`publish_tf`, `publish.*` false, no
robot_state_publisher).

```bash
ros2 service call /r1/r1_locomotion/stand std_srvs/srv/Trigger     # FSM 4  -> stand_up
ros2 service call /r1/r1_locomotion/walk std_srvs/srv/Trigger      # FSM 811 -> balance_stand
ros2 topic pub -r 10 /r1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
```

| Robot interface | Emulated as |
|---|---|
| `/lowstate` (`unitree_hg/LowState`, 500 Hz) | joints in the documented motor slots (`motor_index`), IMU, tick |
| `/odommodestate` (`unitree_go/SportModeState`) | the gaitos estimate |
| `/secondary_imu`, `/lf/bmsstate`, `/arm/action/state` | the IMU, a full battery, nothing holding |
| `/api/sport`: GetFsmId, SetFsmId | gaitos mode ↔ FSM (`fsm.*`): 0 `zero_torque`, 1 `damp`, 4 `stand_up`, 811 `balance_stand` / `locomotion_lipm` |
| `/api/sport`: SetVelocity | body twist kept for its `duration`, only in FSM 811 |
| `/api/arm` | empty action list; the actions answer 3203 (not implemented) |
| `/arm_sdk` (`unitree_hg/LowCmd`), with `arm_sdk:=true` | in the walking FSM only: q, dq, tau, kp, kd of the arms, waist yaw and head (SRDF group `arm_sdk`) blended over the walking controller by the weight in `mode_pr` (`Gaitos::set_joint_commands`). While the weight is above 0: GetFsmId 816, SetVelocity refused (127), SetFsmId(811) refused (1001), what an R1 answered in tests ([`params/r1_arm_sdk.yaml`](../gaitos_unitree_sim/params/r1_arm_sdk.yaml)) |
| `/dex3/{left,right}/cmd` → `/dex3/{left,right}/state` (`unitree_hg/HandCmd`, `HandState`), with `dex3:=true` | kinematic Dex3-1 hands (`Dex3Hand`): each motor with stiffness follows its command as a first-order lag, within its speed and joint range; no pressure, no faults. The simulated R1 has no hands, so nothing is grasped ([`params/r1_dex3.yaml`](../gaitos_unitree_sim/params/r1_dex3.yaml)) |

```bash
ros2 launch gaitos_unitree_sim sim.launch.py robot:=r1 bringup:=true arm_sdk:=true dex3:=true [model:=dex3]
```

The walking is the gaitos LIPM: unitree_r1 asks for at least 0.3 m/s, the controller caps it at
0.15 m/s. By default the simulated R1 has no hands: a gravity compensation computed with the
Dex3 masses (e.g. unitree_r1's r1_arm) over-compensates the arms there. `model:=dex3` (MuJoCo
only) simulates the R1 with its Dex3-1 hands (`gaitos_mujoco model:=dex3`): their mass and look,
with the stand pose of `srdf/r1_dex3.srdf`. Its fingers do not follow `/dex3`, which stays
kinematic.

The balance of `balance_stand` (FSM 811 while not walking) has no feedback of the centre of mass: an
arm held well forward through `arm_sdk` (left shoulder pitch −0.8 rad) tips the R1 over, with or
without hands; −0.3 rad holds. On the robot the Unitree controller compensates it.

unitree_r1's `r1_control.launch.py` (r1_arm, r1_hand) runs against this interface, but on rolling
its spawners need `--param-file` with its `controllers.yaml`: without it the trajectory controllers
fail to load (`joints` empty), since the controllers no longer take their parameters from the
controller manager's file.

**Go2** (`UnitreeGoEmulator`). gaitos also publishes its own state in `/gaitos` (TF,
joint_states, robot_state_publisher). The sport service takes the JSON of the official
SportClient (checked against unitree_sdk2): `{"x", "y", "z"}` for Euler and Move,
`{"data"}` for the rest.

| Robot interface | Emulated as |
|---|---|
| `/lowstate`, `/lf/lowstate` (`unitree_go/LowState`, 500 / 20 Hz) | motors in the model order (FR, FL, RR, RL × hip, thigh, calf), IMU, foot forces, a full battery |
| `/sportmodestate`, `/lf/sportmodestate` (`unitree_go/SportModeState`, 200 / 20 Hz) | the gaitos estimate, sport mode (`sport_modes.*`: 1 balance stand, 3 locomotion, 5 lie down, 6 joint lock, 7 damping, 8 recovery, 10 sit), gait, feet in the body frame and their forces |
| `/api/sport` 1001 Damp, 1002 BalanceStand, 1004 StandUp, 1005 StandDown, 1006 RecoveryStand, 1009 Sit, 1010 RiseSit | a gaitos mode each (`api.*`) |
| `/api/sport` 1008 Move, 1003 StopMove | body twist kept for `move_timeout` (1 s), in balance stand or walking |
| `/api/sport` 1007 Euler | body roll, pitch, yaw while in balance stand |
| `/api/sport` 1015 SpeedLevel, 1027 SwitchJoystick, 1028 Pose | accepted, no effect |
| `/api/sport` tricks and other gaits (Hello, dances, flips, ClassicWalk...) | 3203 (not implemented) |

Every service also answers the internal ids 1 (api version, `1.0.0.1`) and 2 (no-op) of
unitree_sdk2. Neither emulator fills the CRC of `lowstate`.

Programs built on unitree_sdk2 itself (its bundled CycloneDDS 0.10.2) cannot talk to the
emulator: that CycloneDDS crashes (`ddsi_xt_type_init_impl with invalid type object`) as soon
as it discovers the endpoints of a current ROS 2 middleware (CycloneDDS 11 or Fast DDS, both
tried). They need the ROS 2 messages (unitree_ros2) instead.

## NAO in simulation behind naoqi_driver

The NAO in MuJoCo or Gazebo with the ROS 2 interface of naoqi_driver, so a program written for
the real NAO through naoqi_driver runs on it unchanged. gaitos stands for NAOqi ALMotion: its
posture modes, its stance and its walk.

```bash
export ROS_DOMAIN_ID=<free domain>   # these are the topic names of naoqi_driver
ros2 launch gaitos_naoqi_sim sim.launch.py [simulator:=mujoco|gazebo] [headless:=true] [rviz:=true]
ros2 service call /gaitos/mode/stand_init std_srvs/srv/Trigger     # NAOqi wakeUp (it starts in crouch)
ros2 topic pub --once /joint_angles naoqi_bridge_msgs/msg/JointAnglesWithSpeed \
  "{joint_names: [HeadYaw, HeadPitch], joint_angles: [0.5, 0.2], speed: 0.2}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}}"   # holds until a zero twist
ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_footprint}, pose: {position: {x: 0.3}, orientation: {w: 1.0}}}"
```

| naoqi_driver | Emulated as |
|---|---|
| `/joint_states` (50 Hz) | the 26 joints of the body (RHipYawPitch from LHipYawPitch); a robot_state_publisher at the root namespace publishes the link TF |
| TF `odom → base_link → base_footprint`, `/odom` (15 Hz), `/imu/torso` (10 Hz) | the gaitos estimate; base_footprint on the ground under the torso |
| `/cmd_vel` (ALMotion.move) | the velocity holds until the next message; only standing (from stand_init it enters balance_stand) |
| `/joint_angles` (setAngles / changeAngles) | `Gaitos::set_joint_targets`: names or one chain (Body, Head, LArm, RArm, LLeg, RLeg), speed as a fraction of the max speed; head and arms always, any joint in the posture modes |
| `/move_base_simple/goal` (moveTo, `odom` or `base_footprint`) | the walk with a pose goal |
| `/speech` | logged |

Not emulated: cameras, sonars, bumpers, touch sensors, audio, info, diagnostics. NAOqi
wakeUp / rest are the gaitos modes `stand_init` / `crouch` (`/gaitos/mode/<name>`).
