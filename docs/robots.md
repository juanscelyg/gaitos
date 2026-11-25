# Robots

[← gaitos](../README.md#documentation)

| | Go2 | R1 | NAO |
|---|---|---|---|
| Joints | 12 | 26 | 25 actuated (`RHipYawPitch` mimics `LHipYawPitch`) |
| URDF | unitree_ros `go2_description` (+ Gazebo variant) | unitree_ros `r1_description` (+ Gazebo variant) | ros-naoqi `nao_robot` (V5 kinematics and masses, V6 motor limits; + simulation variant) |
| MuJoCo model | unitree_mujoco `go2` | unitree_mujoco `r1`; `r1_dex3.xml` with Dex3-1 hands and stock colours (`model:=dex3`) | `make_mjcf.py` from the simulation variant and the Fuel meshes |
| Motors | `PdPassthrough` | `PdPassthrough` | `PositionServo` (stiffness 0..1; in simulation a PD servo, `servo: {kp, kd}`) |
| Contacts | foot_force / simulator forces | joint torques (real), simulator forces | FSR (simulator forces) |
| Backends | MuJoCo, Gazebo, unitree_sdk2 | MuJoCo, Gazebo, unitree_sdk2 | MuJoCo, Gazebo, `FakeHardware` (NAOqi backend to be written) |

**NAO.** `ros2 launch gaitos_mujoco sim.launch.py robot:=nao` (or `gaitos_gazebo`, 0.5 ms
world). It starts in `crouch`; `mode/stand_init`, `mode/balance_stand` (body posture with
`goal_pose` in `torso`, relative to the body), `mode/locomotion` (a `cmd_vel` in
balance_stand starts it) and `mode/crouch`.

- Walking: the kinematic `GaitController` (alternating steps, 0.6 s, 2 cm of sway, velocity PI
  loop): ≈0.06 m/s and 0.3 rad/s. The LIPM / ZMP controller is not used on the NAO: its
  stabilizer acts through a torque feedforward that position servos do not have, and its
  massless legs model does not suit the NAO (legs ≈45 % of the mass).
- Turning: both legs share the HipYawPitch motor (on mirrored 45° axes), so the feet can only
  turn against each other relative to the torso. `LegIk` solves legs that share joints
  together, leaving free the yaw common to every foot, and the gait plans the yaw of the feet;
  the body turns over the stance foot.
- Joints over the controller (`external_joints`): head and arms always, any joint in the
  posture modes (`joint_targets`, naoqi_driver `/joint_angles`).

- Simulation model: `config/gazebo.yaml` → `urdf/nao_gazebo.urdf` (`make_gazebo_urdf.py`:
  primitive collisions with the soles from the FSR frames, no Gazebo classic plugins, links
  renamed where they clash with a joint name in SDF, no `<mimic>`) → `mjcf/nao.xml`
  (`make_mjcf.py` with `config/mujoco.yaml`: free base, IMU at the torso accelerometer, a motor
  per joint with the torque limit of the URDF, the meshes of the URDF as visual geoms with
  the texture of the robot, the collision primitives in the hidden group 3). The physics never
  depends on the meshes. To make it again (after `colcon build`, which converts the meshes):

  ```bash
  cd src/gaitos/gaitos_description/robots/nao_description
  python3 ../../scripts/make_mjcf.py urdf/nao_gazebo.urdf config/mujoco.yaml mjcf/nao.xml \
    --meshes $(ros2 pkg prefix gaitos_description)/share/nao_description/mjcf/meshes
  ```
- Joint limits of the URDF: the NAO V6 motors (doc.aldebaran.com 2-8, motors_naov6): torque =
  stall torque × reduction (e.g. 8.5 N·m for the pitch joints of the legs, 13.1 N·m for
  HipYawPitch, HipRoll and AnkleRoll), speed = no-load speed / reduction.
- Mimic joints (`RHipYawPitch`, which shares the motor of `LHipYawPitch`, and the fingers,
  which follow `LHand` / `RHand`): both simulator backends drive them to follow the command of
  their master.
- Servos: the commanded stiffness (0..1, NAOqi) scales a PD of `servo.kp` / `servo.kd`:
  200 N·m/rad and 1.0 N·m·s/rad in MuJoCo, 60 and 0.8 in Gazebo (stiffer servos make the feet
  bounce on its contacts; softer ones in MuJoCo give speed spikes the joint limits stop).
- Meshes (looks only): the `.dae` of the model "NAO with Ignition position controller" of
  [Gazebo Fuel](https://app.gazebosim.org/OpenRobotics/fuel/models/NAO%20with%20Ignition%20position%20controller) (OpenRobotics, from osrf/robocup3ds, **CC BY 4.0**), downloaded when building
  (FetchContent) and installed in `robots/nao_description/meshes/V40` with their texture
  (RViz, Gazebo) and as OBJ in `mjcf/meshes` (MuJoCo). Attribution in
  `robots/nao_description/MESHES_LICENSE.md`. The collisions of `urdf/nao.urdf` use the same
  `.dae`.

Adding a robot: URDF (+ meshes), SRDF (legs, `all` group, poses), a parameter file in
`gaitos_description/robots/<robot>_description/config/`, and a backend (MuJoCo with an
MJCF or the URDF, Gazebo with the URDF, or a new `HardwareInterface` plugin).
