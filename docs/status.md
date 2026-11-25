# Status

[← gaitos](../README.md#documentation)

Verified on this machine with ROS 2 rolling (`colcon test`: 64 tests, plus launch runs):

- Go2, `ros2 launch gaitos_mujoco sim.launch.py`: mode services, `cmd_vel` walking (2.4 m),
  odometry vs ground truth (2.21 vs 2.41 m), status, TF with `robot_state_publisher`.
  In the tests: stand up, body pose, trot following vx, vy and wz, sit, goal relative to
  the body, convex MPC walking and turning.
- Go2, `ros2 launch gaitos_gazebo sim.launch.py`: node inside gz-sim, services, `cmd_vel`
  walking, `/clock`, the whole MuJoCo sequence of the quick start (body pose, walking, goal
  relative to the body, convex MPC walking and stopping, stand down).
- Go2, `ros2 launch gaitos_unitree robot.launch.py dds_sim:=true` against `go2_dds_sim`:
  stand up and walk through unitree_sdk2; read only sends no command.
- R1 with Dex3-1 hands (`model:=dex3`, MuJoCo): through `ros2 launch`, gaitos and
  robot_state_publisher get `r1_dex3.urdf` (30.53 kg) and `r1_dex3.srdf` (stand pose with ankle
  pitch −0.28); stands in `stand_up` and `balance_stand` and walks from there with a twist (test
  `R1Dex3ModelStandsAndWalks`). `squat` tips it over.
- R1 with Dex3-1 hands behind the R1 interface (`gaitos_unitree_sim model:=dex3 bringup:=true`,
  MuJoCo): r1_locomotion stand, walk (`balance_stand`) and `cmd_vel` 0.3 m/s (0.56 m), and an
  `arm_sdk` left shoulder pitch of −0.3 rad held in `balance_stand` (pitch 0.04 rad).
- R1: MuJoCo stand, body pose, squat and LIPM walking (forward ≤ 0.15 m/s, turning);
  Gazebo (0.5 ms world) stand and LIPM walking (≈1 m in the launch run, longer walks still
  fall after 15–20 s).
- NAO in MuJoCo and Gazebo: from `crouch` (torso at 0.23 m) to `stand_init` (0.32 m), steady in
  `balance_stand` with body posture, and back to `crouch` (test `NaoStandsFromCrouch` in MuJoCo).
  Walking forward ≈0.06 m/s and turning in place 0.33 rad/s (test `NaoWalks`); Gazebo walked
  0.8 m in 14 s.
- NAO model with the Fuel meshes: `mjcf/nao.xml` with the meshes as visual geoms has the same
  bodies, joints, inertias and collisions as without them (compared with the visuals removed);
  both NAO MuJoCo tests pass with it, and Gazebo renders the same `.dae` with their texture.
- NAO behind naoqi_driver (`gaitos_naoqi_sim`), MuJoCo and Gazebo, driven by a client of the
  naoqi_driver topics: `/joint_states` (26 joints), `/imu/torso`, `/joint_angles` (setAngles to
  (0.5, 0.2) reached, changeAngles −0.3), `/cmd_vel` 0.05 m/s for 8 s (0.36–0.41 m) and stop,
  `/move_base_simple/goal` 0.2 m ahead (0.18–0.195 m), TF odom → base_link → base_footprint and
  the links.
- Go2 emulated (`gaitos_unitree_sim robot:=go2`), MuJoCo and Gazebo, driven by a client of
  the SportClient protocol: StandUp, BalanceStand, Euler (pitch 0.2), Move 0.4 m/s (2.3 m in
  6 s), turning, StopMove, Sit, RiseSit, StandDown, Damp, with the sport mode of
  `/sportmodestate` following (7, 6, 1, 3, 10, 1, 5, 7) and `/lowstate` at 500 Hz.
- unitree_r1 against the emulated R1 (`gaitos_unitree_sim`, `bringup:=true`), MuJoCo and
  Gazebo: r1_state publishes joint_states and odometry from `/lowstate` (500 Hz) and
  `/odommodestate`, r1_locomotion changes FSM (stand, walk, damp) and walks with `cmd_vel`
  (0.85 m in MuJoCo, 0.90 m in Gazebo in 10 s, odometry within 3 cm of the ground truth).
- unitree_r1's `r1_control` (ros2_control with r1_arm and r1_hand) against the emulated R1 with
  `arm_sdk:=true dex3:=true`, MuJoCo: `arm_takeover` activates the arm, both hands and their
  controllers; a trajectory of the upper body (left shoulder −0.3, elbow 0.9, head yaw 0.4 rad)
  reaches −0.26 / 0.92 in `/lowstate`, one of the left hand reaches −1.0 in `/dex3/left/state`,
  and r1_locomotion ignores `cmd_vel` while the arms are held (FSM 816). On rolling its spawners
  need `--param-file` with its `controllers.yaml`.

Limitations:

- R1 walking: forward up to 0.15 m/s and turning; faster or lateral walking falls. It needs
  stiffer legs than the SDK2 example (kp 600).
- R1 balance with the arms: `balance_stand` has no feedback of the centre of mass, so an arm
  held well forward through `arm_sdk` (left shoulder pitch −0.8 rad) tips the R1 over in about
  5 s, with or without hands (reproduced with unitree_r1's r1_arm and with `/arm_sdk` directly);
  −0.3 rad holds. With the hands, `squat` also falls.
- Not tested on the real robots.
- CycloneDDS 11 (ROS rolling) vs 0.10 (unitree_sdk2): a ROS 2 subscriber on an SDK topic
  (`ros2 topic echo /lowstate` or `/lowcmd` with the unitree_ros2 messages) makes the
  CycloneDDS 0.10 side crash with a segfault (`ddsi_xt_type_init_impl with invalid type
  object`). That side is the gaitos node itself or `go2_dds_sim`, and could be the robot's
  services. Listing topics is safe, and so is gaitos sharing the domain without such
  subscribers. With the robot, use a `ROS_DOMAIN_ID` other than 0. `robot.launch.py` warns
  when it is 0.
- NAO: no NAOqi / LoLA backend for the real robot yet; walking only on flat ground, slower
  than NAOqi's walk, and `moveTo` stops a little short of the goal (≈1–2 cm).

The clips (`docs/media`) are made with the tools in `docs/tools`: `record_mujoco.sh` (the
`gaitos_mujoco snapshot` tool, offscreen EGL rendering of a scripted run, `--frames` for
sequences), `record_gazebo.sh` (a fixed camera added to the world, its images bridged to ROS and
saved) and `make_gif.py`. Each tutorial ends with the command of its clips.
