# Quick start, in detail

[← gaitos](../README.md#documentation)

What the commands of the quick start do, and the other ways to launch gaitos: Gazebo, the real Unitree robots and their DDS simulator, the vendor interfaces in simulation and the display without physics. Step by step for each robot: [the tutorials](tutorials/README.md).

## The quick start, step by step

1. **Workspace.** gaitos goes in `src/` of a colcon workspace; every command runs from the
   workspace root.
2. **Dependencies.** MuJoCo 3 and EGL come from Ubuntu (no rosdep key); rosdep installs the
   rest from the `package.xml` files (Gazebo and OSQP through their ROS vendor packages).
3. **Build.** Release, because the controllers run at 500 Hz in lockstep with the physics. The
   first build downloads unitree_sdk2, the unitree_ros2 messages, naoqi_bridge_msgs and the
   Unitree models (FetchContent, pinned commits); offline builds and options in [Build](build.md).
4. **Launch.** The Go2 in MuJoCo, in the namespace `/go2`. It starts lying in `damp` (the safe
   mode). The physics runs in the node, which publishes `/clock`.
5. **Commands, from another terminal.** `stand_up` goes to the `stand` pose of the SRDF;
   `balance_stand` keeps the feet and moves the body; a `cmd_vel` switches it to
   `locomotion` (trot), and it stops 0.5 s after the last twist; `stand_down` lies it down.
   The modes and their rules are in [Operation modes](modes.md), the topics and services in
   [ROS 2 interfaces](interfaces.md).

`robot:=r1` and `robot:=nao` work the same way (the R1 starts standing, the NAO in `crouch`:
`mode/stand_init` first); `gaitos_gazebo` instead of `gaitos_mujoco` runs Gazebo.

## MuJoCo

```bash
ros2 launch gaitos_mujoco sim.launch.py robot:=go2 [rviz:=true] [real_time_factor:=1.0]
```

The robot starts lying in `damp`. From another terminal:

```bash
ros2 service call /go2/mode/stand_up std_srvs/srv/Trigger
ros2 service call /go2/set_mode gaitos_msgs/srv/SetMode "{mode: balance_stand}"
ros2 topic pub --once /go2/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base}, pose: {position: {z: -0.05}, orientation: {y: 0.07, w: 1.0}}}"
ros2 topic pub -r 10 /go2/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4}}"   # walks
ros2 topic pub --once /go2/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}"
ros2 service call /go2/mode/locomotion_mpc std_srvs/srv/Trigger                   # convex MPC
ros2 service call /go2/mode/stand_down std_srvs/srv/Trigger
ros2 topic echo /go2/status
```

`robot:=r1` starts the R1 standing; a `cmd_vel` in `balance_stand` starts the ZMP / LIPM
walking (`locomotion_lipm`). `robot:=r1 model:=dex3` is the R1 with its Dex3-1 hands and stock
colours ([R1 in MuJoCo](tutorials/r1_mujoco.md#6-the-r1-with-dex3-1-hands)).

## Gazebo

```bash
ros2 launch gaitos_gazebo sim.launch.py robot:=go2 [headless:=true] [rviz:=true]
ros2 launch gaitos_gazebo sim.launch.py robot:=r1      # world empty_fine (0.5 ms step)
```

The launch starts gz-sim through `ros_gz_sim`, spawns the robot with the `GaitosGzSystem`
plugin (which creates the gaitos node inside the simulator, stepped in lockstep, with the
motors as PD at every physics step), bridges `/clock` with `ros_gz_bridge` and runs
`robot_state_publisher`. The ROS interfaces are the same as with MuJoCo.

The simulated model is `urdf/<robot>_gazebo.urdf`, made by `make_gazebo_urdf.py` from the
URDF and `config/gazebo.yaml` with what the URDF lacks of the unitree_mujoco model: the
inertia (armature) and losses of the motors, both robots, and for the R1 the contact
geometry (feet boxes and spheres instead of meshes). Without the armature the torque control
of the convex MPC is unstable in Gazebo.

## Unitree robots

```bash
ros2 launch gaitos_unitree robot.launch.py robot:=go2 readonly:=true   # read only first
ros2 launch gaitos_unitree robot.launch.py robot:=go2 [network_interface:=eth0] [joy:=true]
ros2 launch gaitos_unitree robot.launch.py robot:=r1
```

The backends release the built-in sport / loco service (`release_sport_mode`), read
`rt/lowstate` and write `rt/lowcmd` (with its CRC) through unitree_sdk2; the motor drivers
close the PD of each command. `dds_sim:=true` uses interface `lo` and domain 1, the
setting of the [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco) simulator
and of `go2_dds_sim`, a headless Go2 that speaks the same DDS topics:

```bash
ros2 run gaitos_unitree go2_dds_sim \
  $(ros2 pkg prefix gaitos_description)/share/gaitos_description/robots/go2_description/mjcf/scene.xml
ros2 launch gaitos_unitree robot.launch.py robot:=go2 dds_sim:=true
```

gaitos talks to the robot through unitree_sdk2 inside its node, so it works with any RMW.
It is tested with both FastDDS and `rmw_cyclonedds_cpp`. The node can load the CycloneDDS 0.10
bundled with unitree_sdk2 next to the CycloneDDS 11 used by the ROS RMW. With Cyclone, raise
the socket buffer as the RMW suggests: `sudo sysctl -w net.core.rmem_max=8388608`.

The unitree_ros2 messages (`unitree_go`, `unitree_hg`, `unitree_api`) are packages in
`unitree_ros2_msgs/`. They keep the names of unitree_ros2, because the package name gives the
DDS type names. To see the topics of the DDS simulator (domain 1 on
`lo`, without multicast), only list them, and read the limitation in [Status](status.md) before
subscribing:

```bash
ROS_DOMAIN_ID=1 CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><Peers><Peer address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>' \
  ros2 topic list --no-daemon   # /lowcmd, /lowstate
```

## Vendor interfaces in simulation

The Unitree robots behind their DDS interface and the NAO behind naoqi_driver, in MuJoCo or Gazebo: see [Vendor interfaces in simulation](emulators.md).

## Display

```bash
ros2 launch gaitos_description display.launch.py robot:=r1 pose:=squat
```

gaitos on `FakeHardware` (no physics) in an SRDF pose, with `robot_state_publisher` and RViz;
the posture modes move it.
