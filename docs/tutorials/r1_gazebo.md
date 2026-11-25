# R1 in Gazebo

[← Tutorials](README.md)

The Unitree R1 in Gazebo (gz-sim), in the 0.5 ms world. The simulated model is
`urdf/r1_gazebo.urdf` (`make_gazebo_urdf.py`): the contact geometry of `unitree_mujoco` (feet boxes
and spheres instead of meshes) and the motor inertia. The R1 with Dex3-1 hands and stock colours (`model:=dex3`) is only
in MuJoCo for now ([R1 in MuJoCo](r1_mujoco.md#6-the-r1-with-dex3-1-hands)).

| Walking forward (`locomotion_lipm`) |
|:---:|
| ![R1 walking forward in Gazebo](../media/r1_gazebo_walk.gif) |
| balance_stand → 0.15 m/s for 8 s → stop (the camera follows the robot) |

## 1. Before starting

```bash
cd ~/ws/legged_ws
source install/setup.bash
export ROS_DOMAIN_ID=10
```

## 2. Launch

```bash
ros2 launch gaitos_gazebo sim.launch.py robot:=r1 [headless:=true] [rviz:=true]
```

The world is `empty_fine` (0.5 ms physics step) by default for the R1; the arguments are those
of [Go2 in Gazebo](go2_gazebo.md#2-launch). The R1 is spawned standing at z = 0.72 m, in
`stand_up`.

## 3. Modes, posture and walking

```bash
ros2 service call /r1/mode/balance_stand std_srvs/srv/Trigger
ros2 topic pub --once /r1/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: pelvis_link}, pose: {position: {z: -0.05}, orientation: {w: 1.0}}}"
ros2 topic pub -r 10 /r1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}}"
ros2 service call /r1/mode/squat std_srvs/srv/Trigger
ros2 topic echo /r1/status
```

The modes are those of [R1 in MuJoCo](r1_mujoco.md#3-modes).

## 4. unitree_r1 against this simulation

```bash
ros2 launch gaitos_unitree_sim sim.launch.py robot:=r1 simulator:=gazebo headless:=true bringup:=true
ros2 service call /r1/r1_locomotion/stand std_srvs/srv/Trigger
ros2 service call /r1/r1_locomotion/walk std_srvs/srv/Trigger
ros2 topic pub -r 10 /r1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
```

See [R1 in MuJoCo](r1_mujoco.md#5-unitree_r1-against-this-simulation).

## 5. Known issues

- Walking forward works (≈1 m in the launch runs); long walks fall after 15–20 s and turning
  in place falls in Gazebo. In MuJoCo both work.
- `squat` → `stand_up` falls (as in MuJoCo).

## 6. Remake the clip

```bash
GIF_OPTS="--skip 14" CAM="0.6 -3.5 1.0 0 0.1 1.5708" docs/tools/record_gazebo.sh r1 empty_fine \
  docs/media/r1_gazebo_walk.gif wait:20 mode:balance_stand wait:1 vel:0.15,0,0 wait:8 stop wait:4
```
