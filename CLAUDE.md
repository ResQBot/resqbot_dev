# Lotti3 — Claude Code Context

## Project Overview
Lotti3 is a tracked rescue robot with a 5-DOF arm and 4 flippers. The ROS2 workspace is at `control_ws/` and targets **ROS2 Humble + Gazebo Classic 11** (Ubuntu 22.04 on the robot). All real-hardware launch files are in `control_ws/src/lotti_control3/bringup/launch/`.

---

## Workspace Structure

```
control_ws/src/
├── lotti_control3/          # Main package: URDF, hardware interfaces, launch, config
│   ├── bringup/
│   │   ├── config/          # lotti_controllers.yaml, lotti_servo_config.yaml
│   │   └── launch/          # full.launch.py, robot.launch.py, gazebo.launch.py
│   ├── description/
│   │   ├── meshes/          # STL files for arm links
│   │   ├── ros2_control/    # Lotti.ros2_control.xacro (real), Lotti_sim.ros2_control.xacro (sim)
│   │   ├── urdf/            # Lotti.urdf.xacro, Lotti_body.xacro, Lotti_arm3.xacro, Lotti_gazebo_control.xacro
│   │   └── worlds/          # lotti_world.world (Gazebo SDF world)
│   └── hardware/            # C++ serial hardware interfaces (arm3, flipper3, drive3)
├── lotti3_moveit_config/    # MoveIt2 configuration (auto-generated)
├── lotti_drive_controller/  # Custom DriveController plugin
├── lotti_flipper3_controller/ # Custom FlipperController plugin
├── lotti_teleop/            # Python teleop node (joystick → ROS topics)
└── lotti_lidar_tf/          # LiDAR TF broadcaster
```

---

## Robot Description

**Links (body):** `body_link`, `left_main_wheel_link`, `right_main_wheel_link`, `flipper_fr_link`, `flipper_fl_link`, `flipper_rr_link`, `flipper_rl_link`

**Links (arm):** `arm_base_link`, `arm_link1_link` … `arm_link5_link`

**Joints:**
- `left_chain_joint`, `right_chain_joint` — `continuous`, velocity-controlled (drive)
- `flipper_fr/fl/rr/rl_joint` — `continuous`, velocity-controlled
- `arm_link1_joint` … `arm_link5_joint` — `revolute`, position-controlled

---

## ros2_control Hardware Systems

| System | Real plugin | Sim plugin | Joints |
|--------|-------------|------------|--------|
| LottiArm | `arm3_interface/ArmInterface` | `gazebo_ros2_control/GazeboSystem` | arm_link1..5_joint (position) |
| LottiFlipper | `flipper3_interface/FlipperInterface` | `gazebo_ros2_control/GazeboSystem` | flipper_fr/fl/rr/rl_joint (velocity) |
| LottiChains | `drive3_interface/DriveInterface` | `gazebo_ros2_control/GazeboSystem` | left/right_chain_joint (velocity) |

The switch is controlled by a xacro arg in `Lotti.urdf.xacro`:
- `use_sim:=false` (default) → loads `Lotti.ros2_control.xacro` (serial)
- `use_sim:=true` → loads `Lotti_sim.ros2_control.xacro` + `Lotti_gazebo_control.xacro`

---

## Launch Files

| File | Purpose |
|------|---------|
| `robot.launch.py` | Real hardware, no servo/teleop |
| `full.launch.py` | Real hardware, full stack (servo + teleop) |
| `operator.launch.py` | Remote operator station (RViz + servo only) |
| `gazebo.launch.py` | **Simulation** — Gazebo + full stack |
| `description/launch/view_lotti.launch.py` | URDF visualization only (RViz + joint_state_publisher_gui) |

---

## Simulation Setup (Gazebo Classic 11)

### Launch simulation
```bash
cd control_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch lotti_control3 gazebo.launch.py
```

### Verify controllers are running
```bash
ros2 control list_controllers
```
Expected: `joint_state_broadcaster`, `arm_controller`, `drive_controller`, `flipper3_controller` all **active**.

### Test drive
```bash
ros2 topic pub /cmd/drive geometry_msgs/msg/Twist "linear: {x: 0.3}" --rate 10
```

### Test flippers
```bash
ros2 topic pub /cmd/flipper_fr std_msgs/msg/Int8 "data: 1" --rate 10
```

### Validate URDF (without Gazebo)
```bash
xacro src/lotti_control3/description/urdf/Lotti.urdf.xacro use_sim:=true > /tmp/lotti_sim.urdf
check_urdf /tmp/lotti_sim.urdf
```

---

## Key Design Decisions

- **`use_sim` xacro arg** controls which hardware plugin is loaded. Real launches never pass this arg so they always use `default="false"` → serial plugins.
- **Chain joints are `continuous`** (changed from `fixed`). This is correct for both sim and real — the real hardware interface manages velocity commands regardless of the URDF joint type.
- **`lotti_servo_config.yaml` stays unchanged** (`use_gazebo: false`). The `gazebo.launch.py` overrides this in Python at launch time.
- **`package.xml` for `lotti_control3`** was missing from the repo; it has been created. Add `gazebo_ros2_control` and `gazebo_ros` as `exec_depend`.

---

## Jazzy + Gazebo Harmonic migration (future)
If upgrading to ROS2 Jazzy + Gazebo Harmonic:
- Plugin: `gz_ros2_control/GazeboSimSystem`
- Gazebo plugin tag: `<plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">`
- Launch package: `ros_gz_sim` instead of `gazebo_ros`
- Spawn: `ros_gz_sim create` action
- Package dep: `gz_ros2_control`
