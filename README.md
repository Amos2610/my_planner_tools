# my_planner_tools

This package provides basic motion planning tools for xArm6 using joint-space targets.  
It's designed for testing or integrating with xarm_planner and MoveIt in ROS 2.

---

## 📦 Features

- Plan and execute joint-space trajectories using `xarm_joint_plan` and `xarm_exec_plan`
- Parameter input via command-line (`--ros-args -p target_joint:=[...]`)
- Intended for use with a fake or simulated xArm6 setup (e.g., via MoveIt)

---

## 🛠 Dependencies

- ROS 2 (Humble or later)
- [xarm_ros2](https://github.com/xArm-Developer/xarm_ros2)
- `xarm_msgs`, `rclcpp`

---

## 🚀 Usage

### 1. Launch the fake xArm6 MoveIt environment

```bash
ros2 launch xarm_planner xarm6_planner_fake.launch.py add_gripper:=true
```
### 2. Run the joint test

```bash
ros2 run my_planner_tools xarm6_planner_joint --ros-args -p target_joint:="[1.0, 0.8, 0.2, -1.0, 0.3, 0.0]"
```