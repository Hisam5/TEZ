# Copilot Instructions for TM5/UR16 ROS2 Workspace

This repository is a ROS 2 workspace containing multiple packages for robot models (TM5_900, ur16) and a MoveIt configuration. The workspace is built with `colcon` and targets ROS 2 (presumably Humble or later). Copilot agents working in this repo should focus on the robotics context and the ROS 2-specific conventions.

---
## 🧠 Big Picture Architecture

- **Packages** are under `src/` and follow ROS 2 package conventions (`package.xml`, `CMakeLists.txt`).
  - `TM5_900/` and `ur16/` hold robot description assets (URDF/XACRO, meshes, textures, config, launch files).
  - `ur16_moveit_config/` holds a MoveIt setup for the UR16 manipulator including controllers, kinematics, and launch scripts.

- **URDF/Descriptions**
  - `urdf/` directories usually contain `.urdf`, `.xacro`, and corresponding `.csv` exports from CAD.
  - `config/` contains YAMLs for joint names, controllers, kinematics, and other ROS 2 control parameters.

- **Launch System**
  - Most packages use Python-based launch files (`*.launch.py`) compatible with `ros2 launch`.
  - There are two flavors: Gazebo simulation (`gazebo.launch[.py]`) and RViz/display (`display.launch` or `moveit_rviz.launch.py`).
  - The MoveIt package has additional demos (`demo.launch.py`, `move_group.launch.py`).

- **Build/Run Boundary**
  - Source the workspace: `source install/setup.bash` after `colcon build`.
  - Build with `colcon build --merge-install` from `ros2_ws/`.
  - Launch using `ros2 launch <package> <file>`.

- **Data Flow & Integration**
  - ROS topics and services are defined implicitly in launch files and SRDF/URDF; standard ROS 2 middleware.
  - Controllers and `ros2_control` configuration are in YAML and loaded at runtime via launch files (see `spawn_controllers.launch.py`).
  - MoveIt interacts with `ur16` via the ROS 2 control interface defined in these config files.

---
## ⚙️ Developer Workflows

1. **Build**
   ```bash
   cd ~/ros2_ws
   colcon build --merge-install
   source install/setup.bash
   ```
2. **Environment**
   - Each new shell must source the workspace setup script.
   - Simulation requires also sourcing ROS 2 installation (e.g. `/opt/ros/humble/setup.bash`).
3. **Launch**
   - Run Gazebo: `ros2 launch ur16 gazebo.launch.py` or `ros2 launch ur16_moveit_config gazebo.launch.py`.
   - Display (RViz): `ros2 launch ur16 display.launch.py` or `ros2 launch ur16_moveit_config moveit_rviz.launch.py`.
   - Controller spawning: `ros2 launch ur16_moveit_config spawn_controllers.launch.py`.
4. **Adding Robots/Configurations**
   - Update `urdf/*` files and `config/*` YAMLs; keep CSV exports for reference.
   - Adjust launch scripts to include new nodes/parameters.
5. **Testing/Debugging**
   - Use `ros2 topic echo`/`ros2 service list` to inspect runtime behavior.
   - Check logs via `ros2 run` with `--ros-args --log-level DEBUG`.
   - For build issues, inspect `CMakeLists.txt` for missing dependencies.

---
## 📁 Project-specific Conventions

- **File naming**
  - Launch files are named after their purpose: `gazebo.launch.py`, `display.launch`, etc.
  - YAML configs are descriptive (e.g. `joint_names_ur16.yaml`, `ur16_controllers.yaml`).
- **URDF/XACRO**
  - `.csv` files accompany CAD-derived URDF/XACRO for traceability; don't modify manually.
- **Controllers**
  - Use ROS 2 control interface YAMLs; controllers loaded via `ur16_controllers.yaml` in the MoveIt package.
- **Config directory**\n  - Central location for all parameter files; launch files reference them with `DeclareLaunchArgument` and `ParameterValue`.

---
## 🔗 Integration Points

- **External Dependencies**
  - ROS 2 core packages (`rclcpp`, `controller_manager`, `gazebo_ros`, `moveit_ros_planning_interface`).
  - Gazebo for simulation; ensure versions match ROS 2 distribution.
- **Cross-component Communication**
  - Controllers publish to `/joint_states`; MoveIt subscribes for planning.
  - The Gazebo plugin in URDF uses the `ros2_control` interface to actuate joints.

---
## 💬 Copilot Guidance

- When editing launch files, mirror existing patterns: use `get_package_share_directory` and `LaunchConfiguration` variables.
- If proposing new parameters, follow the YAML structure and update corresponding launch arguments.
- For URDF changes, maintain both `.xacro` and exported `.urdf/.csv` for consistency.
- Ensure any new package added to the workspace includes proper `package.xml` and `CMakeLists.txt` boilerplate.
- Double-check `colcon build` after changes and use `--event-handlers console_cohesion+` for readable output.

---
> 📌 **Note:** This file is a starting point. Ask the human maintainer for any missing context or if you need to update the instructions.
