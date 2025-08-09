# Unified Interface for Industrial Robots

## 1. Project Overview

This project provides a unified high-level interface for two industrial robots: Franka Emika Panda and UR5e. It supports invoking various robot actions and tasks via command line or scripts. The project is based on ROS Noetic, designed for Ubuntu 20.04 with RT kernel (5.9.1-rt20), and integrates MoveIt, franka_ros, universal_robot, and other mainstream ROS packages.

---

## 2. Environment

- **OS**: Ubuntu 20.04
- **RT Kernel**: 5.9.1-rt20
- **ROS Version**: Noetic
- **Python Version**: Recommended Python 3.8+
- **Supported Robots**:
  - Franka Emika Panda
  - UR5e

---

## 3. Dependencies

Please install the following dependencies according to the official documentation:

- **Franka**  
  - [franka_ros and libfranka](https://frankaemika.github.io/docs/)

- **UR5e**  
  - [universal_robot](https://github.com/ros-industrial/universal_robot)
  - [Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)

- **General**  
  - `moveit`

> **Note**: Some packages need to be cloned from official or third-party repositories into your catkin workspace `src` directory.

---

## 4. Robot Environment Setup

### 4.1 Franka Emika Panda

1. **Hardware Connection**: Ensure the Panda controller and PC are on the same LAN.
2. **IP Configuration**: Edit the IP address in `panda_with_impedance_controller.launch` to match your Panda controller's actual IP.
3. **Dependencies**: Install and build `franka_ros` and related dependencies.
4. **MoveIt Setup**: Ensure `panda_moveit_config` is available.

### 4.2 UR5e

1. **Hardware Connection**: Ensure the UR5e controller and PC are on the same LAN.
2. **IP Configuration**: Edit the `<arg name="robot_ip" ...>` in `ur5e_control/launch/ur5e_control.launch` to match your UR5e's actual IP.
3. **Dependencies**: Install and build `universal_robot`, `ur_robot_driver`, `ur5e_moveit_config`, etc.
4. **MoveIt Setup**: Ensure `ur5e_moveit_config` is available.

---

## 5. Build and Environment Variables

1. **Build the Workspace**  
   In the root of your catkin workspace, run:
   ```bash
   catkin_make
   ```

2. **Source Environment**  
   Every new terminal, run:
   ```bash
   source /path/to/devel/setup.bash
   ```
   It is recommended to add this to your `~/.bashrc` for convenience.

---

## 6. How to Run

### Method 1: Use the Unified Interface Directly

1. **Go to the interface scripts directory**  
   ```bash
   cd /path/to/src/interface/scripts
   ```

2. **Run the interface**  
   - For Franka (e.g., shaft1_assembling or gear_assembling):
     ```bash
     # Default is gear_assembling task
     python3 robot_interface.py franka execute_plan 
     # Or select task like following
     python3 robot_interface.py franka execute_plan shaft1
     python3 robot_interface.py franka execute_plan gear_assembling
     ```
   - For UR5e (e.g., desk_cleanup or gear_assembling):
     ```bash
     python3 robot_interface.py ur5e desk_cleanup
     python3 robot_interface.py ur5e gear_assembling
     ```
   > The interface will automatically launch the corresponding launch file and wait for the robot nodes to be ready.

---

### Method 2: Manually Launch and Run Control Scripts

1. **Manually launch the robot bringup**  
   - Franka:
     ```bash
     roslaunch panda_base panda_with_impedance_controller.launch
     ```
   - UR5e:
     ```bash
     roslaunch ur5e_control ur5e_control.launch robot_ip:=<your_ur5e_ip>
     ```

2. **Wait for all nodes to be ready (recommended 10-15 seconds)**

3. **Run the control script**  
   - Franka:
     ```bash
     # Default is gear_assembling task
     rosrun panda_base panda_control.py
     # Or select task like following
     rosrun panda_base panda_control.py mode:="shaft1"
     rosrun panda_base panda_control.py mode:="gear_assembling"
     ```
   - UR5e:
     ```bash
     rosrun ur5e_control ur5e_control.py gear_assembling
     rosrun ur5e_control ur5e_control.py desk_cleanup
     ```

---

## 7. Notes

- **IP Address**: Make sure to set the `robot_ip` parameter in the launch file to your actual robot IP.
- **Permissions**: Some scripts may require execution permission. Use `chmod +x script.py` if needed.
- **Dependencies**: If you encounter missing packages, ensure you have sourced the workspace and built it.
- **RT Kernel**: For best real-time performance, use the RT kernel.

