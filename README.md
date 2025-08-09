# Unified Interface for Industrial Robots

This project provides a unified high-level interface for two industrial robots: **Franka Emika Panda** and **UR5e**. It supports invoking robot **skills** and executing **task plans** from a single command-line interface.  
The stack is built on **ROS Noetic** (Ubuntu 20.04, optional RT kernel) and integrates **MoveIt**, **franka_ros/libfranka**, **universal_robot**, and **Universal Robots ROS Driver**.

---

## 1) Requirements

- **OS**: Ubuntu 20.04  
- **ROS**: Noetic (desktop-full recommended)  
- **Python**: 3.8+  
- **Hardware**: Franka Panda, UR5e (e-Series)  
- **(Optional)**: RT kernel for better realtime performance

> ⚠️ Safety first. Keep clear of the workspace, limit speeds for initial tests, and follow vendor safety procedures.

---

## 2) Dependencies & Robot Environments


> If you already have **ROS Noetic + MoveIt** installed, you can skip the `apt install` part.
> If `rosdep` is already initialized on your machine, you can skip `sudo rosdep init`.

**Install dependencies**
```bash
sudo apt update
sudo apt install -y ros-noetic-desktop-full \
  ros-noetic-moveit ros-noetic-ros-control ros-noetic-ros-controllers \
  python3-rosdep python3-catkin-tools

# Initialize rosdep (first time only; safe to skip if already done)
sudo rosdep init || true
rosdep update
```

**Robot Environments**
  
> - **Franka**: You **must install `libfranka` at the system level first**, then build `franka_ros` in your catkin workspace.  
> - **UR5e**: On the **PC side**, the ROS driver packages are usually enough (no extra system libraries). On the **robot side**, install & enable the **External Control** URCap and provide an External Control program (this project defaults to `/programs/xinlong.urp`).

### Franka (system prerequisite + ROS packages)
- **System library: `libfranka` (required before building `franka_ros`)**  
  Documentation & installation guidance:  
  → Franka Robotics Docs: https://frankarobotics.github.io/docs/index.html
- **ROS packages (vendored in this repo): `franka_ros`, `panda_moveit_config`**  
  Build them in your catkin workspace after `libfranka` is installed:  
  → Franka Robotics Docs: https://frankarobotics.github.io/docs/index.html

### UR5e (ROS driver + robot-side URCap)
- **ROS packages (vendored in this repo): `Universal_Robots_ROS_Driver`**  
  PC-side driver; build in your catkin workspace:  
  → Universal Robots ROS Driver: https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
- **ROS packages (vendored in this repo): `universal_robot` (MoveIt configs, etc.)**  
  Build in your catkin workspace:  
  → universal_robot: https://github.com/ros-industrial/universal_robot
- **Robot-side requirement: External Control URCap (teach pendant)**  
  Download and **install** the *External Control URCap* on the robot (System → URCaps), then **enable** it. If urcap is already installed on your robot, please ignore this step.
  Create an **External Control** program on the pendant (Host IP = your ROS PC) and **save it as** `/programs/xinlong.urp`.  
  *You do not need to load/run it at this step; the launch script will load it automatically at runtime.*  
  - URCap download: https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases  
  - Driver setup guide (program creation, network settings): https://github.com/UniversalRobots/Universal_Robots_ROS_Driver



> If you save the program under a **different path/name**, update `src/ur5e/ur5e_control/scripts/load_externalcontrol.sh` accordingly.


> Optional: If you prefer **upstream repos** instead of the vendored copies in this repo, remove/rename the vendored folders and clone the upstream sources using the links above. Then run `rosdep install --from-paths src --ignore-src -r -y` and `catkin_make` as usual.


## 3) Install & Build

**Create a catkin workspace and clone this repo**
```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone https://github.com/wxl97/Skill-transfer-between-Franka-and-UR5e.git
cd ..
```

**Install package dependencies for this workspace (after clone)**
```bash
# Run from the workspace root: ~/robot_ws
rosdep install --from-paths src --ignore-src -r -y
```

**Build & source**
```bash
catkin_make
echo 'source ~/robot_ws/devel/setup.bash' >> ~/.bashrc
source ~/robot_ws/devel/setup.bash
```

**Make scripts executable (first time only)**
```bash
chmod +x ~/robot_ws/src/Skill-transfer-between-Franka-and-UR5e/src/interface/scripts/robot_interface.py
chmod +x ~/robot_ws/src/Skill-transfer-between-Franka-and-UR5e/src/franka/panda_base/scripts/panda_control.py
chmod +x ~/robot_ws/src/Skill-transfer-between-Franka-and-UR5e/src/ur5e/ur5e_control/scripts/ur5e_control.py
...
```



## 4) Hardware Setup

### 4.1 Franka Emika Panda
- Connect the Panda controller and the PC to the same LAN.
- Edit `src/franka/panda_base/launch/panda_with_impedance_controller.launch`:
  ```xml
  <arg name="robot_ip" default="192.168.x.x"/>
  ```
- Ensure `franka_ros` builds successfully. Follow Franka safety procedures (enable robot, release E-stop).

### 4.2 UR5e (e-Series) with External Control URCap
1. On the teach pendant, **install & enable** the **External Control** URCap.  
2. Create a program containing a single **External Control** node:
   - **Host IP** = your ROS PC IP  
   - Keep the default port unless changed in driver settings
3. Save the program on the controller as: **`/programs/xinlong.urp`**  
   > The launch/scripts auto-load this exact path via dashboard services. If you change it, also update `src/ur5e/ur5e_control/scripts/load_externalcontrol.sh`.
4. Switch to **Remote Control** and clear any protective/estop faults.

---

## 5) Quick Start (Unified Interface)

The interface will auto-launch the proper bringup (drivers + MoveIt), wait for nodes to initialize, then execute the selected task plan.

**Available tasks** (from `src/interface/scripts/planlist.json`):  
`shaft1`, `shaft2`, `shaft3`, `gear1`, `gear2`, `gear3`, `gear_assembling`

**Franka – execute a predefined plan**
```bash
rosrun interface robot_interface.py franka execute_plan gear_assembling
# Other examples:
rosrun interface robot_interface.py franka execute_plan shaft1
rosrun interface robot_interface.py franka execute_plan shaft2
```

**UR5e – execute a predefined plan**
```bash
rosrun interface robot_interface.py ur5e execute_plan gear_assembling
# Other examples:
rosrun interface robot_interface.py ur5e execute_plan shaft1
rosrun interface robot_interface.py ur5e execute_plan shaft3
```

> For UR5e, the interface will also load & play `/programs/xinlong.urp` via dashboard services automatically.

---

## 6) Manually launch the robot bringup

**Franka**
```bash
roslaunch panda_base panda_with_impedance_controller.launch robot_ip:=192.168.x.x
# (optional) In another terminal:
rosrun panda_base panda_control.py
```

**UR5e**
```bash
roslaunch ur5e_control ur5e_control.launch robot_ip:=172.16.x.x
# The launch calls dashboard services to load & play /programs/xinlong.urp automatically.
# Wait until all nodes start successfully. If any node fails, press Ctrl+C and relaunch.
# After loading launch file, run in another terminal:
rosrun ur5e_control ur5e_control.py gear_assembling
```

---

## 7) Configuration & Knowledge Base

- **Interface task list**: `src/interface/scripts/planlist.json`  
  Defines high-level tasks and their required skill sequences.
- **Global knowledge base**: `src/interface/scripts/global_knowledge_base.json`  
  Used by the interface to check capability compatibility.




## 8) Notes for Maintainers

- UR5e launch loads `/programs/xinlong.urp` through `ur5e_control/scripts/load_externalcontrol.sh`.  
  If you rename or relocate the URP program, update that script accordingly (or parameterize it).
- Keep code, comments, and logs in **English** for easier collaboration.

---

## 9) Acknowledgements

- Franka Emika: `franka_ros`, `libfranka`  
- Universal Robots: `ur_robot_driver`, `universal_robot`  
- MoveIt, ROS Control, and the broader ROS community
