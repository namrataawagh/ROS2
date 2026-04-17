# ROS 2 URDF, TF, and Launch System – Notes

---

# 1. Core Concept Overview

In ROS 2, visualizing and controlling a robot requires three main components:

### 1. URDF (Robot Description)

* Defines robot structure:

  * Links (rigid bodies)
  * Joints (connections)
  * Geometry and hierarchy
* Static → does **not represent motion**

### 2. Joint States

* Represents **current position of joints**
* Example:

  * Wheel rotation = 1.52 radians
* Comes from:

  * Real robot → encoders
  * Simulation → physics engine
  * Tutorial → GUI (fake publisher)

### 3. Robot State Publisher

* Core node that computes transforms
* Inputs:

  * URDF (structure)
  * Joint states (motion)
* Output:

  * TF (transform tree)

---

# 2. Data Flow in ROS 2

```
URDF (robot structure)
        ↓
robot_state_publisher
        ↑
joint_states (joint angles)
        ↓
TF (transforms)
        ↓
RViz / Navigation / MoveIt
```

---

# 3. TF (Transform System)

* TF defines **relationship between coordinate frames**
* Example tree:

```
base_footprint
   └── base_link
         ├── left_wheel
         ├── right_wheel
         └── caster_wheel
```

* Published on:

```
/tf
```

* Used by:

  * Navigation stack
  * MoveIt
  * SLAM
  * Custom applications

---

# 4. Important ROS Topics

### List topics

```
ros2 topic list
```

### Key Topics

* `/tf` → transformations
* `/joint_states` → joint angles
* `/robot_description` → URDF content

---

### Inspect joint states

```
ros2 topic echo /joint_states
```

### Inspect URDF

```
ros2 topic echo /robot_description
```

---

# 5. Running Nodes Manually

## 5.1 Robot State Publisher

```
ros2 run robot_state_publisher robot_state_publisher \
--ros-args -p robot_description:="$(xacro my_robot.urdf)"
```

* Parameter:

  * `robot_description` = URDF content

---

## 5.2 Joint State Publisher (GUI)

```
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

* Provides sliders to simulate joint movement

---

## 5.3 RViz (Visualization)

```
ros2 run rviz2 rviz2
```

### Setup in RViz:

* Add:

  * RobotModel
  * TF
* Set:

  * Fixed Frame → `base_footprint`
  * Description Topic → `/robot_description`

---

# 6. ROS Graph Understanding

* Nodes:

  * robot_state_publisher
  * joint_state_publisher

* Topics:

  * `/tf`
  * `/joint_states`

* robot_state_publisher:

  * Subscribes → `/joint_states`
  * Publishes → `/tf`

---

# 7. Creating ROS 2 Workspace

```
mkdir ros2_ws
cd ros2_ws
mkdir src
colcon build
```

### Source workspace:

```
source install/setup.bash
```

(Optional: add to `.bashrc`)

---

# 8. Creating Description Package

```
ros2 pkg create my_robot_description
```

### Recommended naming convention:

```
<robot_name>_description
```

Example:

* `turtlebot3_description`

---

## Package Structure

```
my_robot_description/
├── urdf/
│   └── my_robot.urdf
├── launch/
├── rviz/
├── CMakeLists.txt
├── package.xml
```

---

# 9. Installing URDF in CMakeLists.txt

```
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)
```

After build:

```
install/my_robot_description/share/my_robot_description/urdf/
```

---

# 10. Launch File (XML)

## 10.1 Why XML?

* Simpler than Python for most use cases
* Easier syntax for basic setups
* Python only needed for complex logic

---

## 10.2 Basic Launch File

```
<launch>

  <let name="urdf_path"
       value="$(find-pkg-share my_robot_description)/urdf/my_robot.urdf"/>

  <node pkg="robot_state_publisher"
        exec="robot_state_publisher">
    <param name="robot_description"
           value="$(command 'xacro $(var urdf_path)')" />
  </node>

  <node pkg="joint_state_publisher_gui"
        exec="joint_state_publisher_gui" />

  <node pkg="rviz2"
        exec="rviz2"
        output="screen" />

</launch>
```

---

## Key Concepts

### Variable creation

```
<let name="var_name" value="..." />
```

### Using variable

```
$(var var_name)
```

### Finding package path

```
$(find-pkg-share package_name)
```

---

# 11. Adding RViz Config

## Steps:

1. Configure RViz manually
2. Save config:

```
File → Save Config As → urdf_config.rviz
```

3. Add to package:

```
rviz/urdf_config.rviz
```

---

## Install RViz folder

```
install(
  DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)
```

---

## Update Launch File

### Add variable:

```
<let name="rviz_config"
     value="$(find-pkg-share my_robot_description)/rviz/urdf_config.rviz"/>
```

### Use in RViz node:

```
<node pkg="rviz2"
      exec="rviz2"
      args="-d $(var rviz_config)"
      output="screen"/>
```

---

# 12. Build and Run

```
colcon build
source install/setup.bash
ros2 launch my_robot_description display.launch.xml
```

---

# 13. Key Takeaways

* URDF = robot structure
* Joint states = motion data
* robot_state_publisher = computes transforms
* TF = final representation used everywhere

---

# 14. Conceptual Analogy

* URDF → Skeleton
* Joint States → Muscle movement
* robot_state_publisher → Brain
* TF → Actual pose of robot

---

# 15. Final Insight

* TF is the **foundation of all robotics applications in ROS**
* Without TF:

  * No navigation
  * No manipulation
  * No spatial awareness

---


