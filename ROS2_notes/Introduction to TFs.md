# ROS2 URDF & TF Visualization Notes

## 1. Introduction

* Goal: Visualize a robot and understand TF (Transforms)
* Approach: Use an existing robot instead of creating one
* Tool: URDF Tutorial Package

---

## 2. Installing URDF Tutorial Package

* Command:

```bash
sudo apt install ros-<distro>-urdf-tutorial
```

* Example (Humble):

```bash
sudo apt install ros-humble-urdf-tutorial
```

* Source ROS setup:

```bash
source /opt/ros/<distro>/setup.bash
```

* Optional: Add sourcing to `.bashrc`

---

## 3. Navigating ROS Installation

* Directory:

```bash
cd /opt/ros/<distro>/share
```

* Contains:

  * Launch files
  * URDF files
  * Config files

* Navigate to:

```bash
cd urdf_tutorial/urdf
```

---

## 4. Launching Robot Visualization

* Command:

```bash
ros2 launch urdf_tutorial display.launch.py model:=08-macroed.urdf.xacro
```

* Output:

  * RViz window opens
  * Robot model displayed
  * Joint State Publisher GUI appears

---

## 5. RViz Overview

* Purpose: 3D visualization tool in ROS

* Features:

  * Rotate: Left click
  * Zoom: Scroll / Right click
  * Move view: Middle click

* Difference from Gazebo:

  * RViz → Visualization only
  * Gazebo → Simulation (physics)

---

## 6. Robot Model (Links)

* Definition:

  * A **link = rigid part of the robot**

* Examples:

  * Base
  * Wheels
  * Gripper
  * Head

* Features:

  * Enable/disable links
  * Adjust transparency (alpha)

* Shapes:

  * Box
  * Cylinder
  * Custom meshes

---

## 7. Problem: Link Relationships

* Question:

  * How are links positioned relative to each other?

* Need:

  * A system to define position + orientation

---

## 8. TF (Transformations)

* Meaning:

  * TF = Transformation between links

* Defines:

  * Position (translation)
  * Orientation (rotation)

* Each link has:

  * A coordinate frame (origin)

---

## 9. Coordinate System Convention

* X-axis → Forward (Red)
* Y-axis → Left (Green)
* Z-axis → Up (Blue)

---

## 10. TF Visualization in RViz

* Shows:

  * Axes for each link
  * Relationship between parts

* Each axis = frame of a link

---

## 11. Joints & Motion

* TF also represents joints

* Types of motion:

  * Rotation (e.g., wheels, head)
  * Translation (e.g., gripper extension)

* Example:

  * Wheel rotates around Y-axis
  * Gripper moves along X-axis

---

## 12. Human Body Analogy

* Arm system:

  * Upper arm → link
  * Forearm → link
  * Elbow → joint (TF)

* TF defines:

  * Position + movement relationship

---

## 13. TF Tree (Hierarchy)

* Structure: Tree (parent-child)

* Example:

  * base_link → parent
  * legs → children

* Key rule:

  * Moving parent moves all children

---

## 14. Parent-Child Relationship

* Parent → defines base reference

* Child → positioned relative to parent

* Example:

  * Moving gripper base → entire gripper moves
  * Moving elbow → hand moves

---

## 15. TF Topic in ROS2

* Commands:

```bash
ros2 topic list
ros2 topic echo /tf
```

* Contains:

  * Timestamp
  * Parent frame (frame_id)
  * Child frame (child_frame_id)
  * Translation (x, y, z)
  * Rotation (quaternion)

---

## 16. TF Data Meaning

* Transformation =

  * Translation (position)
  * Rotation (orientation)

* Continuous updates via `/tf` topic

---

## 17. TF2 Tools (Tree Visualization)

* Install:

```bash
sudo apt install ros-<distro>-tf2-tools
```

* Command:

```bash
ros2 run tf2_tools view_frames
```

* Output:

  * `.gv` file
  * `.pdf` file (TF tree)

---

## 18. TF Tree PDF

* Shows:

  * All links (nodes)
  * Transformations (edges)

* Helps:

  * Understand full robot structure

---

## 19. Key Insight

* Robot model = Physical structure (links)
* TF = Spatial relationships + motion
* TF is **critical for ROS functioning**

---

## 20. Multi-Robot Systems

* Add:

  * World frame

* Structure:

  * world → robot1 base_link
  * world → robot2 base_link

* TF handles:

  * Multiple robots in same environment

---

## 21. Summary

* URDF → defines robot structure
* Links → rigid parts
* TF → relationships + movement
* RViz → visualization
* TF Tree → hierarchy of robot

---
