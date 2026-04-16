## 1. What is XML?

### Definition

**XML (Extensible Markup Language)** is a structured, text-based language used to represent hierarchical data in a **machine-readable and human-readable format**.

It is not a programming language—it is a **data description format**.

---

### Core Characteristics

* **Hierarchical (Tree Structure)**
  Data is organized as nested elements (parent-child relationships)

* **Self-descriptive**
  Tags define meaning (you create your own tags)

* **Platform-independent**
  Can be used across systems, languages, and tools

* **Strict syntax rules**

  * Must have a single root element
  * Tags must be properly closed
  * Case-sensitive

---

### Basic XML Structure

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <link name="base_link"/>
</robot>
```

---

### Components of XML

| Component | Description                |
| --------- | -------------------------- |
| Tag       | `<link>`                   |
| Attribute | `name="base_link"`         |
| Element   | `<link name="base_link"/>` |
| Root      | `<robot>`                  |
| Nesting   | Elements inside elements   |

---

### Tree Representation

```
robot
 └── link (base_link)
```

---

### Why XML is Used in Robotics

* Easy to parse (libraries available in all languages)
* Represents hierarchical systems naturally
* Ideal for describing **robot structures (links & joints)**

---

## 2. What is URDF?

### Definition

**URDF (Unified Robot Description Format)** is an XML-based format used in ROS to describe the **physical structure, geometry, and kinematics of a robot**.

It defines:

* Rigid bodies (**links**)
* Connections (**joints**)
* Spatial relationships (**TF frames**)

---

### Conceptual Model

A robot in URDF is modeled as:

```
Robot = Links + Joints + Transformations
```

* **Links** → physical parts (chassis, arm, wheel)
* **Joints** → relationships between parts
* **Transforms (TF)** → position/orientation in 3D space

---

### Minimal URDF Example

```xml
<robot name="my_robot">
  <link name="base_link"/>
</robot>
```

---

## 3. Core Components of URDF

### 3.1 Link

A **link** represents a rigid body.

```xml
<link name="base_link">
  <visual>
    <geometry>
      <box size="0.6 0.4 0.2"/>
    </geometry>
  </visual>
</link>
```

#### Properties of a Link

* Geometry (box, cylinder, sphere, mesh)
* Visual appearance
* Inertial properties (mass, inertia)
* Collision model

---

### 3.2 Joint

A **joint** defines how two links are connected.

```xml
<joint name="base_to_link" type="fixed">
  <parent link="base_link"/>
  <child link="second_link"/>
  <origin xyz="0 0 0.2" rpy="0 0 0"/>
</joint>
```

---

### Joint Types

| Type       | Description         |
| ---------- | ------------------- |
| fixed      | No movement         |
| revolute   | Rotates with limits |
| continuous | Infinite rotation   |
| prismatic  | Linear motion       |

---

### 3.3 Origin

Defines transformation between frames.

```xml
<origin xyz="x y z" rpy="roll pitch yaw"/>
```

* **xyz** → translation (meters)
* **rpy** → rotation (radians)

---

### 3.4 Material

Defines color of visual elements.

```xml
<material name="blue">
  <color rgba="0 0 0.5 1"/>
</material>
```

---

## 4. Transform (TF) Concept

URDF builds a **kinematic tree**

Example:

```
base_link
   └── second_link
          └── third_link
```

Each joint defines a transformation:

```
T_parent→child = Translation + Rotation
```

---

## 5. Critical Concept: Joint vs Visual Origin

This is the most important idea in URDF.

### Joint Origin

* Defines **actual spatial position (TF)**
* Affects robot kinematics

### Visual Origin

* Only affects **appearance**
* Does NOT affect TF

---

### Example

#### Joint (position in space)

```xml
<origin xyz="0 0 0.2" rpy="0 0 0"/>
```

#### Visual (appearance offset)

```xml
<origin xyz="0 0 0.05" rpy="0 0 0"/>
```

---

## 6. Correct Workflow for Building URDF

### Step-by-step Process

1. Create link
2. Create joint
3. Set all origins to zero
4. Fix **joint origin first (TF alignment)**
5. Verify in RViz (TF frames)
6. Adjust **visual origin after TF is correct**

---

### Why This Order Matters

If you modify visual first:

* You will get correct appearance but wrong TF
* Robot behavior will break (especially in motion planning)

---

## 7. Example: Stacking a Box on a Cylinder

### Step 1: Fix Joint

Cylinder height = 0.2

```xml
<origin xyz="0 0 0.2" rpy="0 0 0"/>
```

---

### Step 2: Fix Visual

Box height = 0.1 → offset = 0.05

```xml
<origin xyz="0 0 0.05" rpy="0 0 0"/>
```

---

## 8. Why Geometry Appears Centered

URDF defines geometry around its **center of mass**.

So:

* Box is centered at origin
* Cylinder is centered at origin

To place it on surfaces → you must offset manually

---

## 9. Common Errors

* Multiple root links (missing joint) 
* Editing visual instead of joint
* Incorrect axis in joints
* Mixing degrees and radians
* Not following parent-child hierarchy

---

## 10. Key Mental Model

Think in two layers:

### Layer 1: Kinematics (Reality)

* Defined by joints
* Controls TF tree

### Layer 2: Visualization

* Defined by visual tags
* Only affects rendering

---

## 11. Summary

* XML = structured data format
* URDF = XML-based robot description format
* Robot = links + joints + transforms
* Always fix **joint (TF)** before **visual**
* Geometry is centered → requires offsets
* URDF builds a **tree, not a graph**

---

