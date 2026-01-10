# 🟦 Topic 2 — Write Your First ROS2 Program (Create a Python Package)

This topic covers how to set up a **ROS2 workspace**, configure it correctly, and create your **first Python package**.  
Follow these steps whenever you start a new ROS2 project.

---

##  1. Create a ROS2 Workspace

A **ROS2 workspace** is a directory that contains your source code, installed files, and build outputs.

````markdown
### **Steps**
```bash
cd ~
mkdir ros2_ws        # or <appname>_ws
cd ros2_ws
mkdir src
````

### **Workspace Structure**

```
ros2_ws/
 ├── src/           # All packages go here
```

This is the minimum required structure to initialize a ROS2 workspace.

---

## 2. Build the Empty Workspace

Always build your workspace **from the root of the workspace**, NOT from the `src/` folder.

```bash
cd ~/ros2_ws
colcon build
```

### After build, three new folders appear:

```
build/
install/
log/
src/
```

If the `colcon` command doesn’t work, install ROS dev tools:

```bash
sudo apt install ros-dev-tools
```

---

##  3. Source the Workspace

To use anything you install or create inside the workspace, you must **source** the setup script.

### Inside the install folder

```
ros2_ws/install/setup.bash
```

### Add this to `.bashrc` so it happens automatically:

```bash
nano ~/.bashrc
```

Add these lines **in order**:

```bash
source /opt/ros/humble/setup.bash    # Global ROS2 installation
source ~/ros2_ws/install/setup.bash  # Your workspace
```

Save and open a new terminal.

---

## 4. Create Your First ROS2 Python Package

To create a package:

### Navigate to the `src/` folder:

```bash
cd ~/ros2_ws/src
```

### Use the ROS2 package creation command:

```bash
ros2 pkg create my_py_pkg \
  --build-type ament_python \
  --dependencies rclpy
```

### What this does:

* Creates a new folder: `my_py_pkg`
* Sets up Python package structure
* Adds the dependency **rclpy** (ROS2 Python client library)

Ignore warnings about missing license files unless you plan to publish your code.

---

## 5. Understanding the Package Structure

Inside `my_py_pkg/` you will find:

```
my_py_pkg/
 ├── my_py_pkg/            # Python module (write your nodes here)
 │    └── __init__.py
 ├── resource/
 ├── test/
 ├── package.xml           # Package metadata + dependencies
 ├── setup.py              # Install script
 └── setup.cfg             # Configuration for installation
```

### 🔍 Important Files

#### **package.xml**

* Contains package name, version, description
* Stores dependencies (like `rclpy`)
* Defines build type (`ament_python`)

Add new dependencies here when needed.

#### **setup.py / setup.cfg**

* Used when installing and running your Python nodes
* You will modify these when adding executable scripts

---

## 6. Build the Package

Return to the workspace root:

```bash
cd ~/ros2_ws
colcon build
```

You can also build only a specific package:

```bash
colcon build --packages-select my_py_pkg
```

---

## 7. Workspace Ready for Node Creation

Your Python package is now ready to host ROS2 nodes.
In the next step, you'll write your first node inside:

```
my_py_pkg/my_py_pkg/
```

---

## Summary

* A ROS2 workspace holds all your code and must be built before use.
* Always source both the global ROS installation **and** your workspace.
* Use `ros2 pkg create` to create structured, dependency-aware packages.
* Your Python code lives inside the package’s internal folder.
* Build the workspace with `colcon build` to make the package usable.

---

Happy coding! 🚀
Your ROS2 environment is now fully set up and ready for node development.

```
