# Topic 4 — Running & Managing ROS2 Nodes (CLI, Introspection, remapping, `--symlink-install`, `rqt`, turtlesim)

This note summarizes how to run, inspect, configure and iterate on ROS2 nodes (Python & C++), plus useful CLI options and development tips. Keep this as a `README.md` for quick reference.

---

## 1. Recap — executables after build

* After you **build & install** a package, ROS2 places the executable inside the workspace `install/` folder.
* Use `ros2 run <package> <executable>` to start a node.

  * Example: `ros2 run demo_nodes_cpp talker`

---

## 2. Environment & sourcing (very important)

* Make sure both **global ROS2** and **your workspace** are sourced in this order:

```bash
# ~/.bashrc
source /opt/ros/<distro>/setup.bash
source ~/ros2_ws/install/setup.bash
```

* If you add/modify packages, re-open terminal or `source ~/.bashrc` (or `source ~/ros2_ws/install/setup.bash`) so shell autocompletion and `ros2 run` find your packages.

---

## 3. Useful `ros2` CLI quick commands

* Show help for any command:

```bash
ros2 <command> -h
```

* Run a node:

```bash
ros2 run <package_name> <executable_name>
```

* Introspect running nodes:

```bash
ros2 node list
ros2 node info /node_name
```

* Examples:

```bash
ros2 run my_py_pkg py_node
ros2 node list
ros2 node info /py_test
```

---

## 4. Node name conflicts & renaming (remap)

* **Two nodes with the same name** in the same namespace can cause unexpected behavior. Avoid identical names.
* You can **rename** (remap) a node at runtime — no code change required — using ROS args:

```bash
ros2 run <pkg> <exe> --ros-args -r __node:=new_name
# short form: -r __node:=new_name
```

* Example: start two instances with different names:

```bash
ros2 run my_py_pkg py_node --ros-args -r __node:=sensor_01
ros2 run my_py_pkg py_node --ros-args -r __node:=sensor_02
```

* `--ros-args` prefix is needed before any ROS remap or parameter args.

---

## 5. Development speedup for Python: `--symlink-install`

* When developing Python nodes, repeatedly building is slow. Use `colcon build --symlink-install`:

  * On first install: `colcon build --packages-select my_py_pkg --symlink-install`
  * This makes `ros2 run` execute the Python file directly from `src/`, so edits are visible immediately — **no rebuild required**.
* **Caution**: `--symlink-install` can cause subtle issues; if something odd happens, rebuild without it:

```bash
colcon build --packages-select my_py_pkg
```

* Recommended workflow:

  1. Build once with `--symlink-install` during rapid development.
  2. Before release or when debugging weird errors, rebuild normally (no `--symlink-install`) and test.

---

## 6. `colcon build` tips

* Always build from workspace root (NOT inside `src/`):

```bash
cd ~/ros2_ws
colcon build
```

* Build only specific packages:

```bash
colcon build --packages-select my_py_pkg another_pkg
```

---

## 7. `rqt` (GUI introspection & debugging)

* `rqt` offers graphical plugins for introspection, topics, services, parameters, and node graph.
* Launch GUI:

```bash
rqt
# or for node graph directly
rqt_graph
```

* Useful plugins:

  * **Node Graph** (`rqt_graph`) — visualize nodes and topics
  * **Topic Monitor / Topic Tools** — inspect messages
  * **Service Tester** — call/test services
  * **Parameter Inspector** — view/change parameters live
* `rqt_graph` shows nodes, topics and connections — great for quick architecture checks.

---

## 8. Practical example — `turtlesim`

* Launch `turtlesim` node (desktop install usually includes this):

```bash
ros2 run turtlesim turtlesim_node
```

* Run teleop to control the turtle:

```bash
ros2 run turtlesim turtle_teleop_key
# focus the terminal and use arrow keys to move the turtle
```

* Visualize graph in `rqt_graph` to see `turtlesim` and `teleop` nodes and topic connections (e.g., `/turtle1/cmd_vel`).

---

## 9. Runtime remapping example (turtlesim)

* Rename the node at launch:

```bash
ros2 run turtlesim turtlesim_node --ros-args -r __node:=my_turtle
```

* Graph updates reflect the remapped name immediately.

---

## 10. Common troubleshooting & tips

* `Package not found` when running your own package:

  * You didn’t source the workspace (`source ~/ros2_ws/install/setup.bash`) after building.
  * Or you built in a different terminal without sourcing.
* If Python runtime code changes don’t appear:

  * If not using `--symlink-install`, you must rebuild.
* If weird behavior occurs with `--symlink-install`:

  * Rebuild without it: `colcon build --packages-select <pkg>`.
* Use `ros2 node info /node_name` to see publishers/subscribers/services/parameters the node uses.
* Use `ros2 topic echo /topic_name` and `ros2 topic list` for quick message inspection.

---

## 11. Recommended iteration loop (Python)

1. Create package & node.
2. `colcon build --packages-select <pkg> --symlink-install` (dev only).
3. `source ~/ros2_ws/install/setup.bash`
4. `ros2 run <pkg> <exe>`
5. Edit Python file in `src/`, save, re-run the node (no rebuild needed with symlink).
6. When stable: `colcon build --packages-select <pkg>` (normal install) and test again.

---

## 12. Summary 

* Use `ros2 run` to start nodes; `ros2 node list/info` and `rqt_graph` to inspect them.
* Always source global ROS first, then your workspace.
* Avoid duplicate node names; remap names at runtime using `--ros-args -r __node:=name`.
* `--symlink-install` dramatically speeds up Python editing/iteration but use cautiously.
* `rqt` provides powerful visual debugging tools (graph, topics, parameters, services).

---
