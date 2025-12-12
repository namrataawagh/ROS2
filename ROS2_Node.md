# 🟦 Topic 3 — ROS2 Node

## What is a ROS2 node?

* A **node** is a single-purpose subprogram (process) of a robot application.
* Nodes are grouped into **packages**. A package contains one or more nodes and related resources.
* Each node should have **one clear responsibility** (e.g., camera driver, image processor, motion planner, motor controller).
* Nodes run in separate processes and **communicate** with each other using ROS2 communication primitives (topics, services, actions, parameters, etc.).

---

## Why use nodes?

* **Reduce complexity**: splitting functionality into nodes keeps code modular and maintainable.
* **Scalability**: adding sensors/algorithms is easier when responsibilities are separated.
* **Fault tolerance**: a crash in one node usually does not crash others (process isolation).
* **Language-agnostic**: nodes can be written in different languages (commonly Python `rclpy` or C++ `rclcpp`) and still interoperate.
* **Reusability**: nodes/packages can be reused across projects.

---

## Typical application architecture (example)

* **Camera package**

  * `camera_driver_node` — reads frames from camera
  * `image_processing_node` — processes frames, publishes environment info
* **Motion planning package**

  * `motion_planning_node` — computes trajectories
  * `path_correction_node` — adjusts trajectories using perception input
* **Hardware control package**

  * `motor_driver_node` — executes velocity/position commands
  * `state_publisher_node` — publishes encoder/health data

Data flow example: `camera_driver_node` → `image_processing_node` → `path_correction_node` → `motion_planning_node` → `motor_driver_node`

---

## Node characteristics & rules

* Nodes are combined into a **graph** and communicate via ROS2 interfaces.
* **One node = one primary purpose** (like one class in OOP).
* **Node names must be unique** in the same namespace. To run multiple instances, use unique names or different namespaces.
* Use nodes to isolate **critical** vs **experimental** functionality (prevents non-critical bugs from affecting critical systems).

---

## ROS2 client libraries (brief)

* **RCL (ROS Client Library)**: low-level C library that interfaces with the ROS2 middleware (DDS). It is the base for higher-level client libs.
* **rclcpp**: C++ client library built on top of RCL.
* **rclpy**: Python client library built on top of RCL.
* Other language bindings (Node.js, Java, etc.) exist, often community-maintained.

---

## Node lifecycle basics (Python)

* Typical steps in a Python node:

  1. `rclpy.init()` — initialize ROS2.
  2. Create a node class inheriting from `rclpy.node.Node`.
  3. Add publishers/subscribers/services/actions/timers/parameters as needed.
  4. `rclpy.spin(node)` — keep the node alive and process callbacks.
  5. On shutdown: `node.destroy_node()` and `rclpy.shutdown()`.

---

## Common pattern: class-based node (template)

```python
# my_node.py
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')               # node name
        self.get_logger().info('MyNode started')  # logging
        self.counter_ = 0                         # example state attribute
        # Timer: call timer_callback every 1.0 second
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info(f'hello {self.counter_}')
        self.counter_ += 1

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)           # process callbacks (timers, subscriptions, etc.)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Notes:

* `create_timer(period_sec, callback)` registers a callback to run periodically (very common for sensor polling, status publishing).
* Give attributes trailing underscores (e.g., `counter_`) as a convention for node internal state (optional).
* Avoid calling callback function when registering (`self.timer_callback`, *no parentheses*).

---

## Timers & Callbacks

* **Timer**: schedule a function to run periodically (e.g., read sensor at 10 Hz).
* **Callback**: function invoked by timers, subscribers, service requests, or action progress.
* `rclpy.spin(node)` ensures callbacks are executed while the node is running.

---

## Debugging & errors

* **Syntax errors** appear at build time for Python packages (colcon build will fail).
* Runtime exceptions may appear only when the node runs — use logs and introspection tools (`ros2 topic echo`, `ros2 node list`, `ros2 topic info`) to debug.
* Keep critical nodes well-tested; non-critical experimental nodes can be run separately.

---

## Example workflow (develop → run)

1. Write node code inside package (e.g., `my_py_pkg/my_py_pkg/my_node.py`).
2. Ensure `setup.py`/`setup.cfg` expose the executable entry point (for `ros2 run`).
3. Build workspace from workspace root:

   ```bash
   cd ~/ros2_ws
   colcon build
   ```
4. Source workspace:

   ```bash
   source ~/ros2_ws/install/setup.bash
   ```
5. Run node:

   ```bash
   ros2 run my_py_pkg my_node_executable
   ```

---

## Good practices

* Name nodes & packages meaningfully: e.g., `camera_driver_node`, `image_processing_node`.
* Keep a node **single-responsibility** (one main task).
* Group related nodes in packages, but avoid overly large packages — split logical concerns.
* Use logging heavily (`get_logger().info/debug/warn/error`) to trace behavior.
* Use timers for periodic tasks; prefer callbacks for event-driven code.
* Use namespaces if you need multiple instances of the same node (e.g., multi-robot systems).

---

## Summary

* A **ROS2 node** = single-purpose process that uses ROS2 communication to interact with other nodes.
* Nodes improve modularity, scalability, and fault tolerance.
* Use Python (`rclpy`) or C++ (`rclcpp`) client libraries — both sit on top of RCL and are interoperable.
* Timers, publishers, subscribers, services, actions, and parameters are the building blocks for node interactions.
* Class-based node templates with `create_timer` and `rclpy.spin()` are a simple and reusable starting point.

---

If you want, I can:

* generate a **copy-paste-ready node template file** with the correct `setup.py` entry_points for `ros2 run`; or
* produce a small **diagram** (package ↔ nodes ↔ topics) to visualize the example architecture.
