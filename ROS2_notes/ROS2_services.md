# 🟦 ROS2 Services 

## 1. Intuition — real-life analogy (quick)

* Imagine an **online weather service**:

  * Your computer = **client** (asks for weather).
  * Weather server = **server** (receives location, returns weather).
  * HTTP URL ≈ **ROS2 service name**.
  * Client sends a **request** (e.g., location). Server processes it and returns a **response** (weather).
* Key rules from the analogy:

  * Request and response must match the **service interface** (expected fields/types).
  * **Many clients** can call the same server.
  * Usually **one server** per service name (server provides the service).

---

## 2. What is a ROS2 service? (concise)

* A **service** = synchronous request/response RPC-style communication between nodes.
* A service has:

  * a **name** (like a URL)
  * an **interface** (two messages: `Request` and `Response`, separated by `---`)
* Use services when you need an RPC (do X → get result) rather than a continuous stream (use topics for streaming).

---

## 3. When to use services (vs topics)

* **Use a service** when you need:

  * an action/query with a reply (e.g., enable motor, compute IK, toggle LED, database lookup).
  * guaranteed request/response semantics.
* **Use a topic** when you need continuous, unidirectional streaming (e.g., sensor data, cmd_vel).

---

## 4. Service characteristics & best practices

* **Single server** per service name is standard; **multiple clients** call that server.
* Services can be called **synchronously** (blocking) or **asynchronously** (preferred).
* **Name services with verbs** (e.g., `set_led`, `compute_ik`, `add_two_ints`) — they represent actions or computations.
* All clients and server must agree on the **service type**.
* Servers exist once they are created; clients should **wait_for_service()** before calling.

---

## 5. Service interface (example: `AddTwoInts`)

* Service interfaces are under `srv/`, not `msg/`.
* Example: `example_interfaces/srv/AddTwoInts`:

  ```
  # request
  int64 a
  int64 b
  ---
  # response
  int64 sum
  ```
* The `---` separates request and response definitions.

Command to inspect a service interface:

```bash
ros2 interface show example_interfaces/srv/AddTwoInts
```

---

## 6. CLI introspection & test commands

* **List services**

```bash
ros2 service list
```

* **Info about a service (type, providers)**

```bash
ros2 service type /add_two_ints
# then:
ros2 service info /add_two_ints
```

* **Call a service from terminal**

```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 7}"
# returns the response from the server
```

> Terminal `ros2 service call` is convenient for simple requests. For complex nested requests, write a client script.

---

## 7. Remapping a service name at runtime

* You can remap service names when launching a node — no code change required:

```bash
ros2 run <pkg> <exe> --ros-args -r /old_service_name:=/new_service_name
# short: -r <from>:=<to>
```

* Example:

```bash
ros2 run my_pkg my_server --ros-args -r __node:=led_server -r /set_led:=/robot1/set_led
```

* This lets you run the same server code under different names or connect clients to different service names.

---

## 8. Python examples (server & client)

### 8.1 Service server (Python, `rclpy`)

`add_two_ints_server.py` — a minimal, clear server:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        # create_service(Type, name, callback)
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('AddTwoInts service ready')

    def add_two_ints_callback(self, request, response):
        # request: AddTwoInts.Request, response: AddTwoInts.Response
        response.sum = request.a + request.b
        self.get_logger().info(f'Received request: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Notes:**

* Create server in constructor.
* Provide `type`, `name`, and `callback`.
* **Always return** the `response` object from the callback.

---

### 8.2 Simple non-OOP client (Python, `rclpy`)

`add_two_ints_client_no_oop.py` — useful test template:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from rclpy.task import Future

def main(args=None):
    rclpy.init(args=args)
    node = Node('add_two_ints_client_no_oop')

    client = node.create_client(AddTwoInts, 'add_two_ints')

    # wait for server
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().warning('Waiting for add_two_ints service...')

    # build request
    req = AddTwoInts.Request()
    req.a = 3
    req.b = 8

    # asynchronous call (preferred)
    future = client.call_async(req)

    # block until response arrives
    rclpy.spin_until_future_complete(node, future)
    if future.done():
        res = future.result()
        node.get_logger().info(f'{req.a} + {req.b} = {res.sum}')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Notes:**

* Use `wait_for_service()` to avoid calling a non-existing server.
* Use `call_async()` and `spin_until_future_complete()` (preferred) — avoids blocking the node's executor incorrectly.
* `call()` (sync) exists but may cause deadlocks unless carefully used.

---

## 9. Integrating a client into an OOP node

* Inside a node class use:

  * `self.client = self.create_client(MySrvType, 'my_service')`
  * `self.client.call_async(request)` and handle the future with `add_done_callback(...)` or `spin_until_future_complete(...)` from outside.
* For repeated or background requests, use async + callbacks to keep the node reactive.

---

## 10. Error cases & debug tips

* **No service found** → client will error unless `wait_for_service()` used.
* **Type mismatch** between client and server → call will fail (must use same service type).
* **Multiple servers** on same name → usually avoided; remap to give unique names.
* **Complex request structures** → prefer writing a small client script versus `ros2 service call`.
* Use `ros2 service list`, `ros2 service info /name`, and `rqt_graph` (visualize services via graph plugins) to debug.

---

## 11. Quick reference — common commands

```bash
ros2 service list
ros2 service info /add_two_ints
ros2 service type /add_two_ints
ros2 interface show example_interfaces/srv/AddTwoInts
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 7}"
# Remap service at runtime
ros2 run my_pkg my_server --ros-args -r /add_two_ints:=/robot1/add_two_ints
```

---

## 12. Summary (short)

* **Services** = RPC-style client/server with a named interface (`Request` / `Response`).
* Use services for single-shot requests or commands requiring a reply.
* Servers: `create_service(Type, name, callback)` → callback returns `response`.
* Clients: `create_client(Type, name)` → `call_async(req)` + `spin_until_future_complete()` or async callbacks.
* Remapping allows runtime renaming of services without changing code.

---
