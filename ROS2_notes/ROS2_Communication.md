# 🟦 Topic — ROS2 Topics, Publishers & Subscribers 

## 1. Intuition — radio analogy

* Think of a **topic** as a radio **frequency / station** (a named bus).

  * Publisher = **radio transmitter** (sends a data stream on a frequency).
  * Subscriber = **radio receiver / phone / car** (listens to that frequency).
* A topic has:

  * a **name** (like a frequency) and
  * a **message type** (the format the transmitter and receiver must agree on).
* Multiple publishers → many transmitters can broadcast on the same topic.
  Multiple subscribers → many receivers can listen to the same topic.
* Publishers and subscribers are **anonymous** to each other — they only know the topic name and message type, not the identity of peers.
* Topics are **unidirectional** streams: publisher → topic → subscriber (no direct response).

---

## 2. Technical definition (concise)

* A **topic** is a named bus over which nodes exchange messages.
* Use topics when you need continuous/unidirectional streaming (e.g., sensor data, velocity commands).
* Messages are transported by the ROS2 middleware (DDS); client libraries (`rclpy`, `rclcpp`, etc.) abstract DDS details.

---

## 3. Topic names & message types

* **Topic name rules** (summary): must begin with a letter; may include letters, digits, underscores, slashes, tildes, etc. (follow ROS naming rules — use readable names and namespaces).
* **Message type**: every topic has an associated message type (e.g., `example_interfaces/msg/String`, `sensor_msgs/msg/Image`).
  All publishers/subscribers on a topic **must use the same message type**.

---

## 4. Publisher & Subscriber (concept + small Python templates)

### Publisher (concept)

* A node creates a publisher to send messages to a topic.
* Publisher API lets you `publish()` messages at any rate (often used with timers).

### Subscriber (concept)

* A node creates a subscriber to receive messages from a topic.
* Subscriber registers a callback invoked whenever a new message arrives.

### Minimal Python examples (rclpy)

**publisher.py**

```python
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class NewsPublisher(Node):
    def __init__(self):
        super().__init__('news_publisher')
        self.pub = self.create_publisher(String, '/robot_news', 10)
        self.create_timer(0.5, self.timer_cb)  # 2 Hz
        self.i = 0

    def timer_cb(self):
        msg = String()
        msg.data = f'hello {self.i}'
        self.pub.publish(msg)
        self.get_logger().info(f'published: {msg.data}')
        self.i += 1

def main():
    rclpy.init()
    node = NewsPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

**subscriber.py**

```python
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class NewsSubscriber(Node):
    def __init__(self):
        super().__init__('news_subscriber')
        self.sub = self.create_subscription(String, '/robot_news', self.cb, 10)

    def cb(self, msg):
        self.get_logger().info(f'received: {msg.data}')

def main():
    rclpy.init()
    node = NewsSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

## 5. Useful `ros2` topic CLI commands (introspection & tools)

* **List topics**

```bash
ros2 topic list
```

* **Info (type, pubs, subs)**

```bash
ros2 topic info /robot_news
# shows interface type (e.g., example_interfaces/msg/String),
# number of publishers and subscribers
```

* **Show message structure for an interface**

```bash
ros2 interface show example_interfaces/msg/String
# displays fields (e.g., string data)
```

* **Echo (subscribe from terminal)**

```bash
ros2 topic echo /robot_news
# prints incoming messages to terminal; acts like a subscriber
```

* **Publish from terminal**

```bash
ros2 topic pub -r 5 /robot_news example_interfaces/msg/String "{data: 'hello from terminal'}"
# -r 5 -> publish at 5 Hz
```

> Note: `ros2 topic pub` is best for simple message types. Complex nested types are hard to compose on the CLI.

* **Measure publish rate (Hz)**

```bash
ros2 topic hz /robot_news
# waits a few seconds and reports average rate (Hz)
```

* **Measure bandwidth**

```bash
ros2 topic bw /robot_news
# measures bytes/sec and messages/sec
```

---

## 6. Remapping topics at runtime

* You can remap topics (and other names) when starting a node — no code changes required.
* Example: remap a node’s `/default_topic` to `/my_topic`:

```bash
ros2 run <pkg> <exe> --ros-args -r /default_topic:=/my_topic
# short: -r <from>:=<to>
```

* This is useful to run multiple instances, isolate test topics, or connect existing nodes to different topics.

---

## 7. rqt_graph and visual introspection

* `rqt_graph` (or `rqt` → Node Graph plugin) visualizes nodes, topics, and their connections:

```bash
rqt_graph
# or rqt then open Node Graph plugin
```

* Graph shows publishers → topics → subscribers, making it easy to debug wiring and remapping.

---

## 8. Practical behaviors & edge cases

* **Topic visible only while active**: `ros2 topic list` shows a topic only if there's at least one publisher or subscriber.
* **No publisher**: subscribers exist but receive nothing until a publisher appears.
* **No subscriber**: publishers send messages but nobody receives them (fire-and-forget stream).
* **Multiple publishers**: subscribers receive messages from all publishers on that topic (order depends on DDS).
* **Type mismatch**: publisher and subscriber must share the same message type — otherwise communication fails.

---

## 9. Combined tools for debugging

* `ros2 node info /node_name` shows which topics a node publishes and subscribes to.
* Use combinations to debug:

  * `ros2 node list` → find node names
  * `ros2 node info /node` → see topic names & interfaces
  * `ros2 topic info /topic` → confirm types and counts
  * `ros2 topic echo /topic` → see actual messages
  * `rqt_graph` → visualize structure

---



