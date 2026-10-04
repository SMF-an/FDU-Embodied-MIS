#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStatePrefixBridge(Node):
    def __init__(self):
        super().__init__("joint_state_prefix_bridge")
        self.declare_parameter("input_topic", "joint_states")
        self.declare_parameter("output_topic", "moveit_joint_states")
        self.declare_parameter("joint_prefix", "")
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._prefix = self.get_parameter("joint_prefix").value
        self._publisher = self.create_publisher(JointState, output_topic, 10)
        self.create_subscription(JointState, input_topic, self._on_joint_state, 10)

    def _on_joint_state(self, message):
        if not message.name or any(not name.startswith(self._prefix) for name in message.name):
            self.get_logger().warning("Ignoring joint state with unexpected joint names")
            return
        output = JointState()
        output.header = message.header
        output.name = [name[len(self._prefix):] for name in message.name]
        output.position = message.position
        output.velocity = message.velocity
        output.effort = message.effort
        self._publisher.publish(output)


def main():
    rclpy.init()
    node = JointStatePrefixBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
