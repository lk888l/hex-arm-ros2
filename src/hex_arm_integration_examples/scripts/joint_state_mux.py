#!/usr/bin/env python3
"""Forward disjoint sensor streams to the robot's public joint_states topic.

Each message retains its original stamp and joint subset. Robot state publisher
and MoveIt accumulate these updates. No zero positions or fresh stamps are
invented for missing sensors. This stream is never a driver safety input.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class JointStateMux(Node):
    def __init__(self):
        super().__init__("joint_state_mux")
        sources = self.declare_parameter("sources", ["/left/joint_states", "/right/joint_states"]).value
        output = self.resolve_topic_name("joint_states")
        if any(self.resolve_topic_name(source) == output for source in sources):
            raise ValueError("joint state mux input cannot equal its output")
        self.publisher = self.create_publisher(JointState, "joint_states", 10)
        self.owners = {}
        self.subscriptions_ = [self.create_subscription(
            JointState, source, lambda message, source=source: self.forward(source, message),
            qos_profile_sensor_data) for source in sources]

    def forward(self, source, message):
        if len(message.name) != len(set(message.name)):
            self.get_logger().error(f"Duplicate joint names in {source}; dropping sample")
            return
        if any(len(values) not in (0, len(message.name))
               for values in (message.position, message.velocity, message.effort)):
            self.get_logger().error(f"Invalid joint state array lengths from {source}")
            return
        if any(self.owners.get(name, source) != source for name in message.name):
            self.get_logger().error(f"Conflicting joint ownership from {source}; dropping sample")
            return
        self.owners.update({name: source for name in message.name})
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = JointStateMux()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
