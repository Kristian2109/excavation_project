"""
raw_urdf_publisher.py – Publishes the raw URDF string on a latched topic.

robot_state_publisher strips <ros2_control> tags from the published URDF.
The controller_manager needs the full URDF including those tags.
This node re-publishes the unmodified URDF for the controller_manager.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String


class RawUrdfPublisher(Node):
    def __init__(self):
        super().__init__('raw_urdf_publisher')
        self.declare_parameter('robot_description', '')

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(String, '/robot_description_raw', qos)

        urdf = self.get_parameter('robot_description').value
        if urdf:
            msg = String()
            msg.data = urdf
            self.pub.publish(msg)
            self.get_logger().info(
                f'Published raw URDF ({len(urdf)} chars) on /robot_description_raw')
        else:
            self.get_logger().error('robot_description parameter is empty!')


def main(args=None):
    rclpy.init(args=args)
    node = RawUrdfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
