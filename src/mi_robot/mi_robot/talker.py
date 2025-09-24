# src/mi_robot/mi_robot/talker.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.tick)
        self.i = 0

    def tick(self):
        msg = String()
        msg.data = f"Hello ROS 2 ({self.i})"
        self.pub.publish(msg)
        self.get_logger().info(msg.data)
        self.i += 1

def main():
    rclpy.init()
    node = Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
