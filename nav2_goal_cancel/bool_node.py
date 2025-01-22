import rclpy
from rclpy import qos

from std_msgs.msg import Bool
from .cancel_node import GoalCancelNode

class BoolCancelNode(GoalCancelNode):
    def __init__(self):
        super().__init__('bool')

        self.create_subscription(Bool, 'pass', self.pass_cb, qos.qos_profile_system_default)

    def pass_cb(self, data: Bool):
        self.set_state(data.data)

def main():
    rclpy.init()
    node = BoolCancelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
