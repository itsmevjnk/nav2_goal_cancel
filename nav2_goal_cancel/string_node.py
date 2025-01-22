import rclpy
from rclpy import qos

from std_msgs.msg import String
from .cancel_node import GoalCancelNode

class StringCancelNode(GoalCancelNode):
    def __init__(self):
        super().__init__('string')

        self.name = self.declare_parameter('robot_name', 'robot').get_parameter_value().string_value

        self.create_subscription(String, 'pass', self.pass_cb, qos.qos_profile_system_default)
        self.create_subscription(String, 'stop', self.stop_cb, qos.qos_profile_system_default)

    def pass_cb(self, data: String):
        if data.data == self.name:
            self.set_state(True)

    def stop_cb(self, data: String):
        if data.data == self.name:
            self.set_state(False)

def main():
    rclpy.init()
    node = StringCancelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
