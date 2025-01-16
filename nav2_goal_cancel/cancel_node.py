import rclpy
from rclpy import qos
from rclpy.node import Node

from std_msgs.msg import Empty, Bool
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped

ACTION_CANCEL_SERVICE = '/navigate_to_pose/_action/cancel_goal'

class GoalCancelNode(Node):
    def __init__(self):
        super().__init__('nav2_goal_cancel')

        self.cancel_srv = self.create_client(CancelGoal, ACTION_CANCEL_SERVICE)
        while not self.cancel_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for action server')

        # self.cancel_sub = self.create_subscription(Empty, 'cancel_goal', self.cancel_cb, qos.qos_profile_sensor_data) # TODO
        # self.renav_sub = self.create_subscription(Empty, 'renavigate_goal', self.renav_cb, qos.qos_profile_sensor_data) # TODO

        self.last_state = True # True = pass, False = stop
        self.pass_sub = self.create_subscription(Bool, 'pass', self.pass_cb, qos.qos_profile_system_default)

        qos_profile = qos.QoSProfile(
            reliability=qos.QoSReliabilityPolicy.RELIABLE,
            depth=1,
            durability=qos.QoSDurabilityPolicy.VOLATILE
        ) # same as bt_navigator
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_cb, qos_profile)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', qos_profile)

        self.last_known_goal: PoseStamped | None = None

        self.get_logger().info('ready to receive messages')

    def pass_cb(self, data: Bool):
        state = data.data
        # self.get_logger().info(f'received message from pass topic: {state}')

        if state != self.last_state:
            self.last_state = state
            if state: self.renav_cb(None)
            else: self.cancel_cb(None)

    def cancel_cb(self, data):
        self.get_logger().info('cancelling all goals')
        
        request = CancelGoal.Request() # should be all zeros
        self.cancel_srv.call_async(request) # we don't care about the result anyway
    
    def goal_cb(self, data: PoseStamped):
        self.get_logger().info(f'saving goal pose: position ({data.pose.position.x}, {data.pose.position.y}), orientation ({data.pose.orientation.z}, {data.pose.orientation.w})')
        self.last_known_goal = data
        self.last_state = True # moving
    
    def renav_cb(self, data):
        if self.last_known_goal is None:
            self.get_logger().error('no goal pose received by node, not resuming')
            return
        
        data = self.last_known_goal
        data.header.stamp = self.get_clock().now().to_msg() # not sure if needed

        self.get_logger().info(f'navigating to last known goal pose: position ({data.pose.position.x}, {data.pose.position.y}), orientation ({data.pose.orientation.z}, {data.pose.orientation.w})')
        self.goal_pub.publish(data)

def main():
    rclpy.init()
    node = GoalCancelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
