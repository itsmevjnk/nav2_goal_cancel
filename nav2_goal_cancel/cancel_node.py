from rclpy import qos
from rclpy.node import Node

from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped

ACTION_CANCEL_SERVICE = '/navigate_to_pose/_action/cancel_goal'

class GoalCancelNode(Node):
    def __init__(self, prefix):
        super().__init__(f'{prefix}_goal_cancel')

        action = self.declare_parameter('action', 'navigate_to_pose').get_parameter_value().string_value
        self.cancel_srv = self.create_client(CancelGoal, f'/{action}/_action/cancel_goal')
        while not self.cancel_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'waiting for action server {action}')

        self.last_state = True # True = pass, False = stop

        qos_profile = qos.QoSProfile(
            reliability=qos.QoSReliabilityPolicy.RELIABLE,
            depth=1,
            durability=qos.QoSDurabilityPolicy.VOLATILE
        ) # same as bt_navigator
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_cb, qos_profile)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', qos_profile)

        self.last_known_goal: PoseStamped | None = None

        self.get_logger().info('ready to receive messages')

    def goal_cb(self, data: PoseStamped):
        self.get_logger().info(f'saving goal pose: position ({data.pose.position.x}, {data.pose.position.y}), orientation ({data.pose.orientation.z}, {data.pose.orientation.w})')
        self.last_known_goal = data
        self.last_state = True # moving
    
    def set_state(self, state):
        if state != self.last_state:
            self.last_state = state
            if not state: self.cancel_goal()
            else: self.renavigate_goal()

    def cancel_goal(self):
        self.get_logger().info('cancelling all goals')
        
        request = CancelGoal.Request() # should be all zeros
        self.cancel_srv.call_async(request) # we don't care about the result anyway
    
    def renavigate_goal(self):
        if self.last_known_goal is None:
            self.get_logger().error('no goal pose received by node, not resuming')
            return
        
        data = self.last_known_goal
        data.header.stamp = self.get_clock().now().to_msg() # not sure if needed

        self.get_logger().info(f'navigating to last known goal pose: position ({data.pose.position.x}, {data.pose.position.y}), orientation ({data.pose.orientation.z}, {data.pose.orientation.w})')
        self.goal_pub.publish(data)

