from rclpy import qos
from rclpy.node import Node

from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped

from scipy.spatial.transform import Rotation

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
        self.goal_sub_alt = self.create_subscription(PoseStamped, 'goal_pose_alt', self.goal_cb, qos_profile)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', qos_profile)

        self.last_known_goal: PoseStamped | None = None

        init_goal = self.declare_parameter('init_goal', False).get_parameter_value().bool_value
        if init_goal:
            x = self.declare_parameter('init_x', 0.0).get_parameter_value().double_value
            y = self.declare_parameter('init_y', 0.0).get_parameter_value().double_value
            yaw = self.declare_parameter('init_yaw', 0.0).get_parameter_value().double_value
            frame = self.declare_parameter('init_frame', 'map').get_parameter_value().string_value
            self.last_known_goal = PoseStamped()
            self.last_known_goal.header.frame_id = frame
            self.last_known_goal.pose.position.x = x
            self.last_known_goal.pose.position.y = y
            self.last_known_goal.pose.orientation.x, self.last_known_goal.pose.orientation.y, self.last_known_goal.pose.orientation.z, self.last_known_goal.pose.orientation.w = Rotation.from_euler('z', yaw).as_quat()
            self.get_logger().info(f'initial goal set to (({x},{y}),{yaw})')

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

