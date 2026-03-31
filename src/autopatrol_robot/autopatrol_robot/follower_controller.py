import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from gazebo_msgs.srv import GetEntityState
from rclpy.node import Node


class FollowerController(Node):
    def __init__(self):
        super().__init__('follower_controller')

        self.declare_parameter('target_pose_topic', '/main_robot/follow_target_pose')
        self.declare_parameter('follower_entity_name', 'follower_robot')
        self.declare_parameter('entity_state_service', '/gazebo_state/get_entity_state')
        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('cmd_vel_topic', '/follower_robot/cmd_vel')
        self.declare_parameter('follow_distance', 1.0)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('linear_gain', 0.8)
        self.declare_parameter('angular_gain', 2.0)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_reverse_speed', 0.3)
        self.declare_parameter('max_angular_speed', 1.8)
        self.declare_parameter('position_tolerance', 0.08)
        self.declare_parameter('heading_tolerance', 0.08)

        self.target_pose_topic = self.get_parameter('target_pose_topic').value
        self.follower_entity_name = self.get_parameter('follower_entity_name').value
        self.entity_state_service = self.get_parameter('entity_state_service').value
        self.reference_frame = self.get_parameter('reference_frame').value
        self.follow_distance = float(self.get_parameter('follow_distance').value)
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.linear_gain = float(self.get_parameter('linear_gain').value)
        self.angular_gain = float(self.get_parameter('angular_gain').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_reverse_speed = float(self.get_parameter('max_reverse_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.position_tolerance = float(self.get_parameter('position_tolerance').value)
        self.heading_tolerance = float(self.get_parameter('heading_tolerance').value)

        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.target_pose_sub = self.create_subscription(
            PoseStamped,
            self.target_pose_topic,
            self.target_pose_callback,
            10,
        )
        self.entity_state_client = self.create_client(GetEntityState, self.entity_state_service)
        self.target_pose = None
        self.pending_future = None
        self.prev_linear = 0.0
        self.prev_angular = 0.0
        self.smoothing_factor = 0.3
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

    def clamp(self, value, lower, upper):
        return max(lower, min(value, upper))

    def yaw_from_quaternion(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def target_pose_callback(self, msg):
        self.target_pose = msg.pose

    def publish_stop(self):
        if not rclpy.ok():
            return
        self.cmd_pub.publish(Twist())

    def control_loop(self):
        if self.target_pose is None:
            self.publish_stop()
            self.get_logger().warn(
                f'Waiting for target pose on {self.target_pose_topic}',
                throttle_duration_sec=5.0,
            )
            return

        if not self.entity_state_client.wait_for_service(timeout_sec=0.0):
            self.publish_stop()
            self.get_logger().warn(
                f'Waiting for entity state service {self.entity_state_service}',
                throttle_duration_sec=5.0,
            )
            return

        if self.pending_future is not None and not self.pending_future.done():
            return

        request = GetEntityState.Request()
        request.name = self.follower_entity_name
        request.reference_frame = self.reference_frame
        self.pending_future = self.entity_state_client.call_async(request)
        self.pending_future.add_done_callback(self.handle_entity_state)

    def handle_entity_state(self, future):
        self.pending_future = None

        try:
            response = future.result()
        except Exception as exc:
            self.publish_stop()
            self.get_logger().warn(str(exc), throttle_duration_sec=5.0)
            return

        if not response.success:
            self.publish_stop()
            self.get_logger().warn(
                f'GetEntityState failed for {self.follower_entity_name}: {response.status_message}',
                throttle_duration_sec=5.0,
            )
            return

        if self.target_pose is None:
            self.publish_stop()
            return

        leader_x = self.target_pose.position.x
        leader_y = self.target_pose.position.y
        leader_yaw = self.yaw_from_quaternion(
            self.target_pose.orientation.x,
            self.target_pose.orientation.y,
            self.target_pose.orientation.z,
            self.target_pose.orientation.w,
        )
        follower_pose = response.state.pose
        follower_x = follower_pose.position.x
        follower_y = follower_pose.position.y
        follower_yaw = self.yaw_from_quaternion(
            follower_pose.orientation.x,
            follower_pose.orientation.y,
            follower_pose.orientation.z,
            follower_pose.orientation.w,
        )

        target_world_x = leader_x - self.follow_distance * math.cos(leader_yaw)
        target_world_y = leader_y - self.follow_distance * math.sin(leader_yaw)

        error_world_x = target_world_x - follower_x
        error_world_y = target_world_y - follower_y

        cos_yaw = math.cos(follower_yaw)
        sin_yaw = math.sin(follower_yaw)
        target_x = cos_yaw * error_world_x + sin_yaw * error_world_y
        target_y = -sin_yaw * error_world_x + cos_yaw * error_world_y

        distance_error = math.hypot(target_x, target_y)
        heading_error = math.atan2(target_y, target_x)

        cmd = Twist()

        if distance_error < self.position_tolerance:
            self.cmd_pub.publish(cmd)
            return

        if abs(heading_error) > 1.0:
            linear = 0.0
        else:
            linear = self.linear_gain * target_x

        if distance_error < self.position_tolerance * 2.0:
            angular = 0.0
        elif abs(heading_error) < self.heading_tolerance:
            angular = 0.0
        else:
            angular = self.angular_gain * heading_error

        linear = self.clamp(linear, -self.max_reverse_speed, self.max_linear_speed)
        angular = self.clamp(angular, -self.max_angular_speed, self.max_angular_speed)

        # EMA 低通滤波平滑，避免速度跳变
        sf = self.smoothing_factor
        linear = sf * linear + (1.0 - sf) * self.prev_linear
        angular = sf * angular + (1.0 - sf) * self.prev_angular
        self.prev_linear = linear
        self.prev_angular = angular

        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FollowerController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
