from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import GetEntityState
import rclpy
from rclpy.node import Node


class EntityPosePublisher(Node):
    def __init__(self):
        super().__init__('entity_pose_publisher')

        self.declare_parameter('entity_name', 'main_robot')
        self.declare_parameter('entity_state_service', '/gazebo_state/get_entity_state')
        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('pose_topic', '/main_robot/follow_target_pose')
        self.declare_parameter('publish_rate', 15.0)

        self.entity_name = self.get_parameter('entity_name').value
        self.entity_state_service = self.get_parameter('entity_state_service').value
        self.reference_frame = self.get_parameter('reference_frame').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.pose_pub = self.create_publisher(
            PoseStamped,
            self.get_parameter('pose_topic').value,
            10,
        )
        self.entity_state_client = self.create_client(GetEntityState, self.entity_state_service)
        self.pending_future = None
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

    def timer_callback(self):
        if not self.entity_state_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().warn(
                f'Waiting for entity state service {self.entity_state_service}',
                throttle_duration_sec=5.0,
            )
            return

        if self.pending_future is not None and not self.pending_future.done():
            return

        request = GetEntityState.Request()
        request.name = self.entity_name
        request.reference_frame = self.reference_frame
        self.pending_future = self.entity_state_client.call_async(request)
        self.pending_future.add_done_callback(self.handle_entity_state)

    def handle_entity_state(self, future):
        self.pending_future = None
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=5.0)
            return

        if not response.success:
            self.get_logger().warn(
                f'GetEntityState failed for {self.entity_name}: {response.status_message}',
                throttle_duration_sec=5.0,
            )
            return

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.reference_frame
        pose_msg.pose = response.state.pose
        self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EntityPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
