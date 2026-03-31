import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import math
import tf2_ros
from sensor_msgs.msg import LaserScan
from message_filters import ApproximateTimeSynchronizer, Subscriber


class LaserMerger(Node):
    def __init__(self):
        super().__init__('laser_merger_node')

        # === 声明参数 ===
        self.declare_parameter('front_left_topic', 'scan_front_left')
        self.declare_parameter('back_right_topic', 'scan_back_right')
        self.declare_parameter('output_topic', 'scan')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('car_length', 0.30)
        self.declare_parameter('car_width', 0.20)
        self.declare_parameter('body_filter_margin_x', 0.0)
        self.declare_parameter('body_filter_margin_y', 0.0)
        self.declare_parameter('output_angle_min_deg', -180.0)
        self.declare_parameter('output_angle_max_deg', 180.0)
        self.declare_parameter('output_angle_increment_deg', 1.0)
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 10.0)
        self.declare_parameter('tf_timeout_sec', 0.1)
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('slop_sec', 0.05)
        self.declare_parameter('scan_frequency_hz', 10.0)

        # === 读取参数并转换为弧度（保存为成员变量）===
        front_left_topic = self.get_parameter('front_left_topic').value
        back_right_topic = self.get_parameter('back_right_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.base_frame = self.resolve_frame_name(self.get_parameter('base_frame').value)
        self.car_length = self.get_parameter('car_length').value
        self.car_width = self.get_parameter('car_width').value
        self.body_filter_margin_x = self.get_parameter('body_filter_margin_x').value
        self.body_filter_margin_y = self.get_parameter('body_filter_margin_y').value
        
        # 保存为 self. 成员变量，供 merge_scans 使用
        self.angle_min_rad = math.radians(self.get_parameter('output_angle_min_deg').value)
        self.angle_max_rad = math.radians(self.get_parameter('output_angle_max_deg').value)
        self.angle_increment = math.radians(self.get_parameter('output_angle_increment_deg').value)
        
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.tf_timeout = Duration(seconds=self.get_parameter('tf_timeout_sec').value)
        queue_size = self.get_parameter('queue_size').value
        slop = self.get_parameter('slop_sec').value
        self.scan_freq = self.get_parameter('scan_frequency_hz').value

        # === TF Buffer ===
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # === 订阅器（时间同步）===
        front_sub = Subscriber(self, LaserScan, front_left_topic)
        back_sub = Subscriber(self, LaserScan, back_right_topic)

        self.ts = ApproximateTimeSynchronizer(
            [front_sub, back_sub],
            queue_size=queue_size,
            slop=slop,
            allow_headerless=False
        )
        self.ts.registerCallback(self.synced_scan_callback)

        # === 发布器 ===
        self.merged_scan_pub = self.create_publisher(LaserScan, output_topic, 10)


        self.get_logger().info(
            f"Laser merger node started!\n"
            f"  Input: {front_left_topic}, {back_right_topic}\n"
            f"  Output: {output_topic} (frame: {self.base_frame})\n"
            f"  Angle range: {self.get_parameter('output_angle_min_deg').value:.1f}° ~ "
            f"{self.get_parameter('output_angle_max_deg').value:.1f}°\n"
            f"  Car size: {self.car_length:.2f}m x {self.car_width:.2f}m\n"
            f"  Body filter margin: x={self.body_filter_margin_x:.3f}m, y={self.body_filter_margin_y:.3f}m"
        )


    def resolve_frame_name(self, frame_name):
        if not frame_name:
            return frame_name
        frame_name = frame_name.lstrip('/')
        namespace = self.get_namespace().strip('/')
        if not namespace or '/' in frame_name:
            return frame_name
        return f"{namespace}/{frame_name}"

    def is_self_occlusion(self, x, y):
        half_length = self.car_length / 2 + self.body_filter_margin_x
        half_width = self.car_width / 2 + self.body_filter_margin_y
        return abs(x) < half_length and abs(y) < half_width

    def _extract_2d_transform(self, transform_stamped):
        """从 TransformStamped 中提取 2D 变换参数 (tx, ty, cos_yaw, sin_yaw)"""
        t = transform_stamped.transform
        tx = t.translation.x
        ty = t.translation.y
        qz = t.rotation.z
        qw = t.rotation.w
        yaw = math.atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz))
        return tx, ty, math.cos(yaw), math.sin(yaw)

    def transform_scan_to_base(self, scan_msg):
        angles = []
        ranges = []

        if not scan_msg.ranges:
            return angles, ranges

        try:
            trans = self.tf_buffer.lookup_transform(
                self.base_frame,
                scan_msg.header.frame_id,
                scan_msg.header.stamp,
                timeout=self.tf_timeout
            )
        except tf2_ros.TransformException as e:
            self.get_logger().warn(
                f"TF lookup failed from '{scan_msg.header.frame_id}' to '{self.base_frame}': {str(e)[:80]}",
                throttle_duration_sec=5.0
            )
            return angles, ranges

        # 一次提取变换矩阵参数，避免逐点调用 do_transform_point
        tx, ty, cos_yaw, sin_yaw = self._extract_2d_transform(trans)
        half_length = self.car_length / 2 + self.body_filter_margin_x
        half_width = self.car_width / 2 + self.body_filter_margin_y

        angle = scan_msg.angle_min
        cos_inc = math.cos(scan_msg.angle_increment)
        sin_inc = math.sin(scan_msg.angle_increment)
        current_cos = math.cos(angle)
        current_sin = math.sin(angle)
        r_min = scan_msg.range_min
        r_max = scan_msg.range_max

        for r in scan_msg.ranges:
            if math.isinf(r) or math.isnan(r) or r < r_min or r > r_max:
                new_cos = current_cos * cos_inc - current_sin * sin_inc
                new_sin = current_sin * cos_inc + current_cos * sin_inc
                current_cos, current_sin = new_cos, new_sin
                continue

            # 局部坐标
            x_local = r * current_cos
            y_local = r * current_sin

            # 手动 2D 刚体变换: R * p + t
            x_base = cos_yaw * x_local - sin_yaw * y_local + tx
            y_base = sin_yaw * x_local + cos_yaw * y_local + ty

            # 内联自遮挡过滤（避免函数调用开销）
            if not (abs(x_base) < half_length and abs(y_base) < half_width):
                r_merged = math.hypot(x_base, y_base)
                angle_merged = math.atan2(y_base, x_base)
                angles.append(angle_merged)
                ranges.append(r_merged)

            new_cos = current_cos * cos_inc - current_sin * sin_inc
            new_sin = current_sin * cos_inc + current_cos * sin_inc
            current_cos, current_sin = new_cos, new_sin

        return angles, ranges

    def merge_scans(self, front_scan, back_scan):
        fl_angles, fl_ranges = self.transform_scan_to_base(front_scan)
        br_angles, br_ranges = self.transform_scan_to_base(back_scan)

        all_angles = fl_angles + br_angles
        all_ranges = fl_ranges + br_ranges

        if not all_angles:
            return

        merged_scan = LaserScan()
        merged_scan.header.frame_id = self.base_frame
        merged_scan.header.stamp = self.get_clock().now().to_msg()
        
        # ✅ 使用参数化的角度范围（不再硬编码！）
        merged_scan.angle_min = self.angle_min_rad
        merged_scan.angle_max = self.angle_max_rad
        merged_scan.angle_increment = self.angle_increment
        merged_scan.range_min = self.range_min
        merged_scan.range_max = self.range_max

        # ✅ 补全缺失字段
        if self.scan_freq > 0:
            merged_scan.time_increment = self.angle_increment / (2 * math.pi * self.scan_freq)
            merged_scan.scan_time = 1.0 / self.scan_freq
        else:
            merged_scan.time_increment = 0.0
            merged_scan.scan_time = 0.0
        merged_scan.intensities = []  # 若需支持强度，可扩展

        num_readings = int((merged_scan.angle_max - merged_scan.angle_min) / merged_scan.angle_increment) + 1
        bin_dict = {}

        for angle, r in zip(all_angles, all_ranges):
            # 角度归一化到 [angle_min, angle_max]
            angle = math.fmod(angle - self.angle_min_rad, 2 * math.pi) + self.angle_min_rad
            if angle < self.angle_min_rad:
                angle += 2 * math.pi
            if angle > self.angle_max_rad:
                angle -= 2 * math.pi

            idx = int(round((angle - merged_scan.angle_min) / merged_scan.angle_increment))
            idx = max(0, min(idx, num_readings - 1))

            if idx not in bin_dict or r < bin_dict[idx]:
                bin_dict[idx] = r

        merged_scan.ranges = [float('inf')] * num_readings
        for idx, r in bin_dict.items():
            merged_scan.ranges[idx] = r

        self.merged_scan_pub.publish(merged_scan)

    def synced_scan_callback(self, front_msg, back_msg):
        self.merge_scans(front_msg, back_msg)



def main(args=None):
    rclpy.init(args=args)
    node = LaserMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down laser merger node...")
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {e}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
