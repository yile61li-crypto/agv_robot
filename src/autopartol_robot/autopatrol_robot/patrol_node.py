from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import rclpy.time
from autopatrol_interfaces.srv import SpeechText


class PatrolNode(BasicNavigator):
    def __init__(self, node_name='patrol_node'):
        super().__init__(node_name)
        self.declare_parameter('initial_point',[0.0, 0.0, 0.0])
        self.declare_parameter('target_points',[0.0, 0.0, 0.0, 1.0, 1.0, 1.57])
        self.initial_point_ = self.get_parameter('initial_point').value
        self.target_points_ = self.get_parameter('target_points').value
        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_,self)
        self.speech_client_ = self.create_client(SpeechText,'speech_text')


    def get_pose_by_xyzaw(self, x, y, yaw):
        """
        return PoseStamped对象
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        #返回顺序为 xyzw
        quat = quaternion_from_euler(0,0,yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        return pose


    def init_robot_pose(self):
        """
        初始化机器人的位姿
        """
        self.initial_point_ = self.get_parameter('initial_point').value
        init_pose = self.get_pose_by_xyzaw(self.initial_point_[0], self.initial_point_[1], self.initial_point_[2])
        self.setInitialPose(init_pose)
        self.waitUntilNav2Active()

    
    def get_target_points(self):
        """
        通过参数值获取目标点的集合
        """
        points = []
        self.target_points_ = self.get_parameter('target_points').value
        for index in range(int(len(self.target_points_)/3)):
            x = self.target_points_[index*3]
            y = self.target_points_[index*3+1]
            yaw = self.target_points_[index*3+2]
            points.append([x,y,yaw])
            self.get_logger().info(f'获得到目标点{index}->{x},{y},{yaw}')
        return points

    
    def nav_to_pose(self, target_point):
        """
        导航到目标点，返回是否成功
        """
        self.goToPose(target_point)
        while not self.isTaskComplete():
            feedback = self.getFeedback()
            if feedback:
                self.get_logger().info(f'剩余距离：{feedback.distance_remaining:.2f}')
        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('导航成功')
            return True
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('导航被取消')
            return False
        elif result == TaskResult.FAILED:
            self.get_logger().error('导航失败')
            return False
        else:
            self.get_logger().error(f'导航返回未知状态：{result}')
            return False

    
    def get_current_pose(self):
        """
        获取机器人当前的位置
        """
        while rclpy.ok():
            try:
                tf = self.buffer_.lookup_transform(
                    'map', 'base_footprint', rclpy.time.Time(seconds=0), rclpy.time.Duration(seconds=1))
                transform = tf.transform
                rotation_euler = euler_from_quaternion([
                    transform.rotation.x,
                    transform.rotation.y,
                    transform.rotation.z,
                    transform.rotation.w
                ])
                self.get_logger().info(
                    f'平移:{transform.translation},旋转四元数:{transform.rotation}:旋转欧拉角:{rotation_euler}')
                return transform
            except Exception as e:
                self.get_logger().warn(f'不能够获取坐标变换，原因: {str(e)}')

    def speech_text(self, text):
        """
        调用服务合成语音
        """
        while not self.speech_client_.wait_for_service(timeout_sec=1):
            self.get_logger().info('语音合成服务未上线，等待中...')
        
        request = SpeechText.Request()
        request.text = text
        future = self.speech_client_.call_async(request)
        rclpy.spin_until_future_complete(self,future)
        if future.result() is not None:
            response = future.result()
            if response.result == True:
                self.get_logger().info(f'语音合成成功{text}')
            else:
                self.get_logger().warn(f'语音合成失败{text}')
        else:
            self.get_logger().warn(f'语音合成服务响应失败')

def main():
    rclpy.init()
    patrol = PatrolNode()
    try:
        patrol.speech_text("正在准备初始化位置")
        patrol.init_robot_pose()
        patrol.speech_text("位置初始化完成")

        while rclpy.ok():
            points = patrol.get_target_points()
            for point in points:
                if not rclpy.ok():
                    break
                x, y, yaw = point[0], point[1], point[2]
                target_pose = patrol.get_pose_by_xyzaw(x, y, yaw)
                patrol.speech_text(f'正在准备前往{x},{y}目标点')
                success = patrol.nav_to_pose(target_pose)
                if success:
                    patrol.speech_text(f'已经到达目标点{x},{y}')
                else:
                    patrol.speech_text(f'前往{x},{y}目标点失败，跳过')
    except KeyboardInterrupt:
        patrol.get_logger().info('巡逻被用户中断')
    except Exception as e:
        patrol.get_logger().error(f'巡逻异常：{e}')
    finally:
        patrol.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
