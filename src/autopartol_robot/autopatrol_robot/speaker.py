import rclpy
from rclpy.node import Node
from autopatrol_interfaces.srv import SpeechText
import espeakng

class Speaker(Node):
    def __init__(self):
        super().__init__('speaker')
        self.speech_service_ = self.create_service(SpeechText,"speech_text",self.speech_text_callback)
        self.speaker_ = espeakng.Speaker()
        self.speaker_.voice = 'zh'

    def speech_text_callback(self,request,response):
        self.get_logger().info(f'正在准备朗读{request.text}')
        self.speaker_.say(request.text)
        self.speaker_.wait()
        response.result =  True
        return response
    
def main():
    rclpy.init()
    node = Speaker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('语音节点被用户中断')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()