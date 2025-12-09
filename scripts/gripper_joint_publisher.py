#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from dsr_msgs2.srv import SetModbusOutput
from std_msgs.msg import Int32

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        
        # Publisher for joint states
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.02, self.publish_joint_states)  # 50Hz
        
        # Gripper joint names
        self.joint_names = [
            'gripper_rh_r1',
            'gripper_rh_r2', 
            'gripper_rh_l1',
            'gripper_rh_l2'
        ]
        
        # Gripper position control
        # stroke: 0 = open, 700 = fully closed (real gripper)
        # joint angle: 0.0 = open, ~1.1 rad = closed
        self.stroke = 0  # Current stroke value (0~700)
        self.target_stroke = 0
        self.stroke_speed = 20  # stroke units per cycle
        
        # Stroke to joint angle conversion
        # stroke 700 → ~1.0 rad
        self.stroke_to_rad = 1.0 / 700.0
        
        # Services (간단한 열기/닫기)
        self.open_srv = self.create_service(Trigger, 'gripper/open', self.open_callback)
        self.close_srv = self.create_service(Trigger, 'gripper/close', self.close_callback)
        
        # Stroke 기반 서비스 (DART Platform 스타일)
        self.move_srv = self.create_service(SetModbusOutput, 'gripper/move', self.move_callback)
        
        # 간단한 토픽 기반 제어도 추가
        self.stroke_sub = self.create_subscription(Int32, 'gripper/stroke', self.stroke_callback, 10)
        
        self.get_logger().info('========================================')
        self.get_logger().info('Gripper Controller Ready!')
        self.get_logger().info('----------------------------------------')
        self.get_logger().info('Services:')
        self.get_logger().info('  gripper/open   - Open gripper')
        self.get_logger().info('  gripper/close  - Close gripper')
        self.get_logger().info('  gripper/move   - Move to stroke (0~700)')
        self.get_logger().info('Topic:')
        self.get_logger().info('  gripper/stroke - Int32 (0~700)')
        self.get_logger().info('========================================')
        
    def open_callback(self, request, response):
        self.target_stroke = 0
        self.get_logger().info('Gripper OPENING (stroke → 0)')
        response.success = True
        response.message = 'Gripper opening'
        return response
        
    def close_callback(self, request, response):
        self.target_stroke = 700
        self.get_logger().info('Gripper CLOSING (stroke → 700)')
        response.success = True
        response.message = 'Gripper closing'
        return response
    
    def move_callback(self, request, response):
        """DART Platform 스타일: gripper_move(stroke)"""
        # SetModbusOutput 메시지의 value 필드를 stroke로 사용
        stroke = request.value
        stroke = max(0, min(700, stroke))  # Clamp to 0~700
        self.target_stroke = stroke
        self.get_logger().info(f'Gripper moving to stroke: {stroke}')
        response.success = True
        return response
    
    def stroke_callback(self, msg):
        """토픽으로 stroke 값 수신"""
        stroke = max(0, min(700, msg.data))
        self.target_stroke = stroke
        self.get_logger().info(f'Gripper stroke set to: {stroke}')
    
    def publish_joint_states(self):
        # Smooth movement towards target
        if abs(self.stroke - self.target_stroke) > 1:
            if self.stroke < self.target_stroke:
                self.stroke = min(self.stroke + self.stroke_speed, self.target_stroke)
            else:
                self.stroke = max(self.stroke - self.stroke_speed, self.target_stroke)
        else:
            self.stroke = self.target_stroke
        
        # Convert stroke to joint angle
        joint_angle = self.stroke * self.stroke_to_rad
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [joint_angle] * 4
        msg.velocity = [0.0] * 4
        msg.effort = [0.0] * 4
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = GripperController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
