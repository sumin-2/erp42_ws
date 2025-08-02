#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

class SteeringDriveBridge(Node):
    def __init__(self):
        super().__init__('steering_drive_bridge')

        # 파라미터 선언
        self.declare_parameter('wheel_radius', 0.28)
        self.declare_parameter('steering_gain', 0.3)
        self.declare_parameter('max_steer', math.radians(30.0))   # 30°
        self.declare_parameter('steer_time_constant', 0.2)        # [s]

        # 파라미터 읽기
        self.wheel_radius    = self.get_parameter('wheel_radius').value
        self.steering_gain   = self.get_parameter('steering_gain').value
        self.max_steer       = self.get_parameter('max_steer').value
        self.time_constant   = self.get_parameter('steer_time_constant').value

        # 내부 상태 초기화
        self.target_steer    = 0.0
        self.current_steer   = 0.0
        self.target_speed    = 0.0    # << 여기 추가

        # 퍼블리셔
        self.steer_pub    = self.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', 10)
        self.velocity_pub = self.create_publisher(
            Float64MultiArray, '/velocity_controller/commands', 10)

        # cmd_vel 구독
        self.create_subscription(
            Twist, '/cmd_vel', self.twist_callback, 10)

        # 50Hz 업데이트 타이머
        update_hz = 50.0
        self.timer = self.create_timer(1.0/update_hz, self.update_control)

    def twist_callback(self, msg: Twist):
        # 목표 조향 계산 & 포화
        raw = self.steering_gain * msg.angular.z
        self.target_steer = max(-self.max_steer,
                                min(self.max_steer, raw))
        # 목표 속도 저장
        if abs(self.wheel_radius) < 1e-6:
            self.target_speed = 0.0
        else:
            self.target_speed = msg.linear.x / self.wheel_radius

    def update_control(self):
        # 시간 상수 기반 1차 지연
        dt = self.timer.timer_period_ns * 1e-9
        alpha = dt / self.time_constant
        if alpha > 1.0:
            alpha = 1.0
        self.current_steer += alpha * (self.target_steer - self.current_steer)

        # 조향 퍼블리시
        steer_msg = Float64MultiArray()
        steer_msg.data = [self.current_steer, self.current_steer]
        self.steer_pub.publish(steer_msg)

        # 속도 퍼블리시 (후륜 두 개)
        vel_msg = Float64MultiArray()
        vel_msg.data = [self.target_speed, self.target_speed]
        self.velocity_pub.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SteeringDriveBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
