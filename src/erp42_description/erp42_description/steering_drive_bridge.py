import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class SteeringDriveBridge(Node):
    def __init__(self):
        super().__init__('steering_drive_bridge')

        # Parameters
        self.declare_parameter('wheel_base', 1.0)          # 앞뒤 차축 거리 (m)
        self.declare_parameter('track_width', 1.02)        # 좌우 트랙 너비 (m)
        self.declare_parameter('wheel_radius', 0.28)       # 휠 반지름 (m)
        self.declare_parameter('max_steer', math.radians(30.0))  # 최대 조향 각 (rad)
        self.declare_parameter('steer_time_constant', 0.2) # 조향 반응 시간 상수 (s)
        self.declare_parameter('speed_time_constant', 0.8) # 속도 반응 시간 상수 (s)
        self.declare_parameter('max_speed', 1.0)           # 최대 차량 선속도 (m/s)
        self.declare_parameter('decay_factor', 0.5)        # 입력이 0일 때 자연 감쇠 계수

        # Load parameters
        self.wheel_base = self.get_parameter('wheel_base').value
        self.track_width = self.get_parameter('track_width').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_steer = self.get_parameter('max_steer').value
        self.steer_tc = self.get_parameter('steer_time_constant').value
        self.speed_tc = self.get_parameter('speed_time_constant').value
        self.max_speed = self.get_parameter('max_speed').value
        self.decay_factor = self.get_parameter('decay_factor').value

        # Internal state
        self.target_steer = 0.0       # desired steering angle (rad)
        self.current_steer = 0.0      # filtered steering
        self.target_speed = 0.0       # desired forward speed (m/s)
        self.current_speed = 0.0      # filtered forward speed (m/s)

        # Publishers
        self.steer_pub = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.speed_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)

        # Subscriber
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Timer (control loop)
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.02, self.update_control)  # 50Hz


    def cmd_vel_callback(self, msg: Twist):
        # Cmd_vel.linear.x is forward speed in m/s, clamp
        desired_speed = max(-self.max_speed, min(self.max_speed, msg.linear.x))
        self.target_speed = desired_speed

        # Interpret angular.z as desired nominal steering angle (could be scaled)
        raw_steer = msg.angular.z
        # If user is giving small angular velocities, we can scale if needed
        # Here we assume angular.z is already in radians of desired turn angle
        self.target_steer = max(-self.max_steer, min(self.max_steer, raw_steer))

        

    def low_pass(self, current, target, dt, time_constant):
        if time_constant <= 0:
            return target
        alpha = dt / (time_constant + dt)
        return current + alpha * (target - current)

    def compute_ackermann(self, steer_angle: float):
        """
        Given a nominal steering angle, compute left/right hinge angles using Ackermann approximation.
        Positive steer_angle means turning left.
        """
        if abs(steer_angle) < 1e-4:
            return 0.0, 0.0

        R = self.wheel_base / math.tan(steer_angle)  # turning radius
        # inner / outer depending on direction
        if steer_angle > 0:  # left turn: left is inner
            inner = math.atan(self.wheel_base / (R - self.track_width / 2))
            outer = math.atan(self.wheel_base / (R + self.track_width / 2))
            left_angle = inner
            right_angle = outer
        else:  # right turn
            inner = math.atan(self.wheel_base / (R + self.track_width / 2))
            outer = math.atan(self.wheel_base / (R - self.track_width / 2))
            left_angle = outer
            right_angle = inner

        # clamp to max steer
        left_angle = max(-self.max_steer, min(self.max_steer, left_angle))
        right_angle = max(-self.max_steer, min(self.max_steer, right_angle))
        return left_angle, right_angle

    def update_control(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0:
            return
        self.last_time = now

        # Smooth steering and speed
        self.current_steer = self.low_pass(self.current_steer, self.target_steer, dt, self.steer_tc)
        self.current_speed = self.low_pass(self.current_speed, self.target_speed, dt, self.speed_tc)

        # If no forward command, apply gentle decay so wheels don't stop abruptly
        if abs(self.target_speed) < 1e-6 and abs(self.current_speed) > 1e-6:
            self.current_speed *= (1.0 - min(1.0, dt * self.decay_factor))

        # Compute ackermann left/right steering
        left_angle, right_angle = self.compute_ackermann(self.current_steer)

        # Convert forward speed (m/s) to wheel angular velocity (rad/s)
        if abs(self.wheel_radius) < 1e-6:
            wheel_omega = 0.0
        else:
            wheel_omega = self.current_speed / self.wheel_radius

        # Publish steering: [left, right]
        steer_msg = Float64MultiArray()
        steer_msg.data = [left_angle, right_angle]
        self.steer_pub.publish(steer_msg)

        # Publish wheel velocities: rear two wheels same omega
        speed_msg = Float64MultiArray()
        speed_msg.data = [wheel_omega, wheel_omega]
        self.speed_pub.publish(speed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SteeringDriveBridge()
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)  # 디버그 필요시
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()