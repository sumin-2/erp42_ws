import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class SteeringDriveBridge(Node):
    def __init__(self):
        super().__init__('steering_drive_bridge')
        # 선언 (중복 문제가 있었던 use_sim_time은 선언하지 않음)
        self.declare_parameter('wheel_radius', 0.28)
        self.declare_parameter('steering_gain', 0.3)
        self.declare_parameter('max_steer', 0.52)

        # 값 읽기 (기본값이 선언돼 있으므로 get_parameter로 안전)
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.steering_gain = self.get_parameter('steering_gain').value
        self.max_steer = self.get_parameter('max_steer').value

        # Publishers/subscriptions
        self.steer_pub = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.velocity_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)

    def twist_callback(self, msg: Twist):
        target_steer = self.steering_gain * msg.angular.z
        # 포화
        if target_steer > self.max_steer:
            target_steer = self.max_steer
        elif target_steer < -self.max_steer:
            target_steer = -self.max_steer

        steer_array = Float64MultiArray()
        steer_array.data = [target_steer, target_steer]
        self.steer_pub.publish(steer_array)

        wheel_speed = 0.0
        if self.wheel_radius != 0:
            wheel_speed = msg.linear.x / self.wheel_radius

        vel_array = Float64MultiArray()
        vel_array.data = [wheel_speed, wheel_speed, wheel_speed, wheel_speed]
        self.velocity_pub.publish(vel_array)


def main(args=None):
    rclpy.init(args=args)
    node = SteeringDriveBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()