import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class FrontCameraViewer(Node):
    def __init__(self):
        super().__init__('front_camera_viewer')

        # Camera topic: middle_camera3 is assumed as front camera
        self.subscription = self.create_subscription(
            Image,
            '/middle_camera3/image_raw',  # front camera topic
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info("Subscribed to /middle_camera3/image_raw")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Front Camera View", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = FrontCameraViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()