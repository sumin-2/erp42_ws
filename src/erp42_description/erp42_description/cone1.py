
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from sensor_msgs.msg import LaserScan
import time

class CameraViewer(Node):
    def __init__(self):
        super().__init__('Image_sub')
        self.last_cone_time = time.time()
        self.stable_cone = None
        self.latest_scan = None  # 거리 데이터 저장용 변수

        self.bridge = CvBridge()
        self.K = None
        self.D = None
        self.lower_orange = np.array([5, 50, 50])
        self.upper_orange = np.array([25, 255, 255])

        self.fx = 476.7030836014194
        self.fy = 476.7030836014194
        self.width = 800
        self.height = 800

        fov_x = 2 * np.arctan(self.width / (2 * self.fx)) * 180 / np.pi
        fov_y = 2 * np.arctan(self.height / (2 * self.fy)) * 180 / np.pi
        self.fov_x=fov_x#이미지 좌로 40.25도, 우로 40.25도
        self.fov_y=fov_y
        self.left_deg = fov_x/2
        self.right_deg = fov_x/2

        self.sub_info = self.create_subscription(
            CameraInfo,
            '/skidbot/camera_sensor/camera_info',
            self.camera_info_callback,
            10
        )

        self.sub_image = self.create_subscription(
            Image,
            '/middle_camera3/image_raw',
            self.image_callback,
            10
        )
        self.sub_distance = self.create_subscription(
            LaserScan, 'skidbot/scan', self.sub_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def preprocess_image(self, msg):
        if not msg.encoding:
            self.get_logger().warn("encoding 정보 없음, 'rgb8'로 가정합니다.")
            encoding = 'rgb8'
        else:
            encoding = msg.encoding

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            return cv2.undistort(frame, self.K, self.D)
        except CvBridgeError as e:
            self.get_logger().error(f"cv_bridge 변환 실패: {e}")
            return np.zeros((800, 800, 3), dtype=np.uint8)
    
    def detect_cones(self,undistorted):
        hsv = cv2.cvtColor(undistorted, cv2.COLOR_BGR2HSV)
        # 마스크 생성
        mask = cv2.inRange(hsv, self.lower_orange, self.upper_orange)
        # 노이즈 제거 (선택)    
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        # 작은 점같은거 제거
        # 결과 표시
        #cv2.imshow("Orange Mask", mask)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # 외곽 윤곽선 추출하여 contours에 윤곽선들의 리스트로 저장
        cone_found = False  # 라바콘을 찾았는지 여부를 저장
        cx, cy = None, None
        angle_deg = None
        image_center_x = undistorted.shape[1] // 2
        best_cone = None 
        min_offset = float('inf')
        for cnt in contours:
            area = cv2.contourArea(cnt)# 하나의 윤곽선이 감싸는 면적을 계산하는 함수
            if area > 300:  # 너무 작은 잡음 제거
                x, y, w, h = cv2.boundingRect(cnt)
                # x:바운딩 박스의 왼쪽 위 꼭짓점 x좌표
                # y:바운딩 박스의 왼쪽 위 꼭짓점 y좌표
                # w:바운딩 박스의 너비 
                # h:바운딩 박스의 높이 
                aspect_ratio = h / float(w)
                if aspect_ratio > 0.4:   #가로로 긴 객체 필터링
                    cv2.rectangle(undistorted, (x, y), (x+w, y+h), (0, 255, 0), 2)#객체 외곽에 초록색 박스를 그림
                    cx = x + w//2
                    cy = y + h//2
                    cone_found = True
                    cv2.circle(undistorted, (cx, cy), 5, (255, 0, 0), -1)
                    cv2.putText(undistorted, f'({cx},{cy})', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0), 1)

                    offset = abs(cx - image_center_x)

                    # 중심에 가까운 것 중, 충분히 크면 후보
                    if offset < min_offset and h > 100:  # 가까이 있고 충분히 큼
                        min_offset = offset
                        best_cone = (cx, x, y, w, h)
                        angle_deg = np.arctan((cx - image_center_x) / self.fx) * 180 / np.pi
        if cone_found:
            self.get_logger().info(f"라바콘 중심: ({cx}, {cy})")
        else:
            self.get_logger().info("라바콘이 감지되지 않았습니다.")

        
        return best_cone, image_center_x, angle_deg    


    def compute_twist(self,best_cone,image_center_x,undistorted,angle_deg):
        twist = Twist()
        if angle_deg is None:
            angle_deg = 0.0  # 정중앙 가정
        best_pos = 360-angle_deg*4
        best_pos = int(best_pos)     
        if best_cone and self.latest_scan:
                cx, x, y, w, h = best_cone
                offset_x = cx - image_center_x

                if self.latest_scan.ranges[best_pos]<1.0:
                    # 회피 방향 결정
                    if abs(offset_x) < 50:
                        # 정중앙 → 기본 우측 회피
                        twist.linear.x = 0.2
                        twist.angular.z = -0.9
                        self.get_logger().info("center.")
                    elif offset_x > 0:
                        # 오른쪽 → 왼쪽 회피
                        twist.linear.x = 0.3
                        twist.angular.z = +0.6
                        self.get_logger().info("right.")
                    else:
                        # 왼쪽 → 오른쪽 회피
                        twist.linear.x = 0.3
                        twist.angular.z = -0.6
                        self.get_logger().info("left.")
                else:
                    twist.linear.x = 0.5
                    twist.angular.z = 0.0
                    self.get_logger().info("go.") 

                cv2.putText(undistorted, "Close CONE!", (x, y - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.rectangle(undistorted, (x, y), (x+w, y+h), (0, 0, 255), 3)  # 강조 사각형
        else:
                # 라바콘 없음 or 작음 → 직진
                twist.linear.x = 0.5
                twist.angular.z = 0.0
                self.get_logger().info("go.")   
        return twist    

    def sub_callback(self,msg):
        self.latest_scan = msg     


    def camera_info_callback(self, msg):
        self.K = np.array(msg.k).reshape((3, 3))
        self.D = np.array(msg.d)
        self.get_logger().info("CameraInfo 수신 완료. K, D 설정됨.")

    def image_callback(self, msg):
        if self.K is None or self.D is None:
            self.get_logger().warn("CameraInfo가 아직 수신되지 않음.")
            return

        # 왜곡 보정
        undistorted = self.preprocess_image(msg)

        best_cone, image_center_x,angle_deg= self.detect_cones(undistorted)
        twist = self.compute_twist(best_cone, image_center_x, undistorted,angle_deg)
        self.cmd_pub.publish(twist) 
    
        # OpenCV로 출력
        #cv2.imshow("Original", frame)
        
        try:
            cv2.imshow("Undistorted", undistorted)
            cv2.waitKey(1)
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        twist=Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        node.cmd_pub.publish(twist) 
        print("\n[INFO] Ctrl+C 입력됨. 노드 종료 중...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        print("[INFO] 노드와 OpenCV 창 정상 종료됨.")

if __name__ == '__main__':
    main()
