#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
import numpy as np
import struct
import cv2

def create_cloud_xyzrgb(header, points):
    fields = []
    for name, offset, datatype, count in [
        ('x', 0, PointField.FLOAT32, 1),
        ('y', 4, PointField.FLOAT32, 1),
        ('z', 8, PointField.FLOAT32, 1),
        ('rgb', 12, PointField.UINT32, 1),
    ]:
        pf = PointField()
        pf.name = name
        pf.offset = offset
        pf.datatype = datatype
        pf.count = count
        fields.append(pf)

    cloud = PointCloud2()
    cloud.header = header
    cloud.height = 1
    cloud.width = len(points)
    cloud.is_dense = False
    cloud.is_bigendian = False
    cloud.fields = fields
    cloud.point_step = 16
    cloud.row_step = cloud.point_step * cloud.width

    buffer = bytearray()
    for x, y, z, rgb in points:
        buffer.extend(struct.pack('ffff', x, y, z, rgb))
    cloud.data = bytes(buffer)
    return cloud

class RGBDProjectiveFusion(Node):
    def __init__(self):
        super().__init__('rgbd_projective_fusion')
        self.bridge = CvBridge()

        self.rgb_topic = '/middle_camera3/image_raw'
        self.depth_topic = '/middle_native_depth/depth/image_raw'
        self.rgb_info_topic = '/middle_camera3/camera_info'
        self.depth_info_topic = '/middle_native_depth/depth/camera_info'

        self.rgb_sub = Subscriber(self, Image, self.rgb_topic)
        self.depth_sub = Subscriber(self, Image, self.depth_topic)
        self.rgb_info_sub = Subscriber(self, CameraInfo, self.rgb_info_topic)
        self.depth_info_sub = Subscriber(self, CameraInfo, self.depth_info_topic)

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.rgb_info_sub, self.depth_info_sub],
            queue_size=10,
            slop=0.05
        )
        self.sync.registerCallback(self.callback)

        self.pub = self.create_publisher(PointCloud2, '/rgbd/pointcloud', 10)
        self.get_logger().info("Started RGBD projective fusion node (no sensor_msgs_py).")

        # X축 +90도 회전 행렬 (앞면이 아래를 보는 것을 바로잡기)
        self.R_correction = np.array([
            [1, 0, 0],
            [ 0, 1, 0],
            [ 0,  0,  1]
        ])


    def get_intrinsics_fallback(self, info, width, fov):
        K = np.array(info.k).reshape(3,3)
        if np.count_nonzero(K) == 0:
            fx = fy = width / (2 * np.tan(fov / 2))
            cx = cy = width / 2.0
            K = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0,  0,  1]])
            D = np.zeros(5)
        else:
            fx = K[0,0]; fy = K[1,1]
            cx = K[0,2]; cy = K[1,2]
            D = np.array(info.d)
        return K, D, fx, fy, cx, cy

    def callback(self, rgb_msg: Image, depth_msg: Image, rgb_info: CameraInfo, depth_info: CameraInfo):
        fov = 1.3962634
        K_d, D_d, fx_d, fy_d, cx_d, cy_d = self.get_intrinsics_fallback(depth_info, 320, fov)
        K_r, D_r, fx_r, fy_r, cx_r, cy_r = self.get_intrinsics_fallback(rgb_info, 800, fov)

        try:
            rgb_cv = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='rgb8')
            depth_cv = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        h_d, w_d = depth_cv.shape
        h_r, w_r, _ = rgb_cv.shape

        map1_d, map2_d = cv2.initUndistortRectifyMap(K_d, D_d, np.eye(3), K_d, (w_d, h_d), cv2.CV_32FC1)
        depth_undist = cv2.remap(depth_cv, map1_d, map2_d, interpolation=cv2.INTER_NEAREST)

        map1_r, map2_r = cv2.initUndistortRectifyMap(K_r, D_r, np.eye(3), K_r, (w_r, h_r), cv2.CV_32FC1)
        rgb_undist = cv2.remap(rgb_cv, map1_r, map2_r, interpolation=cv2.INTER_LINEAR)

        points = []
        step = 4

        for v in range(0, h_d, step):
            for u in range(0, w_d, step):
                z = depth_undist[v, u]
                if np.isnan(z) or z <= 0.0:
                    continue
                x = (u - cx_d) * z / fx_d
                y = (v - cy_d) * z / fy_d

                # 색을 뽑을 때는 회전 적용 전 좌표 사용
                point_3d = np.array([[x, y, z]], dtype=np.float32)
                rvec = np.zeros(3)
                tvec = np.zeros(3)
                image_pts, _ = cv2.projectPoints(point_3d, rvec, tvec, K_r, D_r)
                u_r, v_r = image_pts[0,0]
                u_r = int(round(u_r))
                v_r = int(round(v_r))
                if u_r < 0 or u_r >= w_r or v_r < 0 or v_r >= h_r:
                    continue

                r, g, b = rgb_undist[v_r, u_r]
                rgb_packed = struct.unpack('I', struct.pack('BBBB', int(b), int(g), int(r), 0))[0]

                # 출력은 X축 +90도 회전 적용
                pt = np.array([x, y, z])
                x_corr, y_corr, z_corr = self.R_correction @ pt

                points.append([x_corr, y_corr, z_corr, rgb_packed])

        if not points:
            self.get_logger().warning("no points to publish")
            return

        cloud = create_cloud_xyzrgb(rgb_msg.header, points)
        self.pub.publish(cloud)

def main(args=None):
    rclpy.init(args=args)
    node = RGBDProjectiveFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
