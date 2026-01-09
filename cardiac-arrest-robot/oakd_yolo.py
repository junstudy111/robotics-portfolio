#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge
from ultralytics import YOLO
from std_msgs.msg import Bool, Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np

class DualYoloNode(Node):
    def __init__(self):
        super().__init__('dual_yolo_node')
        self.bridge = CvBridge()
        self.model = YOLO('/home/jun/rokey_ws/src/rescue/model/best.pt')
        
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)

        self.current_role_is_true = None 
        
        self.data = {
            'robot3': {'rgb': None, 'depth': None, 'rgb_K': None, 'stereo_K': None},
            'robot5': {'rgb': None, 'depth': None, 'rgb_K': None, 'stereo_K': None}
        }

        # ================= PUBLISHERS =================
        self.pub_aed = self.create_publisher(Bool, '/robotA/aed_detected', 10)
        self.pub_crowd_a = self.create_publisher(Bool, '/robotA/crowd_detected', 10)
        self.pub_patient_dist = self.create_publisher(Float32, '/robotA/patient/distance', 10)

        self.pub_responder_done = self.create_publisher(Bool, '/robotB/responder_done', 10)
        self.pub_crowd_b = self.create_publisher(Bool, '/robotB/crowd_detected', 10)

        # ================= SUBSCRIBERS =================
        for ns in ['robot3', 'robot5']:
            self.create_subscription(CompressedImage, f'/{ns}/oakd/rgb/image_raw/compressed', 
                                     lambda msg, n=ns: self.rgb_cb(msg, n), qos)
            self.create_subscription(Image, f'/{ns}/oakd/stereo/image_raw', 
                                     lambda msg, n=ns: self.depth_cb(msg, n), qos)
            # ✅ Camera Info 구독 추가
            self.create_subscription(CameraInfo, f'/{ns}/oakd/rgb/camera_info',
                                     lambda msg, n=ns: self.rgb_info_cb(msg, n), 10)
            self.create_subscription(CameraInfo, f'/{ns}/oakd/stereo/camera_info',
                                     lambda msg, n=ns: self.stereo_info_cb(msg, n), 10)

        self.create_subscription(Bool, '/robot_role', self.role_callback, 10)
        self.create_timer(0.1, self.run_yolo_dual)

    def role_callback(self, msg: Bool):
        self.current_role_is_true = msg.data
        if self.current_role_is_true:
            self.get_logger().info("🔄 [Role] True 수신: Robot3=A(AED), Robot5=B(EMT)")
        else:
            self.get_logger().info("🔄 [Role] False 수신: Robot3=B(EMT), Robot5=A(AED)")

    def rgb_cb(self, msg, robot_id):
        # ✅ CompressedImage 디코딩 (DebugObjectGoal 방식)
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is not None:
            self.data[robot_id]['rgb'] = img
            
            # 디버깅 로그
            if not hasattr(self, f'_rgb_logged_{robot_id}'):
                self.get_logger().info(f'✅ {robot_id} RGB image received: {img.shape}')
                setattr(self, f'_rgb_logged_{robot_id}', True)

    def depth_cb(self, msg, robot_id):
        depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        self.data[robot_id]['depth'] = depth
        
        # ✅ 처음 한 번만 출력
        if not hasattr(self, f'_depth_info_logged_{robot_id}'):
            self.get_logger().info(f'🔍 {robot_id} Depth dtype: {depth.dtype}, shape: {depth.shape}, sample value: {depth[depth > 0][0] if len(depth[depth > 0]) > 0 else 0}')
            setattr(self, f'_depth_info_logged_{robot_id}', True)

    def rgb_info_cb(self, msg, robot_id):
        if self.data[robot_id]['rgb_K'] is None:
            self.data[robot_id]['rgb_K'] = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(f'✓ {robot_id} RGB camera info received')

    def stereo_info_cb(self, msg, robot_id):
        if self.data[robot_id]['stereo_K'] is None:
            self.data[robot_id]['stereo_K'] = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(f'✓ {robot_id} Stereo camera info received')

    def transform_rgb_to_stereo(self, x_rgb, y_rgb, rgb_K, stereo_K, depth_estimate=1.0):
        """RGB 좌표를 Stereo 좌표로 변환"""
        if rgb_K is None or stereo_K is None:
            return None, None

        fx_rgb, fy_rgb = rgb_K[0, 0], rgb_K[1, 1]
        cx_rgb, cy_rgb = rgb_K[0, 2], rgb_K[1, 2]
        fx_stereo, fy_stereo = stereo_K[0, 0], stereo_K[1, 1]
        cx_stereo, cy_stereo = stereo_K[0, 2], stereo_K[1, 2]

        x_norm = (x_rgb - cx_rgb) / fx_rgb
        y_norm = (y_rgb - cy_rgb) / fy_rgb

        X = x_norm * depth_estimate
        Y = y_norm * depth_estimate
        Z = depth_estimate

        x_stereo = (X / Z) * fx_stereo + cx_stereo
        y_stereo = (Y / Z) * fy_stereo + cy_stereo

        return int(x_stereo), int(y_stereo)

    def get_depth(self, u, v, depth_img, rgb_K, stereo_K):
        """✅ 개선된 depth 값 추출 (median filter 적용)"""
        if depth_img is None or rgb_K is None or stereo_K is None:
            return None

        x_s, y_s = self.transform_rgb_to_stereo(u, v, rgb_K, stereo_K, 1.0)
        if x_s is None:
            return None

        h, w = depth_img.shape[:2]
        if not (0 <= y_s < h and 0 <= x_s < w):
            return None

        half = 2
        y1, y2 = max(0, y_s - half), min(h, y_s + half + 1)
        x1, x2 = max(0, x_s - half), min(w, x_s + half + 1)

        region = depth_img[y1:y2, x1:x2]
        valid = region[region > 0]

        if len(valid) == 0:
            return None

        z = np.median(valid)
        
        # ✅ 타입별 변환
        if depth_img.dtype == np.uint16:
            # uint16: mm 단위 -> m로 변환
            z = z / 1000.0
        elif depth_img.dtype == np.float32 or depth_img.dtype == np.float64:
            # float: 이미 m 단위일 가능성 높음
            # 만약 20m가 나온다면 실제로는 mm 단위일 수 있음
            if z > 10.0:  # 10m 이상이면 mm로 판단
                z = z / 1000.0
        
        # ✅ 유효 범위 체크 (OAK-D 스테레오는 보통 0.2m ~ 10m)
        if z < 0.1 or z > 15.0:
            return None
        
        return z

    def run_yolo_dual(self):
        if self.current_role_is_true is None:
            return

        for robot_id in ['robot3', 'robot5']:
            rb_data = self.data[robot_id]
            if rb_data['rgb'] is None or rb_data['depth'] is None:
                continue

            # Camera info 대기
            if rb_data['rgb_K'] is None or rb_data['stereo_K'] is None:
                continue

            if self.current_role_is_true:
                is_currently_a = (robot_id == 'robot3')
            else:
                is_currently_a = (robot_id == 'robot5')

            results = self.model(rb_data['rgb'], conf=0.4, device=0, half=True, verbose=False)[0]
            
            aed_found = False
            responder_found = False
            crowd_count = 0
            crowd_close_count = 0  # ✅ 1.2m 이내 군중 카운트
            debug_frame = rb_data['rgb'].copy()

            for box in results.boxes:
                cls_name = self.model.names[int(box.cls[0])]
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                
                # ✅ 개선된 depth 추출
                Z = self.get_depth(cx, cy, rb_data['depth'], rb_data['rgb_K'], rb_data['stereo_K'])
                
                if Z is None:
                    Z = 0.0

                color = (255, 0, 0) if is_currently_a else (0, 255, 255)
                cv2.rectangle(debug_frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(debug_frame, f"{cls_name} {Z:.2f}m", (x1, y1-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                # ✅ Crowd 감지 로직 개선
                if cls_name == 'Crowd':
                    crowd_count += 1
                    if Z > 0.0 and Z <= 1.2:  # 👈 1.2m 이내만 카운트
                        crowd_close_count += 1
                        # 가까운 군중은 빨간색으로 강조
                        cv2.rectangle(debug_frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
                        cv2.putText(debug_frame, "CLOSE!", (x1, y2+20), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
                if is_currently_a:
                    if cls_name == 'AED': 
                        aed_found = True
                    elif cls_name == 'Patient' and Z > 0.0:
                        self.pub_patient_dist.publish(Float32(data=float(Z)))
                        self.get_logger().info(f'📏 Patient distance: {Z:.2f}m')
                else:
                    if cls_name == 'Responder': 
                        responder_found = True

            # ✅ 1.2m 이내 군중이 2명 이상일 때만 True
            crowd_status = (crowd_close_count >= 2)

            if is_currently_a:
                self.pub_aed.publish(Bool(data=aed_found))
                self.pub_crowd_a.publish(Bool(data=crowd_status))
                if crowd_status:
                    self.get_logger().warn(f'🚨 {robot_id}: {crowd_close_count}명의 군중이 1.2m 이내!')
            else:
                self.pub_responder_done.publish(Bool(data=responder_found))
                self.pub_crowd_b.publish(Bool(data=crowd_status))
                if crowd_status:
                    self.get_logger().warn(f'🚨 {robot_id}: {crowd_close_count}명의 군중이 1.2m 이내!')

            role_text = "Robot A (AED)" if is_currently_a else "Robot B (EMT)"
            cv2.putText(debug_frame, f"{robot_id}: {role_text}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # ✅ 군중 정보 표시
            cv2.putText(debug_frame, f"Crowd total: {crowd_count}, Close(<1.2): {crowd_close_count}", 
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            cv2.imshow(f"Monitoring {robot_id}", debug_frame)
            
        cv2.waitKey(1)
def main():
    rclpy.init()
    rclpy.spin(DualYoloNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()