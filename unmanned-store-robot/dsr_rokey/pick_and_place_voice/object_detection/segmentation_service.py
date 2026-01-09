# object_detection/segmentation_service.py
# (FINAL VERSION: Uses pyzbar to fix QUIRC error)

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from od_msg.srv import SrvDepthPosition
import numpy as np
import json
import time
import os
import cv2

# ⭐ pyzbar 라이브러리 사용 (OpenCV 에러 해결용)
from pyzbar.pyzbar import decode, ZBarSymbol

from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

from object_detection.realsense import ImgNode
from robot_control.module.seg_pick_config import POSITION_CONFIGS
from robot_control.module.seg_pick_utils import (
    get_segmentation_center,
    get_angle_from_segmentation,
    calculate_angle_offset
)

import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"


class SegmentationService(Node):
    def __init__(self):
        super().__init__('segmentation_service')
        
        DR_init.__dsr__node = self
        
        package_path = get_package_share_directory("pick_and_place_voice")
        
        # ================================================================
        # 1. RealSense 초기화
        # ================================================================
        self.get_logger().info("Initializing RealSense camera...")
        self.img_node = ImgNode()
        rclpy.spin_once(self.img_node)
        time.sleep(1)
        
        self.intrinsics = self.img_node.get_camera_intrinsic()
        
        if self.intrinsics is None:
            self.get_logger().error("❌ Failed to get camera intrinsics!")
            raise RuntimeError("Camera intrinsics unavailable")
        
        self.get_logger().info(f"✓ Camera intrinsics loaded: {self.intrinsics}")
        
        # ================================================================
        # 2. YOLO 모델 로드 (Segmentation)
        # ================================================================
        model_path = os.path.join(package_path, "resource", "yolov11_seg_best_sho.pt")
        if not os.path.exists(model_path):
            raise RuntimeError(f"YOLO model missing: {model_path}")
        
        self.model = YOLO(model_path)
        self.get_logger().info("✓ YOLO model loaded")
        
        # Class Names 로드
        json_path = os.path.join(package_path, "resource", "class_name_tool.json")
        with open(json_path, "r") as f:
            self.class_names = json.load(f)
            
        self.seg_confidence = 0.6
        self.latest_results = None

        # ================================================================
        # 3. QR Detector (pyzbar 사용)
        # ================================================================
        self.get_logger().info("✓ QR Detector (pyzbar) ready")

        # ================================================================
        # 4. 서비스 생성
        # ================================================================
        self.srv = self.create_service(
            SrvDepthPosition,
            '/segmentation_pick',
            self.handle_service_request
        )

        self.get_logger().info("="*60)
        self.get_logger().info("✅ Integrated Service Ready")
        self.get_logger().info("   • Segmentation: Buffer Flush + Center Priority")
        self.get_logger().info("   • QR Detection: Using pyzbar (Fixes QUIRC error)")
        self.get_logger().info("="*60)

    # ======================================================================
    # 통합 요청 핸들러
    # ======================================================================
    def handle_service_request(self, request, response):
        target = request.target.strip()
        
        try:
            # [CASE 1] QR 요청 ("QR")
            if target.upper() == "QR":
                return self.process_qr_request(response)

            # [CASE 2] Segmentation 요청
            if ':' in target:
                target_class, position_str = target.split(':')
                position_num = int(position_str)
            else:
                target_class = target
                position_num = self._get_position_from_class(target_class)
            
            self.get_logger().info(f"\n[SEG] Request: class='{target_class}', pos={position_num}")

            result = self.run_segmentation(target_class, position_num)
            
            if result:
                response.depth_position = [
                    float(result['pixel_x']),
                    float(result['pixel_y']),
                    float(result['depth']),
                    float(result['angle'])
                ]
                response.shoe_name = "" 
                self.get_logger().info(f"[SEG] Success: {response.depth_position}")
            else:
                response.depth_position = [0.0, 0.0, 0.0, 0.0]
                self.get_logger().warn("[SEG] Failed to detect object")

            return response

        except Exception as e:
            self.get_logger().error(f"[SERVICE ERROR] {e}")
            response.depth_position = [0.0, 0.0, 0.0, 0.0]
            return response

    # ======================================================================
    # [Logic 1] Segmentation
    # ======================================================================
    def run_segmentation(self, target_class, position_num):
        self.latest_results = None
        success = False
        max_attempts = 5
        
        for attempt in range(max_attempts):
            for _ in range(8): # Buffer flush
                rclpy.spin_once(self.img_node)
                time.sleep(0.02) 
            
            color_img = self.img_node.get_color_frame()
            if color_img is None:
                time.sleep(0.1)
                continue

            try:
                self.latest_results = self.model.predict(
                    color_img, conf=self.seg_confidence, verbose=False
                )
            except Exception:
                continue
            
            if self.latest_results[0].masks is not None and len(self.latest_results[0].masks) > 0:
                success = True
                break
            
            time.sleep(0.2)

        if not success:
            return None

        target = self.find_target_object(target_class)
        if target is None:
            return None

        config = POSITION_CONFIGS.get(position_num, {})
        center_x, center_y = target['center']
        contour = target['contour']

        if config.get("use_fixed_depth", False):
            pick_depth = config["fixed_depth"]
        else:
            rclpy.spin_once(self.img_node)
            depth_frame = self.img_node.get_depth_frame()
            if depth_frame is None: 
                return None
            try:
                pick_depth = depth_frame[center_y, center_x]
                if pick_depth == 0: return None
            except:
                return None

        raw_angle = get_angle_from_segmentation(contour)
        angle_offset = calculate_angle_offset(raw_angle, position_num)

        return {
            'pixel_x': center_x,
            'pixel_y': center_y,
            'depth': pick_depth,
            'angle': angle_offset
        }

    def find_target_object(self, target_class):
        if self.latest_results is None: return None
        
        img_h, img_w = self.latest_results[0].orig_shape
        img_cx, img_cy = img_w / 2, img_h / 2
        
        boxes = self.latest_results[0].boxes
        masks = self.latest_results[0].masks
        target_lower = target_class.lower()
        candidates = []

        for i, box in enumerate(boxes):
            cls_id = int(box.cls[0])
            cls_name = self.class_names.get(str(cls_id), "unknown").lower()
            if cls_name != target_lower: continue

            mask_contour = masks.xy[i]
            center = get_segmentation_center(mask_contour)
            if center:
                cx, cy = center
                dist_sq = (cx - img_cx)**2 + (cy - img_cy)**2
                candidates.append({
                    'center': center,
                    'contour': mask_contour,
                    'dist_sq': dist_sq,
                    'conf': float(box.conf[0])
                })

        if not candidates: return None
        candidates.sort(key=lambda c: (c['dist_sq'], -c['conf']))
        return candidates[0]

    # ======================================================================
    # [Logic 2] QR Detection (pyzbar 사용 - QUIRC 에러 해결)
    # ======================================================================
    def process_qr_request(self, response):
        self.get_logger().info("[QR] Starting Detection (pyzbar)...")
        qr_info = None
        max_qr_attempts = 10
        last_frame = None

        for attempt in range(max_qr_attempts):
            rclpy.spin_once(self.img_node)
            time.sleep(0.05)
            
            frame = self.img_node.get_color_frame()
            if frame is None: continue
            
            last_frame = frame.copy()

            # 1. Grayscale 변환 (필수)
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # 2. pyzbar로 디코딩 (cv2.QRCodeDetector 대신 사용)
            decoded_objects = decode(gray_frame, symbols=[ZBarSymbol.QRCODE])

            if decoded_objects:
                obj = decoded_objects[0] # 첫 번째 QR
                
                # 데이터 디코딩
                text = obj.data.decode('utf-8')
                
                # 중심점 계산
                rect = obj.rect
                cx = int(rect.left + rect.width / 2)
                cy = int(rect.top + rect.height / 2)
                
                # 각도 계산 (간단히 0도 처리, 박스 피킹엔 영향 적음)
                angle = 0.0
                if len(obj.polygon) >= 2:
                    p0, p1 = obj.polygon[0], obj.polygon[1]
                    angle = np.degrees(np.arctan2(p1.y - p0.y, p1.x - p0.x))

                qr_info = {'text': text, 'center': (cx, cy), 'angle': angle}
                self.get_logger().info(f"[QR] Found: {text}")
                break
        
        # QR 못 찾았을 때
        if qr_info is None:
            self.get_logger().warn("[QR] Not found")
            
            # 실패한 이미지 저장 (디버깅용)
            try:
                if last_frame is not None:
                    debug_path = "/home/rokey/qr_failed_debug.jpg"
                    cv2.imwrite(debug_path, last_frame)
                    self.get_logger().warn(f"⚠️ Check this image: {debug_path}")
            except:
                pass

            response.depth_position = [0.0, 0.0, 0.0, 0.0]
            response.shoe_name = "NO_QR"
            return response

        # 3D 좌표 변환
        cx, cy = qr_info['center']
        angle = qr_info['angle']
        label = qr_info['text']

        depth_frame = self.img_node.get_depth_frame()
        z_depth = 0.0
        
        if depth_frame is not None:
            try:
                z_depth = depth_frame[cy, cx]
            except:
                z_depth = 0.0

        if z_depth > 0:
            cam_x, cam_y, cam_z = self._pixel_to_camera_coords(cx, cy, z_depth)
            response.depth_position = [float(cam_x), float(cam_y), float(cam_z), float(angle)]
            response.shoe_name = label
            self.get_logger().info(f"[QR] Result (3D): x={cam_x:.1f}, y={cam_y:.1f}, z={cam_z:.1f}")
        else:
            response.depth_position = [0.0, 0.0, 0.0, float(angle)]
            response.shoe_name = label
            self.get_logger().warn("[QR] Found but depth is 0")

        return response

    def _pixel_to_camera_coords(self, x, y, z):
        fx = self.intrinsics['fx']
        fy = self.intrinsics['fy']
        ppx = self.intrinsics['ppx']
        ppy = self.intrinsics['ppy']

        cam_x = (x - ppx) * z / fx
        cam_y = (y - ppy) * z / fy
        cam_z = z
        return cam_x, cam_y, cam_z

    def _get_position_from_class(self, cls):
        cls = cls.lower()
        mapping = {"nike": 1, "adidas": 2, "newbalance": 3, "new_balance": 3, "box": 6}
        return mapping.get(cls, 1)


def main(args=None):
    rclpy.init(args=args)
    
    dsr_temp = rclpy.create_node("dsr_seg_service_comm", namespace=ROBOT_ID)
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = dsr_temp
    
    try:
        node = SegmentationService()
    except Exception as e:
        print(f"Init failed: {e}")
        dsr_temp.destroy_node()
        rclpy.shutdown()
        return
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(dsr_temp)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        dsr_temp.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()