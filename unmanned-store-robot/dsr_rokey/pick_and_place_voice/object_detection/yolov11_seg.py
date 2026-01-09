# object_detection/yolov11_seg.py (올바른 버전)
# v4 자동검출

import os
import json
import time
import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO

# ❌ 이 줄 삭제! (혹시 있으면)
# from object_detection.yolov11_seg import YoloModel

PACKAGE_NAME = "pick_and_place_voice"
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)

YOLO_MODEL_FILENAME = "yolov11_seg_best.pt"  
YOLO_CLASS_NAME_JSON = "class_name_tool.json"

YOLO_MODEL_PATH = os.path.join(PACKAGE_PATH, "resource", YOLO_MODEL_FILENAME)
YOLO_JSON_PATH = os.path.join(PACKAGE_PATH, "resource", YOLO_CLASS_NAME_JSON)


class YoloModel:
    def __init__(self):
        print(f"[YoloModel] Loading Segmentation Model from: {YOLO_MODEL_PATH}")
        
        if not os.path.exists(YOLO_MODEL_PATH):
            print(f"❌ Error: Model file not found at {YOLO_MODEL_PATH}")
            print("   Please copy your 'best.pt' to the resource folder.")
        
        self.model = YOLO(YOLO_MODEL_PATH)
        
        print(f"[YoloModel] Loading Class Map from: {YOLO_JSON_PATH}")
        with open(YOLO_JSON_PATH, "r", encoding="utf-8") as file:
            class_dict = json.load(file)
            self.class_map = {k.lower(): v for k, v in class_dict.items()}
            
        print(f"✅ Class Map Loaded: {self.class_map}")

    def get_frames(self, img_node, duration=1.0):
        """지정된 시간 동안 프레임을 수집합니다."""
        end_time = time.time() + duration
        frames = []

        while time.time() < end_time:
            rclpy.spin_once(img_node)
            frame = img_node.get_color_frame()
            if frame is not None:
                frames.append(frame)
            time.sleep(0.01)

        return frames

    def get_best_detection(self, img_node, target):
        """
        타겟(target) 이름에 해당하는 물체를 찾아서
        Box, Angle, Score를 반환합니다. (하위 호환성 유지)
        """
        box, angle, score, mask = self.get_best_detection_with_mask(img_node, target)
        return box, angle, score

    def get_best_detection_with_mask(self, img_node, target):
        """
        타겟(target) 이름에 해당하는 물체를 찾아서
        Box, Angle, Score, Mask를 반환합니다.
        """
        frames = self.get_frames(img_node)
        if not frames:
            print("⚠️ No frames captured from camera.")
            return None, 0.0, None, None

        # 1. Segmentation 모드로 추론 (conf=0.5 이상만 감지)
        results = self.model(frames, conf=0.5, verbose=False)
        
        # 2. 타겟 이름(target)을 ID로 변환 (소문자 처리)
        target_lower = target.lower()
        
        if target_lower in self.class_map:
            target_id = self.class_map[target_lower]
            print(f"🔍 Searching for '{target}' (ID: {target_id})...")
        else:
            print(f"⚠️ Target '{target}' not found in JSON. Available keys: {list(self.class_map.keys())}")
            return None, 0.0, None, None

        best_det = None
        max_score = -1.0
        
        # 3. 결과 분석: 타겟 ID와 일치하는 것 중 가장 점수 높은 것 찾기
        for res in results:
            if res.boxes is None or res.masks is None:
                continue
                
            # box, mask, cls(클래스ID), score(확률)를 묶어서 반복
            for box, mask, cls, score in zip(res.boxes, res.masks, res.boxes.cls, res.boxes.conf):
                current_id = int(cls)
                current_score = float(score)

                if current_id == target_id:
                    if current_score > max_score:
                        max_score = current_score
                        best_det = {
                            "box": box.xyxy[0].cpu().numpy(), # [x1, y1, x2, y2]
                            "mask": mask.xy[0],               # 윤곽선 좌표
                            "score": current_score
                        }

        if best_det is None:
            print(f"❌ Failed to detect object ID {target_id} in current frame.")
            return None, 0.0, None, None

        # 4. 각도 계산 (Segmentation 윤곽선 활용)
        angle = self._calculate_angle_from_mask(best_det["mask"])
        
        print(f"✅ Detect Success! Score: {max_score:.2f}, Angle: {angle:.1f}°")
        
        # mask도 함께 반환
        return best_det["box"], angle, best_det["score"], best_det["mask"]

    def _calculate_angle_from_mask(self, mask_contour):
        """마스크 윤곽선을 이용해 회전 각도(Angle) 계산"""
        cnt = np.array(mask_contour, dtype=np.int32)
        if len(cnt) == 0: 
            return 0.0

        # 최소 면적 사각형 (Rotated Rectangle) 구하기
        rect = cv2.minAreaRect(cnt)
        (cx, cy), (rw, rh), angle = rect

        # 로봇 그리퍼가 짧은 면을 잡도록 각도 보정
        if rw < rh:
            target_angle = angle
        else:
            target_angle = angle + 90

        # 각도 정규화 (-90도 ~ 90도 사이로 맞춤)
        while target_angle > 90: 
            target_angle -= 180.0
        while target_angle < -90: 
            target_angle += 180.0

        return target_angle