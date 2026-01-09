# seg_pick_utils.py
"""
Segmentation Pick & Place 유틸리티 함수
"""
import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def get_segmentation_center(mask_contour):
    """
    Segmentation 마스크의 중심점 계산
    """
    cnt = np.array(mask_contour, dtype=np.int32)
    
    if len(cnt) == 0:
        return None
    
    # 윤곽선의 중심점(moments 이용)
    M = cv2.moments(cnt)
    if M["m00"] == 0:
        return None
        
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    
    return (cx, cy)


def get_angle_from_segmentation(mask_contour):
    """
    Segmentation 윤곽선으로 짧은 면 각도 계산
    """
    cnt = np.array(mask_contour, dtype=np.int32)

    if len(cnt) == 0:
        return 0.0

    rect = cv2.minAreaRect(cnt)
    (cx, cy), (rw, rh), angle = rect

    if rw < rh:
        target_angle = angle
    else:
        target_angle = angle + 90

    print(f"[Seg Angle] W:{rw:.1f}, H:{rh:.1f}, Org:{angle:.1f} -> Target:{target_angle:.1f}")
    return target_angle


def get_camera_pos(px, py, pz, K):
    """
    픽셀 좌표를 카메라 좌표로 변환
    """
    cx = (px - K["ppx"]) * pz / K["fx"]
    cy = (py - K["ppy"]) * pz / K["fy"]
    return (cx, cy, pz)


def get_robot_pose_matrix(x, y, z, rx, ry, rz):
    """
    로봇 포즈를 4x4 변환 행렬로 변환
    """
    R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def transform_to_base(camera_coords, current_pose, gripper2cam):
    """
    카메라 좌표를 로봇 베이스 좌표로 변환
    """
    coord = np.append(camera_coords, 1)
    base2gripper = get_robot_pose_matrix(*current_pose)
    base2cam = base2gripper @ gripper2cam
    td = base2cam @ coord
    return td[:3]


def get_depth_value(x, y, depth_frame):
    """
    특정 픽셀 위치의 depth 값 가져오기
    """
    h, w = depth_frame.shape
    if 0 <= x < w and 0 <= y < h:
        return depth_frame[y, x]
    return None


def calculate_angle_offset(raw_angle, position):
    """
    포지션별 각도 오프셋 계산
    5, 6번: 각도 회전 비활성화
    1, 2, 3, 4번: ±90도 범위로 제한
    """
    if position in [5, 6]:
        print(f"⚙️  Angle: Disabled for position {position}")
        return 0
    else:
        # 각도를 -180 ~ 180 범위로 정규화
        while raw_angle > 180:
            raw_angle -= 360
        while raw_angle <= -180:
            raw_angle += 360
        
        # -90 ~ 90 범위로 조정
        if raw_angle > 90:
            angle_offset = raw_angle - 180
        elif raw_angle < -90:
            angle_offset = raw_angle + 180
        else:
            angle_offset = raw_angle
            
        print(f"⚙️  Angle: Raw={raw_angle:.1f}° → Adjusted={angle_offset:.1f}°")
        return angle_offset

# def calculate_angle_offset(raw_angle, position):
#     """
#     포지션별 각도 오프셋 계산
#     5, 6번: 각도 회전 비활성화
#     1, 2, 3, 4번: ±90도 범위로 제한
#     """
#     if position in [5, 6]:
#         print(f"⚙️  Angle: Disabled for position {position}")
#         return 0
#     else:
#         if -90 < raw_angle <= 90:
#             angle_offset = raw_angle
#         else:
#             angle_offset = raw_angle - 180
#         print(f"⚙️  Angle: Raw={raw_angle:.1f}° → Adjusted={angle_offset:.1f}°")
#         return angle_offset