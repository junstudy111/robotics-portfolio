# seg_pick_config.py
"""
Segmentation Pick & Place 시스템 설정
"""
from DR_common2 import posj, posx

# ---------------------------
# 로봇 기본 설정
# ---------------------------
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY = 60
ACC = 60

# 그리퍼 설정
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"

# Z축 오프셋
Z_OFFSET = 10

# ---------------------------
# 로봇 포지션 설정
# ---------------------------
# 홈 포지션
HOME_POSITION = posj([0.05, 2.545, 68.245, -0.006, 109.213, -93.704])

# 시착 존 포지션
TRY_ON_POSITION = posj([-49.666, 31.67, 61.626, 0.172, 87.489, -143.006])

# 반품 했을 때 상자를 반납 존 포지션 6번 자리
RETURN_ZONE_POSITION = posj([20.031, 38.956, 47.964, 0.287, 93.049, -71.961])

# 구매 했을 때 상자 내려놓는 위치
BOX_DROP_POSITION = posj([-80.078, 17.328, 84.95, 0.085, 77.85, -261.584])


# 저장된 포지션 (1~6번) - 감지용
POSITIONS = {
    1: posj([-27.531, 8.172, 76.058, 0.014, 95.511, -120.419]),
    2: posj([2.461, 4.505, 80.057, 0.015, 95.398, -90.261]),
    3: posj([32.626, 11.771, 74.862, -0.17, 92.982, -60.248]),
    4: posj([-18.388, 51.578, 13.655, 0.689, 113.598, -111.005]),
    5: posj([0.479, 27.935, 25.592, 0.187, 126.474, 87.41]),
    6: posj([22.874, 34.491, 17.372, 0.245, 128.272, 112.466])
}

# 배치 위치 (A: TRY_ON 감지용, B: Box 감지용)
DROP_POSITIONS = {
            'A': posj([-45.015, 45.184, 7.801, 0.43, 126.549, -138.296]),
            'B': posj([-81.944, 7.667, 57.412, 0.04, 114.922, -264.221])
}

# ---------------------------
# 포지션별 픽업 설정 (BRING - 신발 가져오기)
# ---------------------------
POSITION_CONFIGS = {
    1: {  # Nike
        "target_class": "nike",
        "use_fixed_depth": True,
        "fixed_depth": 410,
        "offset_x": -36,
        "offset_y": 80
    },
    2: {  # Adidas
        "target_class": "adidas",
        "use_fixed_depth": True,
        "fixed_depth": 408,
        "offset_x": -38,
        "offset_y": 85
    },
    3: {  # NewBalance
        "target_class": "newbalance",
        "use_fixed_depth": True,
        "fixed_depth": 400,
        "offset_x": -36,
        "offset_y": 80
    },
    4: {  # Puma
        "target_class": "puma",
        "use_fixed_depth": True,
        "fixed_depth": 410,
        "offset_x": 20,
        "offset_y": 0
    },
    5: {  # Box (Purchase)
        "target_class": "box",
        "use_closest": True,
        "use_fixed_depth": True,
        "fixed_depth": 475,
        "offset_x": -20,
        "offset_y": 10
    },
    6: {  # Box (Return)
        "target_class": "box",
        "use_closest": True,
        "use_fixed_depth": True,
        "fixed_depth": 460,
        "offset_x": -30,
        "offset_y": 20
    }
}

# ---------------------------
# CLEAR 동작용 설정 (신발 정리)
# ---------------------------
# TRY_ON 구역 감지 포지션 (A 포지션)
TRY_ON_DETECTION_POSITION = DROP_POSITIONS['A']

# Box 구역 감지 포지션 (B 포지션)
BOX_DETECTION_POSITION = DROP_POSITIONS['B']

# CLEAR 마지막 복귀 위치
CLEAR_FINAL_POSITION = HOME_POSITION

# TRY_ON 구역에서 신발 집을 때의 offset (브랜드별)
CLEAR_POSITION_CONFIGS = {
    "nike": {
        "use_fixed_depth": True,
        "fixed_depth": 420,
        "offset_x": -50,
        "offset_y": 75
    },
    "adidas": {
        "use_fixed_depth": True,
        "fixed_depth": 425,
        "offset_x": -40,
        "offset_y": 95
    },
    "newbalance": {
        "use_fixed_depth": True,
        "fixed_depth": 430,
        "offset_x": -45,
        "offset_y": 80
    }
}

# 원래 위치에 놓을 때의 설정 (Zone별 - 실측값)
CLEAR_PLACE_POSITIONS = {
    1: {  # Nike 원래 자리
        "approach_joint": posj([-30.032, 7.43, 104.74, -0.034, 68.323, -119.562]),  # movej로 접근
        "place_cartesian": posx([340.79, -193.615, 281.119, 158.806, 179.5, 69.767]),  # movel로 하강
        "approach_height": 50  # mm
    },
    2: {  # Adidas 원래 자리
        "approach_joint": posj([1.573, 4.33, 110.476, -0.167, 66.11, -89.108]),
        "place_cartesian": posx([361.963, 12.783, 267.889, 12.034, -179.068, -78.211]),
        "approach_height": 50
    },
    3: {  # NewBalance 원래 자리
        "approach_joint": posj([31.408, 11.778, 100.087, -0.303, 68.878, -62.378]),
        "place_cartesian": posx([359.698, 222.296, 277.746, 57.336, -179.17, -36.064]),
        "approach_height": 50
    },

    5: {  # nike 박스 원래 자리
        "approach_joint": posj([0.456, 30.871, 60.017, 0.23, 89.111, -90.162]),
        "place_cartesian": posx([577.931, 7.993, 359.596, 179.697, -179.999, 89.521]),
        "dodge_action" : posj([1.809, 31.414, 24.542, -0.07, 120.84, -87.14]),
        "approach_height": 50
    },
    6: {  # adidas 박스 원래 자리
        "approach_joint": posj([20.033, 35.134, 53.239, 0.071, 91.397, -69.333]),
        "place_cartesian": posx([566.357, 209.902, 359.127, 160.255, -179.702, 71.305]),
        "dodge_action" : posj([22.606, 44.732, -0.038, -1.694, 132.031, -67.371]),
        "approach_height": 50
    }
}

# CLEAR 동작 시 감지할 신발 클래스 순서
CLEAR_CLASS_ORDER = [
    ("nike", 1),
    ("adidas", 2),
    ("newbalance", 3)
]

# ---------------------------
# RETURN 동작용 설정 (박스 반품)
# ---------------------------
# Box 구역에서 박스 집을 때의 offset
RETURN_POSITION_CONFIGS = {
    "box": {
        "use_fixed_depth": True,
                "fixed_depth": 330,
                "offset_x": -30,
                "offset_y": 20
    }
}

# RETURN 동작 시 감지할 클래스
RETURN_CLASS_ORDER = [
    ("box", 6)
]

# RETURN 마지막 복귀 위치
RETURN_FINAL_POSITION = HOME_POSITION

# ---------------------------
# 픽업 동작 파라미터
# ---------------------------
APPROACH_HEIGHT = 50  # Approach 높이 (mm)
LIFT_HEIGHT = 100     # Lift 높이 (mm)
APPROACH_VEL = 50     # Approach 속도
APPROACH_ACC = 50     # Approach 가속도
PICK_VEL = 30         # Pick 속도
PICK_ACC = 30         # Pick 가속도