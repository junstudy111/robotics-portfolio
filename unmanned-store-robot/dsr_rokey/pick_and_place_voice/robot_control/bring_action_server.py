# bring_action_server.py (with Arduino Turntable Control - Option A)
"""
Segmentation Service로부터 픽셀 좌표를 받아서
Action Server에서 offset 적용 및 좌표 변환 수행
+ Arduino 턴테이블 제어 통합 (위치 추적은 Arduino 노드가 담당)
"""
# 아두이노 포함된 코드 2차 완성

import os
import time
import sys
import numpy as np
from scipy.spatial.transform import Rotation
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# 사용자 정의 인터페이스 임포트
from od_msg.srv import SrvDepthPosition
from od_msg.action import PickAndPlace
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import Trigger  # Arduino 서비스

# DSR 제어 및 그리퍼 모듈 임포트
import DR_init
from robot_control.onrobot import RG

# ===== Segmentation Pick Config Import =====
from robot_control.module.seg_pick_config import (
    POSITIONS,
    POSITION_CONFIGS,
    TRY_ON_POSITION,
    DROP_POSITIONS,
    HOME_POSITION,
    Z_OFFSET,
    APPROACH_HEIGHT,
    LIFT_HEIGHT,
    APPROACH_VEL,
    APPROACH_ACC,
    PICK_VEL,
    PICK_ACC
)

# ===== Segmentation Utils Import =====
from robot_control.module.seg_pick_utils import (
    get_camera_pos,
    transform_to_base,
    get_robot_pose_matrix
)


# ===============================
# A. 기본 설정 및 좌표 정의 (Globals)
# ===============================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"

# 재시도 설정
MAX_RETRY_ATTEMPTS = 3
RETRY_WAIT_TIME = 2.0
CAMERA_STABILIZATION_TIME = 1.5

# 전역 테이블
PREDEFINED_POS = {}
SHOE_ZONE = {}

# Object Name → Position Number 매핑
OBJECT_TO_POSITION = {
    "nike": 1,
    "adidas": 2,
    "new_balance": 3,
}

# DSR 함수 Placeholder
movej = movel = get_current_posx = mwait = posj = posx = wait = None
gripper = None


# ===============================
# Bring Action Server
# ===============================
class BringActionServer(Node):

    def __init__(self):
        super().__init__("bring_action_server")

        DR_init.__dsr__node = self
        self.callback_group = ReentrantCallbackGroup()

        # ===== Package Path =====
        package_path = get_package_share_directory("pick_and_place_voice")

        # ===== 1. gripper2cam 로드 =====
        self.get_logger().info("Loading gripper2cam transformation matrix...")
        gripper2cam_path = os.path.join(package_path, "resource", "T_gripper2camera.npy")
        
        if not os.path.exists(gripper2cam_path):
            self.get_logger().error(f"❌ Gripper2Cam not found: {gripper2cam_path}")
            raise RuntimeError("Gripper2Cam matrix file missing")
        
        self.gripper2cam = np.load(gripper2cam_path)
        self.get_logger().info("✓ Gripper2Cam matrix loaded")

        # ===== 2. Camera Intrinsics 로드 =====
        self.get_logger().info("Loading camera intrinsics...")
        from object_detection.realsense import ImgNode
        
        temp_img_node = ImgNode()
        
        for i in range(10):
            rclpy.spin_once(temp_img_node)
            time.sleep(0.1)
            self.intrinsics = temp_img_node.get_camera_intrinsic()
            if self.intrinsics is not None:
                self.get_logger().info(f"✓ Camera intrinsics loaded on attempt {i+1}")
                break
        
        if self.intrinsics is None:
            self.get_logger().error("❌ Failed to get camera intrinsics!")
            raise RuntimeError("Camera intrinsics unavailable")
        
        self.get_logger().info(
            f"✓ Camera params: fx={self.intrinsics['fx']:.1f}, "
            f"fy={self.intrinsics['fy']:.1f}"
        )

        # ===== 3. Segmentation 서비스 클라이언트 =====
        self.get_logger().info("Connecting to Segmentation Service...")
        
        self.seg_client = self.create_client(
            SrvDepthPosition,
            '/segmentation_pick',
            callback_group=self.callback_group
        )
        
        timeout = 10.0
        start_time = time.time()
        while not self.seg_client.wait_for_service(timeout_sec=1.0):
            if time.time() - start_time > timeout:
                self.get_logger().error("❌ Segmentation service not available!")
                raise RuntimeError("Segmentation service timeout")
            self.get_logger().info("Waiting for segmentation service...")
        
        self.get_logger().info("✅ Connected to Segmentation Service")

        # ===== 4. Arduino 서비스 클라이언트 =====
        self.get_logger().info("Connecting to Arduino Turntable Services...")
        
        # ⭐ 2개의 클라이언트 생성
        self.arduino_to_robot = self.create_client(
            Trigger,
            'turntable_to_robot',  # 고객 → 로봇 (반시계)
            callback_group=self.callback_group
        )
        
        self.arduino_to_customer = self.create_client(
            Trigger,
            'turntable_to_customer',  # 로봇 → 고객 (시계)
            callback_group=self.callback_group
        )
        
        # 서비스 대기 (Optional)
        timeout = 5.0
        start_time = time.time()
        
        services_ready = False
        while time.time() - start_time < timeout:
            if (self.arduino_to_robot.wait_for_service(timeout_sec=0.5) and
                self.arduino_to_customer.wait_for_service(timeout_sec=0.5)):
                services_ready = True
                break
            self.get_logger().info("Waiting for Arduino services...")
        
        if services_ready:
            self.get_logger().info("✅ Connected to Arduino Turntable Services")
        else:
            self.get_logger().warn("⚠️  Arduino services not available (continuing without it)")

        # ===== 5. Action Server 생성 =====
        self._action_server = ActionServer(
            self,
            PickAndPlace,
            'bring_action',
            self.execute_callback,
            callback_group=self.callback_group
        )

        self.init_robot()

        self.get_logger().info("="*60)
        self.get_logger().info("✅ Bring Action Server Ready")
        self.get_logger().info(f"   Max Retry: {MAX_RETRY_ATTEMPTS} attempts")
        self.get_logger().info("="*60)

    def _send_feedback_internal(self, goal_handle, message):
        if goal_handle:
            fb = PickAndPlace.Feedback()
            fb.status_message = message
            goal_handle.publish_feedback(fb)
        self.get_logger().info(f"[Feedback] {message}")

    def init_robot(self):
        global movej, gripper, mwait
        self.get_logger().info("Initializing robot to HOME position...")
        movej(HOME_POSITION, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()
        self.get_logger().info("✓ Robot initialized")

    def call_segmentation_service(self, target_class, position_num, goal_handle):
        """Segmentation 서비스 호출 (픽셀 좌표 받기)"""
        req = SrvDepthPosition.Request()
        req.target = f"{target_class}:{position_num}"
        
        self.get_logger().info(f"[SEG] Calling service: '{req.target}'")
        
        future = self.seg_client.call_async(req)
        
        start_time = time.time()
        timeout = 10.0

        while not future.done():
            if time.time() - start_time > timeout:
                self.get_logger().error("[SEG] Service call timeout")
                return False, None
            time.sleep(0.01)
        
        try:
            res = future.result()
            
            if res is None:
                self.get_logger().error("[SEG] No response from service")
                return False, None
            
            if len(res.depth_position) < 4:
                self.get_logger().error("[SEG] Invalid response format")
                return False, None
            
            if all(v == 0.0 for v in res.depth_position):
                self.get_logger().warn(f"[SEG] Failed to detect '{target_class}'")
                return False, None
            
            pixel_x, pixel_y, depth, angle = res.depth_position[0:4]
            
            self.get_logger().info(
                f"[SEG] Pixel coords: X={pixel_x:.1f}, Y={pixel_y:.1f}, "
                f"Depth={depth:.1f}, Angle={angle:.1f}°"
            )
            
            return True, {
                'pixel_x': pixel_x,
                'pixel_y': pixel_y,
                'depth': depth,
                'angle': angle
            }
            
        except Exception as e:
            self.get_logger().error(f"[SEG] Service error: {e}")
            import traceback
            traceback.print_exc()
            return False, None

    def execute_pick_segmentation(self, seg_result, position_num, goal_handle):
        """Segmentation 기반 픽업 동작 실행"""
        global movel, movej, gripper, mwait, posx, get_current_posx
        
        config = POSITION_CONFIGS[position_num]
        
        # Offset 적용 (픽셀 단위)
        pixel_x = seg_result['pixel_x'] + config.get("offset_x", 0)
        pixel_y = seg_result['pixel_y'] + config.get("offset_y", 0)
        depth = seg_result['depth']
        angle_offset = seg_result['angle']
        
        self.get_logger().info(
            f"[PICK] After offset: Pixel({pixel_x:.1f}, {pixel_y:.1f}), Depth={depth:.1f}"
        )
        
        # 좌표 변환
        cam_pos = get_camera_pos(pixel_x, pixel_y, depth, self.intrinsics)
        current_pos = get_current_posx()
        robot_coord = transform_to_base(cam_pos, current_pos[0], self.gripper2cam)
        
        x, y, z = robot_coord
        z -= Z_OFFSET
        
        self.get_logger().info(
            f"[PICK] Robot coords: X={x:.1f}, Y={y:.1f}, Z={z:.1f}"
        )

        # Pick 동작
        current = get_current_posx()[0]
        curr_rx, curr_ry, curr_rz = current[3], current[4], current[5]
        target_rz = curr_rz + angle_offset

        approach = posx([x, y, z + APPROACH_HEIGHT, curr_rx, curr_ry, target_rz])
        pick     = posx([x, y, z,                   curr_rx, curr_ry, target_rz])
        lift     = posx([x, y, z + LIFT_HEIGHT,     curr_rx, curr_ry, target_rz])

        self._send_feedback_internal(goal_handle, f"3.1 Approach (RZ: {target_rz:.2f}°)")
        movel(approach, vel=APPROACH_VEL, acc=APPROACH_ACC)
        mwait()

        self._send_feedback_internal(goal_handle, "3.2 Pick")
        movel(pick, vel=PICK_VEL, acc=PICK_ACC)
        mwait()
        
        self._send_feedback_internal(goal_handle, "3.3 Closing gripper")
        gripper.close_gripper()
        time.sleep(1.0)

        self._send_feedback_internal(goal_handle, "3.4 Lift")
        movel(lift, vel=APPROACH_VEL, acc=APPROACH_ACC)
        mwait()
        
        self._send_feedback_internal(goal_handle, "3.5 Return to HOME")
        movej(HOME_POSITION, vel=APPROACH_VEL, acc=APPROACH_ACC)
        mwait()
        
        self.get_logger().info("✓ Pick Done")
        return True

    # ⭐⭐⭐ Arduino 호출 (위치 추적은 Arduino 노드가 알아서) ⭐⭐⭐
    def call_arduino_to_robot(self, goal_handle):
        """
        턴테이블 → 로봇 쪽
        ⭐ 단순히 명령만 전송 (중복 체크는 Arduino 노드가 담당)
        """
        if not self.arduino_to_robot.service_is_ready():
            self.get_logger().warn("[Arduino] to_robot service not ready")
            return
        
        self.get_logger().info("[Arduino] Command: Rotate to Robot")
        
        req = Trigger.Request()
        future = self.arduino_to_robot.call_async(req)
        
        start_time = time.time()
        timeout = 3.0
        
        while not future.done():
            if time.time() - start_time > timeout:
                self.get_logger().warn("[Arduino] to_robot timeout")
                return
            time.sleep(0.01)
        
        try:
            res = future.result()
            if res.success:
                self.get_logger().info(f"[Arduino] ✓ {res.message}")
            else:
                self.get_logger().warn(f"[Arduino] ✗ {res.message}")
        except Exception as e:
            self.get_logger().error(f"[Arduino] Error: {e}")

    def call_arduino_to_customer(self, goal_handle):
        """
        턴테이블 → 고객 쪽
        ⭐ 단순히 명령만 전송 (중복 체크는 Arduino 노드가 담당)
        """
        if not self.arduino_to_customer.service_is_ready():
            self.get_logger().warn("[Arduino] to_customer service not ready")
            return
        
        self.get_logger().info("[Arduino] Command: Rotate to Customer")
        
        req = Trigger.Request()
        future = self.arduino_to_customer.call_async(req)
        
        start_time = time.time()
        timeout = 3.0
        
        while not future.done():
            if time.time() - start_time > timeout:
                self.get_logger().warn("[Arduino] to_customer timeout")
                return
            time.sleep(0.01)
        
        try:
            res = future.result()
            if res.success:
                self.get_logger().info(f"[Arduino] ✓ {res.message}")
            else:
                self.get_logger().warn(f"[Arduino] ✗ {res.message}")
        except Exception as e:
            self.get_logger().error(f"[Arduino] Error: {e}")

    def place_object(self, target_zone, goal_handle):
        """Place 동작"""
        global movel, movej, gripper, mwait

        drop_pos = TRY_ON_POSITION
        joint_coords = SHOE_ZONE[target_zone] 
        pass_the_singularity = posj([-49.602, 37.928, 38.823, 0.822, 102.755, -138.474])
        
        # ⭐⭐⭐ Place 전: 턴테이블을 로봇 쪽으로 ⭐⭐⭐
        self._send_feedback_internal(goal_handle, "4.1 Rotating turntable to robot...")
        self.call_arduino_to_robot(goal_handle)
        
        self._send_feedback_internal(
            goal_handle, f"4.2 Move to placement zone: {target_zone}"
        )
        
        movej(joint_coords, vel=VELOCITY, acc=ACC) 
        mwait()
        movej(pass_the_singularity, vel=VELOCITY, acc=ACC) 
        mwait()        
        movej(drop_pos, vel=VELOCITY, acc=ACC)
        mwait()
        
        self._send_feedback_internal(goal_handle, "4.3 Opening gripper")
        gripper.open_gripper()
        time.sleep(1.5)
        mwait()
        
        movej(joint_coords, vel=VELOCITY, acc=ACC) 
        mwait()        
        
        self._send_feedback_internal(goal_handle, "4.4 Returning to HOME")
        movej(HOME_POSITION, vel=VELOCITY, acc=ACC)
        mwait()
        
        # ⭐⭐⭐ Place 후: 턴테이블을 고객 쪽으로 ⭐⭐⭐
        self._send_feedback_internal(goal_handle, "4.5 Rotating turntable to customer...")
        self.call_arduino_to_customer(goal_handle)
        
        self.get_logger().info(f"✓ Placed at {target_zone}")
        return True

    def execute_callback(self, goal_handle):
        """Action 실행 콜백 (재시도 로직 포함)"""
        
        global movej, movel, get_current_posx, mwait, gripper

        result = PickAndPlace.Result()

        object_name = goal_handle.request.target_object
        command = goal_handle.request.target_command

        self.get_logger().info(f"\n{'='*70}")
        self.get_logger().info(f"[BRING] New request: {command} {object_name}")
        self.get_logger().info(f"{'='*70}")

        try:
            # 0) 그리퍼 확인
            self._send_feedback_internal(goal_handle, "0. Checking robot")
            
            if not gripper.get_status()[0]:
                self.get_logger().info("Opening gripper...")
                gripper.open_gripper()
                time.sleep(1.0)
                mwait()

            # 1) Detection View 이동
            if object_name not in PREDEFINED_POS:
                raise ValueError(f"[ERROR] Unknown object: {object_name}")

            detection_pos = PREDEFINED_POS[object_name]

            self._send_feedback_internal(
                goal_handle, f"1. Moving to detection view: {object_name}"
            )
            movej(detection_pos, vel=VELOCITY, acc=ACC)
            mwait()
            
            self.get_logger().info(f"✓ At detection position")

            # 2) Segmentation 서비스 호출 (재시도 로직)
            position_num = OBJECT_TO_POSITION.get(object_name)
            if position_num is None:
                raise ValueError(f"[ERROR] No position for {object_name}")
            
            config = POSITION_CONFIGS.get(position_num)
            if config is None:
                raise ValueError(f"[ERROR] No config for position {position_num}")
            
            target_class = config["target_class"]
            
            # ===== 재시도 루프 =====
            seg_success = False
            seg_result = None
            
            for attempt in range(1, MAX_RETRY_ATTEMPTS + 1):
                self._send_feedback_internal(
                    goal_handle, 
                    f"2. Segmentation attempt {attempt}/{MAX_RETRY_ATTEMPTS}: {target_class}"
                )
                
                self.get_logger().info(f"[Retry {attempt}] Camera stabilization...")
                time.sleep(CAMERA_STABILIZATION_TIME)
                
                seg_success, seg_result = self.call_segmentation_service(
                    target_class, position_num, goal_handle
                )
                
                if seg_success:
                    self.get_logger().info(f"✓ Segmentation succeeded on attempt {attempt}")
                    break
                
                if attempt < MAX_RETRY_ATTEMPTS:
                    self._send_feedback_internal(
                        goal_handle,
                        f"⚠️  Detection failed ({attempt}/{MAX_RETRY_ATTEMPTS}). Retrying..."
                    )
                    
                    self.get_logger().warn(f"[Retry {attempt}] Failed. Returning to HOME...")
                    
                    movej(HOME_POSITION, vel=VELOCITY, acc=ACC)
                    mwait()
                    
                    if not gripper.get_status()[0]:
                        gripper.open_gripper()
                        time.sleep(1.0)
                    
                    time.sleep(RETRY_WAIT_TIME)
                    
                    movej(detection_pos, vel=VELOCITY, acc=ACC)
                    mwait()
                    
                else:
                    self.get_logger().error(
                        f"[FINAL] Failed after {MAX_RETRY_ATTEMPTS} attempts"
                    )
                    
                    movej(HOME_POSITION, vel=VELOCITY, acc=ACC)
                    mwait()
                    
                    raise ValueError(
                        f"Failed to detect '{target_class}' "
                        f"after {MAX_RETRY_ATTEMPTS} attempts"
                    )
            
            if not seg_success or seg_result is None:
                raise RuntimeError("Segmentation failed unexpectedly")

            # 3) Pick
            self._send_feedback_internal(goal_handle, "3. Executing pick")
            
            pick_success = self.execute_pick_segmentation(
                seg_result, position_num, goal_handle
            )
            
            if not pick_success:
                raise RuntimeError("Pick failed")

            # 4) Place
            if command == "갖다줘":
                place_zone_key = "try_on_zone"
            else:
                result.success_message = f"Pick only: {object_name}"
                goal_handle.succeed()
                return result

            self._send_feedback_internal(goal_handle, f"4. Placing at {place_zone_key}")
            
            # ⭐ place_object 내부에서 Arduino 제어 포함
            self.place_object(place_zone_key, goal_handle)

            # 5) 성공
            result.success_message = f"{object_name} placed successfully"
            
            self.get_logger().info(f"\n{'='*70}")
            self.get_logger().info(f"✅ SUCCESS: {result.success_message}")
            self.get_logger().info(f"[BRING] Ready for next")
            self.get_logger().info(f"{'='*70}\n")
            goal_handle.succeed()

            return result

        except Exception as e:
            self.get_logger().error(f"\n{'='*70}")
            self.get_logger().error(f"❌ ERROR: {e}")
            self.get_logger().error(f"{'='*70}\n")
            import traceback
            traceback.print_exc()
            
            try:
                self.get_logger().info("Emergency: Returning to HOME...")
                movej(HOME_POSITION, vel=VELOCITY, acc=ACC)
                mwait()
            except:
                pass
            
            result.success_message = f"Failed: {str(e)}"
            goal_handle.abort()
            return result


# ===============================
# MAIN
# ===============================
def main(args=None):

    rclpy.init(args=args)

    dsr_temp = rclpy.create_node("dsr_communication_node", namespace=ROBOT_ID)

    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = dsr_temp

    global movej, movel, get_current_posx, mwait, posj, posx, gripper, wait

    from DSR_ROBOT2 import (
        movej as mvj, 
        movel as mvl, 
        get_current_posx as gcpx, 
        mwait as mwt,
        wait as wt
    )
    from DR_common2 import posj as psj, posx as psx
    
    movej, movel, get_current_posx, mwait, wait = mvj, mvl, gcpx, mwt, wt
    posj, posx = psj, psx

    global PREDEFINED_POS
    PREDEFINED_POS.update({
        "nike": POSITIONS[1],
        "adidas": POSITIONS[2],
        "new_balance": POSITIONS[3],
    })

    global SHOE_ZONE
    SHOE_ZONE.update({
        "try_on_zone": DROP_POSITIONS['A'],
    })

    gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

    node = BringActionServer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(dsr_temp)

    try:
        node.get_logger().info("Bring Action Server starting...")
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        dsr_temp.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()