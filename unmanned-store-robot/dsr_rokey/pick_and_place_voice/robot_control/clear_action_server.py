# clear_action_server.py (Final Merged Version)
# =============================================================================
# 기능:
# 1. 아두이노 턴테이블 제어 (시작 시 로봇 쪽, 픽업 후 고객 쪽 회전)
# 2. 중앙 우선 타겟팅 (Winner Takes All) 적용
# 3. 서비스 타임아웃 방지 및 안전 로직 포함
# =============================================================================

import os
import time
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory

# ==========================================================
# 두산 로봇(DSR) 라이브러리 및 그리퍼
# ==========================================================
import DR_init
from robot_control.onrobot import RG

# ==========================================================
# ROS 2 인터페이스 (Service & Action)
# ==========================================================
from od_msg.srv import SrvDepthPosition
from od_msg.action import PickAndPlace
from std_srvs.srv import Trigger  # ✅ Arduino 제어용

# ==========================================================
# DSR 좌표계 함수
# ==========================================================
from DR_common2 import posj, posx

# ==========================================================
# 사용자 설정값 (Configs) Import
# ==========================================================
from robot_control.module.seg_pick_config import (
    ROBOT_ID, 
    ROBOT_MODEL, 
    VELOCITY, 
    ACC,
    HOME_POSITION, 
    TRY_ON_DETECTION_POSITION, 
    CLEAR_FINAL_POSITION, 
    CLEAR_POSITION_CONFIGS, 
    CLEAR_PLACE_POSITIONS, 
    CLEAR_CLASS_ORDER,
    APPROACH_HEIGHT, 
    LIFT_HEIGHT, 
    PICK_VEL, 
    PICK_ACC,
    GRIPPER_NAME, 
    TOOLCHARGER_IP, 
    TOOLCHARGER_PORT, 
    Z_OFFSET
)

# ==========================================================
# 유틸리티 함수 (좌표 변환용) Import
# ==========================================================
from robot_control.module.seg_pick_utils import (
    get_camera_pos, 
    transform_to_base
)

# ==========================================================
# 전역 변수 및 상수 설정
# ==========================================================
MAX_RETRY_ATTEMPTS = 3          # 최대 재시도 횟수
RETRY_WAIT_TIME = 2.0           # 재시도 전 대기 시간
CAMERA_STABILIZATION_TIME = 2.0 # 카메라 안정화 대기 시간

TRY_ON_SAFE_APPROACH_JOINT = posj([-45.015, 45.184, 7.801, 0.43, 126.549, -138.296])

# DSR 함수 Placeholder
movej = movel = mwait = gripper = wait = get_current_posx = None


# ==========================================================
# ClearActionServer 클래스 정의
# ==========================================================
class ClearActionServer(Node):

    def __init__(self):
        super().__init__("clear_action_server")

        # 콜백 그룹 설정 (Reentrant 필수)
        self.callback_group = ReentrantCallbackGroup()

        self.get_logger().info("Initializing ClearActionServer...")

        # ------------------------------------------------------
        # 1. Gripper2Camera 변환 행렬 로드
        # ------------------------------------------------------
        pkg_path = get_package_share_directory("pick_and_place_voice")
        mat_path = os.path.join(pkg_path, "resource", "T_gripper2camera.npy")
        
        if os.path.exists(mat_path):
            self.gripper2cam = np.load(mat_path)
            self.get_logger().info(f"✓ Transformation Matrix Loaded: {mat_path}")
        else:
            self.get_logger().error(f"❌ Matrix file not found: {mat_path}")
            raise RuntimeError("Matrix file missing")

        # ------------------------------------------------------
        # 2. Camera Intrinsics (내부 파라미터) 로드
        # ------------------------------------------------------
        self.intrinsics = {
            'fx': 615.0, 'fy': 615.0, 
            'cx': 320.0, 'cy': 240.0,
            'ppx': 320.0, 'ppy': 240.0
        }
        
        try:
            from object_detection.realsense import ImgNode
            temp_node = ImgNode()
            time.sleep(0.5) 
            intr = temp_node.get_camera_intrinsic()
            if intr: 
                self.intrinsics = intr
                if 'ppx' not in self.intrinsics and 'cx' in self.intrinsics:
                    self.intrinsics['ppx'] = self.intrinsics['cx']
                    self.intrinsics['ppy'] = self.intrinsics['cy']
                self.get_logger().info("✓ Camera Intrinsics Loaded from RealSense")
            temp_node.destroy_node()
        except Exception as e:
            self.get_logger().warn(f"⚠️ Using default intrinsics: {e}")

        # ------------------------------------------------------
        # 3. Segmentation Service Client 설정
        # ------------------------------------------------------
        self.seg_client = self.create_client(
            SrvDepthPosition, 
            "/segmentation_pick", 
            callback_group=self.callback_group
        )
        
        wait_start = time.time()
        while not self.seg_client.wait_for_service(timeout_sec=1.0):
            if time.time() - wait_start > 10.0:
                self.get_logger().error("❌ Segmentation Service Timeout!")
                raise RuntimeError("Seg Service Timeout")
            self.get_logger().info("Waiting for Segmentation Service...")
        
        self.get_logger().info("✓ Connected to Segmentation Service")

        # ------------------------------------------------------
        # 4. ⭐ Arduino 서비스 클라이언트 추가 (제공된 코드 반영)
        # ------------------------------------------------------
        self.get_logger().info("Connecting to Arduino Turntable Services...")
        
        self.arduino_to_robot = self.create_client(
            Trigger,
            'turntable_to_robot',  # 고객 → 로봇 (시계)
            callback_group=self.callback_group
        )
        
        self.arduino_to_customer = self.create_client(
            Trigger,
            'turntable_to_customer',  # 로봇 → 고객 (반시계)
            callback_group=self.callback_group
        )
        
        # 서비스 연결 확인 (Optional)
        timeout = 2.0
        start_time = time.time()
        services_ready = False
        while time.time() - start_time < timeout:
            if (self.arduino_to_robot.wait_for_service(timeout_sec=0.5) and
                self.arduino_to_customer.wait_for_service(timeout_sec=0.5)):
                services_ready = True
                break
        
        if services_ready:
            self.get_logger().info("✅ Connected to Arduino Turntable Services")
        else:
            self.get_logger().warn("⚠️ Arduino services not found (Logic will continue)")

        # ------------------------------------------------------
        # 5. Action Server 생성
        # ------------------------------------------------------
        self._action_server = ActionServer(
            self, 
            PickAndPlace, 
            "clear_action", 
            self.execute_callback, 
            callback_group=self.callback_group
        )

        self.get_logger().info("="*60)
        self.get_logger().info("✅ CLEAR Action Server Ready")
        self.get_logger().info("="*60)

    # ---------------------------------------------------------
    # Helper: 피드백 전송 함수
    # ---------------------------------------------------------
    def _send_feedback(self, handle, msg):
        if handle:
            fb = PickAndPlace.Feedback()
            fb.status_message = msg
            handle.publish_feedback(fb)
        self.get_logger().info(f"[FB] {msg}")

    # ---------------------------------------------------------
    # ⭐ Arduino 제어 함수 ⭐
    # ---------------------------------------------------------
    def call_arduino_to_robot(self):
        """턴테이블 → 로봇 쪽 (시계)"""
        if not self.arduino_to_robot.service_is_ready():
            self.get_logger().warn("[Arduino] 'to_robot' service not ready")
            return
        
        self.get_logger().info("[Arduino] Rotating to Robot (Clockwise)...")
        req = Trigger.Request()
        future = self.arduino_to_robot.call_async(req)
        
        # 비동기 호출 후 대기 (타임아웃 설정)
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > 3.0:
                self.get_logger().warn("[Arduino] 'to_robot' Call Timeout")
                return
            time.sleep(0.01)
            
        try:
            res = future.result()
            if res.success: self.get_logger().info(f"[Arduino] ✓ {res.message}")
            else: self.get_logger().warn(f"[Arduino] ✗ {res.message}")
        except Exception as e:
            self.get_logger().error(f"[Arduino] Error: {e}")

    def call_arduino_to_customer(self):
        """턴테이블 → 고객 쪽 (반시계)"""
        if not self.arduino_to_customer.service_is_ready():
            self.get_logger().warn("[Arduino] 'to_customer' service not ready")
            return
        
        self.get_logger().info("[Arduino] Rotating to Customer (Counter-Clockwise)...")
        req = Trigger.Request()
        future = self.arduino_to_customer.call_async(req)
        
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > 3.0:
                self.get_logger().warn("[Arduino] 'to_customer' Call Timeout")
                return
            time.sleep(0.01)
            
        try:
            res = future.result()
            if res.success: self.get_logger().info(f"[Arduino] ✓ {res.message}")
            else: self.get_logger().warn(f"[Arduino] ✗ {res.message}")
        except Exception as e:
            self.get_logger().error(f"[Arduino] Error: {e}")

    # ---------------------------------------------------------
    # Segmentation 서비스 호출
    # ---------------------------------------------------------
    def call_segmentation_service(self, target_class, position_num):
        req = SrvDepthPosition.Request()
        req.target = f"{target_class}:{position_num}"
        
        future = self.seg_client.call_async(req)
        
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > 10.0:
                self.get_logger().error(f"❌ Seg Service Timeout for {target_class}")
                return False, None, ""
            time.sleep(0.01)
        
        try:
            res = future.result()
            
            if not res or len(res.depth_position) < 4:
                return False, None, ""
            
            if all(v == 0.0 for v in res.depth_position): 
                return False, None, ""

            x, y, z, ang = res.depth_position[0:4]
            name = res.shoe_name if hasattr(res, 'shoe_name') else target_class
            
            return True, [x, y, z, ang], name

        except Exception as e:
            self.get_logger().error(f"❌ Seg Service Exception: {e}")
            return False, None, ""

    # ---------------------------------------------------------
    # Detection Loop: 중앙 우선 선택 로직 (Winner Takes All)
    # ---------------------------------------------------------
    def try_segmentation_with_retry(self, goal_handle):
        CLASS_TO_ZONE = {
            "nike": 1, "adidas": 2, "newbalance": 3, "new_balance": 3
        }
        
        center_x = self.intrinsics.get('ppx', 320.0)
        center_y = self.intrinsics.get('ppy', 240.0)
        
        # 감지할 대상 목록
        targets_to_check = ["nike", "adidas", "newbalance"]
        
        for attempt in range(1, MAX_RETRY_ATTEMPTS + 1):
            self._send_feedback(goal_handle, f"Scanning Round ({attempt}/{MAX_RETRY_ATTEMPTS})...")
            
            stabilize_sec = CAMERA_STABILIZATION_TIME if attempt == 1 else 1.5
            time.sleep(stabilize_sec)

            # Warmup (첫 시도만)
            if attempt == 1:
                self.call_segmentation_service("nike", 1)
                time.sleep(0.5)

            candidates = []

            # 1. 모든 클래스 스캔
            for cls_name in targets_to_check:
                ok, res, det_name = self.call_segmentation_service(cls_name, 1)
                
                if ok:
                    pixel_x, pixel_y = res[0], res[1]
                    # 화면 중앙과의 거리 계산
                    dist_sq = (pixel_x - center_x)**2 + (pixel_y - center_y)**2
                    
                    final_cls = det_name if det_name else cls_name
                    final_zone = CLASS_TO_ZONE.get(final_cls.lower(), 1)
                    
                    candidates.append({
                        'cls': final_cls,
                        'zone': final_zone,
                        'data': res,
                        'dist': dist_sq
                    })
                    self.get_logger().info(f"   -> Found {final_cls} (Dist: {dist_sq:.0f})")

            # 2. 가장 중앙에 가까운 것 선택 (Winner Takes All)
            if candidates:
                candidates.sort(key=lambda x: x['dist'])
                best = candidates[0]
                self.get_logger().info(f"🏆 Best Target Selected: {best['cls']} (Dist: {best['dist']:.0f})")
                return best['cls'], best['zone'], best['data']
            
            # 3. 실패 시 재시도 모션
            self.get_logger().warn("⚠️ No objects detected.")
            if attempt < MAX_RETRY_ATTEMPTS:
                self._send_feedback(goal_handle, "Retry: Repositioning...")
                movej(HOME_POSITION, vel=VELOCITY, acc=ACC); mwait()
                time.sleep(RETRY_WAIT_TIME)
                movej(TRY_ON_DETECTION_POSITION, vel=VELOCITY, acc=ACC); mwait()

        return None, None, None

    # ---------------------------------------------------------
    # PICK 동작 (+ Arduino Control)
    # ---------------------------------------------------------
    def execute_pick(self, cls_name, seg_data, goal_handle):
        global movel, movej, mwait, gripper, get_current_posx, posx

        px, py, depth, _ = seg_data
        
        config = CLEAR_POSITION_CONFIGS.get(cls_name, {})
        off_x = config.get("offset_x", 0)
        off_y = config.get("offset_y", 0)
        
        final_px = px + off_x
        final_py = py + off_y
        depth = config.get("fixed_depth", depth)
        
        self.get_logger().info(f"[PICK] Pixel: ({px:.1f}, {py:.1f})")

        # 좌표 변환
        cam_pos = get_camera_pos(final_px, final_py, depth, self.intrinsics)
        cur_robot_pos = get_current_posx()
        robot_coords = transform_to_base(cam_pos, cur_robot_pos[0], self.gripper2cam)
        
        target_x, target_y, calculated_z = robot_coords
        target_z = calculated_z - Z_OFFSET

        curr_rot = cur_robot_pos[0][3:] 

        # Motion Sequence
        self._send_feedback(goal_handle, "Approach (Safe Point)")
        movej(TRY_ON_SAFE_APPROACH_JOINT, vel=VELOCITY, acc=ACC); mwait()

        pass_1 = posj([-49.567, 35.595, 39.198, 0.76, 104.311, -138.159])
        movej(pass_1, vel=VELOCITY, acc=ACC); mwait()

        self._send_feedback(goal_handle, "Gripper Open")
        gripper.open_gripper(); time.sleep(0.5)

        self._send_feedback(goal_handle, "Descending")
        pick_pos = posx([target_x, target_y, target_z, *curr_rot])
        movel(pick_pos, vel=PICK_VEL, acc=PICK_ACC); mwait()

        self._send_feedback(goal_handle, "Gripping")
        gripper.close_gripper(); time.sleep(0.8)

        self._send_feedback(goal_handle, "Lifting")
        lift_pos = posx([target_x, target_y, target_z + LIFT_HEIGHT, *curr_rot])
        movel(lift_pos, vel=VELOCITY, acc=ACC); mwait()

        self._send_feedback(goal_handle, "Returning to Safe Point")
        movej(TRY_ON_SAFE_APPROACH_JOINT, vel=VELOCITY, acc=ACC); mwait()
        movej(HOME_POSITION, vel=VELOCITY, acc=ACC); mwait()
        
        # ⭐⭐⭐ [Pick 완료 후] 턴테이블 고객 쪽으로 회전 ⭐⭐⭐
        self._send_feedback(goal_handle, "Rotating turntable to Customer...")
        self.call_arduino_to_customer()
        
        return True

    # ---------------------------------------------------------
    # PLACE 동작
    # ---------------------------------------------------------
    def execute_place(self, zone, goal_handle):
        global movej, movel, mwait, gripper
        
        if zone not in CLEAR_PLACE_POSITIONS:
            raise RuntimeError(f"Unknown Zone: {zone}")
            
        config = CLEAR_PLACE_POSITIONS[zone]
        
        self.get_logger().info(f"[PLACE] Target Zone: {zone}")
        self._send_feedback(goal_handle, f"Moving to Zone {zone}...")
        
        movej(config["approach_joint"], vel=VELOCITY, acc=ACC); mwait()
        
        self._send_feedback(goal_handle, "Descending to Place...")
        movel(config["place_cartesian"], vel=VELOCITY, acc=ACC); mwait()
        
        self._send_feedback(goal_handle, "Releasing...")
        gripper.open_gripper(); time.sleep(1.0)
        
        self._send_feedback(goal_handle, "Lifting...")
        movej(config["approach_joint"], vel=VELOCITY, acc=ACC); mwait()
        
        self._send_feedback(goal_handle, "Returning to HOME...")
        movej(HOME_POSITION, vel=VELOCITY, acc=ACC); mwait()

    # ---------------------------------------------------------
    # Main Callback
    # ---------------------------------------------------------
    async def execute_callback(self, goal_handle):
        global movej, mwait, gripper
        result = PickAndPlace.Result()
        
        self.get_logger().info("🚀 Clear Action Started!")
        
        try:
            # ⭐⭐⭐ [시작 직후] 턴테이블 로봇 쪽으로 회전 ⭐⭐⭐
            self._send_feedback(goal_handle, "Rotating turntable to Robot...")
            self.call_arduino_to_robot()

            movej(HOME_POSITION, vel=VELOCITY, acc=ACC); mwait()
            
            self._send_feedback(goal_handle, "Moving to Detection View...")
            movej(TRY_ON_DETECTION_POSITION, vel=VELOCITY, acc=ACC); mwait()
            
            if not gripper.get_status()[0]: 
                gripper.open_gripper(); time.sleep(0.5)

            # 중앙 우선 탐색 (Winner Takes All)
            cls, zone, data = self.try_segmentation_with_retry(goal_handle)
            if not cls: raise RuntimeError("No shoe detected")

            self._send_feedback(goal_handle, f"Picking {cls}...")
            # Pick 내부에서 마지막에 턴테이블 원복(to Customer) 호출됨
            if not self.execute_pick(cls, data, goal_handle):
                raise RuntimeError("Pick Failed")

            self._send_feedback(goal_handle, f"Placing to Zone {zone}...")
            self.execute_place(zone, goal_handle)

            movej(CLEAR_FINAL_POSITION, vel=VELOCITY, acc=ACC); mwait()

            result.success_message = f"Cleared: {cls} -> Zone {zone}"
            goal_handle.succeed()
            self.get_logger().info("✅ Clear Action Complete")
            return result

        except Exception as e:
            self.get_logger().error(f"❌ Error: {e}")
            try: movej(HOME_POSITION, vel=VELOCITY, acc=ACC); mwait()
            except: pass
            
            result.success_message = str(e)
            goal_handle.abort()
            return result


# ==========================================================
# MAIN EXECUTION
# ==========================================================
def main(args=None):
    rclpy.init(args=args)
    
    tmp_node = rclpy.create_node("dsr_node_clear_comm", namespace=ROBOT_ID)
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = tmp_node
    
    global movej, movel, mwait, gripper, get_current_posx, posx
    from DSR_ROBOT2 import movej as mj, movel as ml, mwait as mw, get_current_posx as gcp
    from DR_common2 import posx as px
    
    movej, movel, mwait, get_current_posx, posx = mj, ml, mw, gcp, px
    
    gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)
    
    node = ClearActionServer()
    
    executor = MultiThreadedExecutor()
    executor.add_node(tmp_node) 
    executor.add_node(node)

    try:
        node.get_logger().info("Clear Action Server Spinning...")
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        tmp_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()