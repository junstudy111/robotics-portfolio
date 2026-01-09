# return_action_server.py
# =============================================================================
# 기능: QR 인식(브랜드 식별) -> Box 피킹(Offset 적용) -> 브랜드별 위치(5/6번)로 반납
# =============================================================================

### 지금 1차 return 맞음

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
# DSR + Gripper
# ==========================================================
import DR_init
from robot_control.onrobot import RG

# ==========================================================
# ROS Interfaces
# ==========================================================
from od_msg.srv import SrvDepthPosition
from od_msg.action import PickAndPlace

# ==========================================================
# DSR Coordinate Systems
# ==========================================================
from DR_common2 import posj, posx

# ==========================================================
# Configurations
# ==========================================================
# ⭐ CLEAR_PLACE_POSITIONS 추가됨 (브랜드별 반납 위치)
from robot_control.module.seg_pick_config import (
    ROBOT_ID, ROBOT_MODEL, VELOCITY, ACC,
    HOME_POSITION, 
    DROP_POSITIONS,          # Zone B (감지 위치)
    RETURN_POSITION_CONFIGS, # Box 피킹 Offset
    CLEAR_PLACE_POSITIONS,   # ⭐ [NEW] 반납 좌표 (5:Nike, 6:Adidas)
    APPROACH_HEIGHT, LIFT_HEIGHT,
    PICK_VEL, PICK_ACC,
    GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT,
    Z_OFFSET
)

# ==========================================================
# Utilities
# ==========================================================
from robot_control.module.seg_pick_utils import (
    get_camera_pos,
    transform_to_base
)

# ==========================================================
# Global Robot Functions
# ==========================================================
movej = movel = mwait = gripper = get_current_posx = None


class ReturnActionServer(Node):

    def __init__(self):
        super().__init__("return_action_server")
        self.callback_group = ReentrantCallbackGroup()
        
        self.get_logger().info("Initializing ReturnActionServer...")

        # ---------------------------------------------------------
        # 1. Load Hand-Eye Matrix
        # ---------------------------------------------------------
        pkg_path = get_package_share_directory("pick_and_place_voice")
        mat_path = os.path.join(pkg_path, "resource", "T_gripper2camera.npy")

        if not os.path.exists(mat_path):
            raise RuntimeError(f"Matrix missing: {mat_path}")
        self.gripper2cam = np.load(mat_path)

        # ---------------------------------------------------------
        # 2. Get Camera Intrinsics
        # ---------------------------------------------------------
        self.intrinsics = {
            'fx': 615.0, 'fy': 615.0,
            'cx': 320.0, 'cy': 240.0,
            'ppx': 320.0, 'ppy': 240.0
        }
        try:
            from object_detection.realsense import ImgNode
            tmp = ImgNode()
            time.sleep(0.5)
            intr = tmp.get_camera_intrinsic()
            if intr:
                self.intrinsics = intr
                if 'ppx' not in intr and 'cx' in intr:
                    self.intrinsics['ppx'] = intr['cx']
                    self.intrinsics['ppy'] = intr['cy']
            tmp.destroy_node()
        except Exception as e:
            self.get_logger().warn(f"Using default intrinsics due to: {e}")

        # ---------------------------------------------------------
        # 3. Segmentation Service Client
        # ---------------------------------------------------------
        self.seg_client = self.create_client(
            SrvDepthPosition,
            "/segmentation_pick",
            callback_group=self.callback_group
        )

        while not self.seg_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for /segmentation_pick service...")
        
        self.get_logger().info("✓ Connected to Segmentation Service")

        # ---------------------------------------------------------
        # 4. Robot Initialization
        # ---------------------------------------------------------
        try:
            movej(HOME_POSITION, vel=VELOCITY, acc=ACC); mwait()
        except Exception:
            pass

        # ---------------------------------------------------------
        # 5. Action Server
        # ---------------------------------------------------------
        self._action_server = ActionServer(
            self,
            PickAndPlace,
            "return_action",
            self.execute_callback,
            callback_group=self.callback_group
        )
        self.get_logger().info("✅ RETURN Action Server Ready")
        self.get_logger().info("   • Sorting Strategy:")
        self.get_logger().info("     - NIKE   -> Position 5")
        self.get_logger().info("     - ADIDAS -> Position 6")

    # =========================================================================
    # Helper Methods
    # =========================================================================
    def send_fb(self, handle, text):
        fb = PickAndPlace.Feedback()
        fb.status_message = text
        handle.publish_feedback(fb)
        self.get_logger().info(f"[FB] {text}")

    def call_service(self, target_name):
        req = SrvDepthPosition.Request()
        req.target = target_name

        future = self.seg_client.call_async(req)
        start_time = time.time()
        
        while not future.done():
            if time.time() - start_time > 10.0:
                self.get_logger().error("Service call timed out")
                return False, None, ""
            time.sleep(0.01)

        try:
            res = future.result()
            # 0.0 좌표라도 이름(shoe_name)은 올 수 있음 (QR의 경우)
            if target_name == "QR":
                 if res.shoe_name == "NO_QR":
                     return False, None, "NO_QR"
                 return True, res.depth_position, res.shoe_name
            
            # Box 인식의 경우 좌표가 중요
            if all(v == 0.0 for v in res.depth_position):
                return False, None, res.shoe_name
            return True, res.depth_position, res.shoe_name

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return False, None, ""

    # =========================================================================
    # Pick Logic (Offset Applied)
    # =========================================================================
    def execute_pick(self, px_data):
        raw_px, raw_py, depth, angle = px_data 

        # 반품 전용 Offset (box)
        config = RETURN_POSITION_CONFIGS.get("box", {})
        off_x = config.get("offset_x", 0)
        off_y = config.get("offset_y", 0)
        
        final_px = raw_px + off_x
        final_py = raw_py + off_y
        
        self.get_logger().info(f"[OFFSET] Apply: ({off_x}, {off_y})")

        cam_pos = get_camera_pos(final_px, final_py, depth, self.intrinsics)
        cur_pos = get_current_posx()
        base_pos = transform_to_base(cam_pos, cur_pos[0], self.gripper2cam)

        t_x, t_y, c_z = base_pos
        c_z = config.get("fixed_depth",0)
        t_z = c_z - Z_OFFSET
        rot = cur_pos[0][3:]

        # Approach -> Pick -> Lift
        approach_pos = posx([t_x, t_y, t_z + APPROACH_HEIGHT, *rot])
        movel(approach_pos, vel=VELOCITY, acc=ACC); mwait()

        pick_pos = posx([t_x, t_y, t_z, *rot])
        movel(pick_pos, vel=PICK_VEL, acc=PICK_ACC); mwait()

        gripper.close_gripper()
        time.sleep(1.0)


        movej(DROP_POSITIONS['B'], vel=VELOCITY, acc=ACC); mwait()

    # =========================================================================
    # ⭐ Place Logic (Brand Sorting)
    # =========================================================================
    def execute_place_sorted(self, brand_name, goal_handle):
        """브랜드에 따라 5번(Nike) 또는 6번(Adidas) 위치에 반납"""
        
        brand = brand_name.lower()
        target_id = None
        
        # QR 데이터에 따른 분기
        if "nike" in brand:
            target_id = 5
        elif "adidas" in brand:
            target_id = 6
        else:
            # QR은 읽었으나 nike/adidas가 아닌 경우 -> 기본값(Nike 자리 5번)
            self.get_logger().warn(f"Unknown brand '{brand}'. Defaulting to Nike(5).")
            target_id = 5

        # Config에서 좌표 가져오기
        place_config = CLEAR_PLACE_POSITIONS.get(target_id)
        if not place_config:
            raise RuntimeError(f"No config for place ID {target_id}")

        self.send_fb(goal_handle, f"Moving to Place: {brand.upper()} (ID:{target_id})")

        # 0. 특이점 해피
        movej(HOME_POSITION, vel=VELOCITY, acc=ACC); mwait()
        pass_the_simularity = place_config["dodge_action"]
        app_joint = place_config["approach_joint"]
        movej(pass_the_simularity, vel=VELOCITY, acc=ACC); mwait()
        

        # 1. Approach (Joint Move) - 안전하게 근처로 이동
        app_joint = place_config["approach_joint"]
        movej(app_joint, vel=VELOCITY, acc=ACC); mwait()

        # 2. Place Descent (Linear Move) - 수직 하강
        place_cart = place_config["place_cartesian"]
        movel(place_cart, vel=PICK_VEL, acc=PICK_ACC); mwait()

        # 3. Release
        gripper.open_gripper()
        time.sleep(1.0)

        # 4. Retreat (Joint Move) - 다시 안전 높이로 복귀
        movej(pass_the_simularity, vel=VELOCITY, acc=ACC); mwait()

    # =========================================================================
    # Main Callback
    # =========================================================================
    async def execute_callback(self, goal_handle):
        self.get_logger().info("🚀 RETURN Process Started")
        result = PickAndPlace.Result()

        try:
            # 복귀
            movej(HOME_POSITION, vel=VELOCITY, acc=ACC); mwait()            
            gripper.open_gripper()
            time.sleep(1.0)            
            # 1. 감지 위치 이동 (Zone B)
            self.send_fb(goal_handle, "Moving to Detect Zone B...")
            movej(DROP_POSITIONS['B'], vel=VELOCITY, acc=ACC); mwait()
            time.sleep(0.8)

            # -----------------------------------------------------------------
            # 2. QR Scan (브랜드 식별)
            # -----------------------------------------------------------------
            qr_found = False
            brand_name = "unknown"
            
            for i in range(5):
                self.send_fb(goal_handle, f"Scanning QR ({i+1}/5)...")
                # QR 요청 -> Segmentation Service에서 "nike" or "adidas"를 받아옴
                ok, _, name = self.call_service("QR")
                
                if ok and name != "NO_QR" and name != "":
                    brand_name = name
                    qr_found = True
                    self.send_fb(goal_handle, f"QR Found: {brand_name}")
                    break
                time.sleep(0.5)

            if not qr_found:
                self.send_fb(goal_handle, "QR Fail -> HOME")
                movej(HOME_POSITION, vel=VELOCITY, acc=ACC); mwait()
                result.success_message = "FAIL: NO_QR"
                goal_handle.succeed()
                return result

            # -----------------------------------------------------------------
            # 3. Box Detect & Pick
            # -----------------------------------------------------------------
            self.send_fb(goal_handle, "Detecting Box...")
            ok, box_data, _ = self.call_service("box") # box ID:6
            
            if not ok:
                result.success_message = "FAIL: Box Not Detected"
                goal_handle.abort()
                return result

            self.execute_pick(box_data)

            # -----------------------------------------------------------------
            # 4. Sorted Place (브랜드별 반납)
            # -----------------------------------------------------------------
            # ⭐ 받아온 brand_name을 이용해 5번/6번 자리로 이동
            movej(HOME_POSITION, vel=VELOCITY, acc=ACC); mwait()
            self.execute_place_sorted(brand_name, goal_handle)

            # 복귀
            movej(HOME_POSITION, vel=VELOCITY, acc=ACC); mwait()

            self.get_logger().info("✅ Return Complete")
            result.success_message = f"Success: Returned {brand_name}"
            goal_handle.succeed()
            return result

        except Exception as e:
            self.get_logger().error(f"Critical Error: {e}")
            result.success_message = f"Error: {str(e)}"
            goal_handle.abort()
            return result


def main(args=None):
    rclpy.init(args=args)

    dsr_node = rclpy.create_node("dsr_return_comm", namespace=ROBOT_ID)
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = dsr_node

    global movej, movel, mwait, gripper, get_current_posx, posx
    from DSR_ROBOT2 import movej as mj, movel as ml, mwait as mw, get_current_posx as gcp
    from DR_common2 import posx as px

    movej, movel, mwait, get_current_posx, posx = mj, ml, mw, gcp, px
    gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

    server = ReturnActionServer()

    executor = MultiThreadedExecutor()
    executor.add_node(server)
    executor.add_node(dsr_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        dsr_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()