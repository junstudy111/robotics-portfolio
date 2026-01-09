#!/usr/bin/env python3
import time
import numpy as np
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from scipy.spatial.transform import Rotation

from my_pkg.aruco_detector_pick_grip import ArucoDetector, GripState


class Phase(Enum):
    ARM_SAFE = 0
    NAVIGATE = 1
    WAIT_NAV = 2
    START_PICKUP = 3
    WAIT_PICKUP_DONE = 4
    PLACE_MOVE = 5
    RELEASE = 6
    DONE = 7


class ScenarioRunner(Node):
    def __init__(self, aruco_node: ArucoDetector):
        super().__init__("scenario_runner")

        self.aruco = aruco_node

        # ─────────────────────────────────────────
        # Isaac Sim bridge publishers
        # ─────────────────────────────────────────
        self.arm_pub = self.create_publisher(PoseStamped, "/rmp_target_pose", 10)
        self.gripper_pub = self.create_publisher(String, "/gripper_command", 10)

        # ─────────────────────────────────────────
        # Nav2
        # ─────────────────────────────────────────
        self.nav = BasicNavigator()

        init_pose = PoseStamped()
        init_pose.header.frame_id = "map"
        init_pose.pose.orientation.w = 1.0
        self.nav.waitUntilNav2Active()

        # ─────────────────────────────────────────
        # Targets
        # ─────────────────────────────────────────
        self.nav_target = PoseStamped()
        self.nav_target.header.frame_id = "map"
        self.nav_target.pose.position.x = 23.7
        self.nav_target.pose.position.y = -2.2

        self.nav_target.pose.orientation.w = 1.0

        # Arm poses (base_link 기준, meters)
        self.arm_safe = np.array([0.45, 0.0, 0.7], dtype=float)

        self.place_pos = np.array([0.2, 0.0, 1.0], dtype=float)

        # Arm orientation (그리퍼가 바닥 보는 방향)
        self.arm_quat = Rotation.from_euler("xyz", [0, np.pi / 2, 0]).as_quat()  # x,y,z,w

        # 타이밍
        self.wait_after_arm_safe = 2.0
        self.wait_after_place_move = 2.0

        # 상태 머신
        self.phase = Phase.ARM_SAFE
        self._next_time = 0.0

        self.get_logger().info("🔥 ScenarioRunner started (nav -> pickup -> place -> release)")

        # 10Hz tick
        self.create_timer(0.1, self.tick)

    # ─────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────
    def publish_arm_pose(self, pos_xyz: np.ndarray):
        msg = PoseStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.position.x = float(pos_xyz[0])
        msg.pose.position.y = float(pos_xyz[1])
        msg.pose.position.z = float(pos_xyz[2])

        msg.pose.orientation.x = float(self.arm_quat[0])
        msg.pose.orientation.y = float(self.arm_quat[1])
        msg.pose.orientation.z = float(self.arm_quat[2])
        msg.pose.orientation.w = float(self.arm_quat[3])

        self.arm_pub.publish(msg)

    def publish_gripper(self, cmd: str):
        m = String()
        m.data = cmd
        self.gripper_pub.publish(m)

    # ─────────────────────────────────────────
    # State machine tick
    # ─────────────────────────────────────────
    def tick(self):
        now = time.time()

        # 1) ARM_SAFE
        if self.phase == Phase.ARM_SAFE:
            self.get_logger().info("🦾 ARM_SAFE")
            self.publish_arm_pose(self.arm_safe)
            self.phase = Phase.NAVIGATE
            self._next_time = now + self.wait_after_arm_safe
            return

        # 2) NAVIGATE (after small delay)
        if self.phase == Phase.NAVIGATE:
            if now < self._next_time:
                return
            self.get_logger().info("🧭 NAV: goToPose")
            self.nav.goToPose(self.nav_target)
            self.phase = Phase.WAIT_NAV
            return

        # 3) WAIT_NAV
        if self.phase == Phase.WAIT_NAV:
            if not self.nav.isTaskComplete():
                fb = self.nav.getFeedback()
                if fb:
                    self.get_logger().info(f"📍 distance remaining: {fb.distance_remaining:.2f}")
                return

            result = self.nav.getResult()
            if result != TaskResult.SUCCEEDED:
                self.get_logger().error("❌ Nav failed")
                self.phase = Phase.DONE
                return

            self.get_logger().info("✅ Nav done")
            self.phase = Phase.START_PICKUP
            return

        # 4) START_PICKUP
        if self.phase == Phase.START_PICKUP:
            self.get_logger().info("🎯 Start ArUco pickup sequence (enable detection)")
            self.aruco.detection_enabled = True
            self.phase = Phase.WAIT_PICKUP_DONE
            return

        # 5) WAIT_PICKUP_DONE
        if self.phase == Phase.WAIT_PICKUP_DONE:
            # ✅ ArUcoDetector가 COMPLETE로 끝나면, 이제 place로 넘어감
            if self.aruco.state == GripState.COMPLETE:
                self.get_logger().info("✅ PICKUP complete → PLACE_MOVE")
                self.phase = Phase.PLACE_MOVE
                self._next_time = now  # 바로 실행
            return

        # 6) PLACE_MOVE
        if self.phase == Phase.PLACE_MOVE:
            if now < self._next_time:
                return
            self.get_logger().info("📦 PLACE_MOVE: move arm to (0.2, 0.0, 1.0)")
            self.publish_arm_pose(self.place_pos)
            self.phase = Phase.RELEASE
            self._next_time = now + self.wait_after_place_move
            return

        # 7) RELEASE
        if self.phase == Phase.RELEASE:
            if now < self._next_time:
                return
            self.get_logger().info("🖐️ RELEASE: open gripper")
            self.publish_gripper("open")
            self.phase = Phase.DONE
            return

        # 8) DONE
        if self.phase == Phase.DONE:
            pass


def main():
    rclpy.init()

    # ArUco node
    aruco_node = ArucoDetector()

    # 안전하게 시작 OFF (nav 끝나고 runner가 켬)
    aruco_node.detection_enabled = False

    runner = ScenarioRunner(aruco_node)

    ex = MultiThreadedExecutor()
    ex.add_node(aruco_node)
    ex.add_node(runner)

    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown()
        aruco_node.destroy_node()
        runner.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
