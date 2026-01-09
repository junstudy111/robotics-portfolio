import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation
from enum import Enum


class GripState(Enum):
    IDLE = 0
    APPROACH = 1          # 위쪽으로 접근
    DESCEND = 2           # 타겟으로 하강
    GRIP = 3              # 그리퍼 닫기
    LIFT = 4              # 들어올리기
    RETREAT_UP = 5        # 픽업 후 안정화(수직 상승)
    PLACE_MOVE = 6        # 놓을 위치로 이동
    HOLD_BEFORE_RELEASE = 7  # 흔들림 가라앉히기
    RELEASE = 8           # 그리퍼 열기
    COMPLETE = 9          # 완료


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector_staged')

        # 마커 및 검출기 설정
        self.marker_size = 0.13
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters_create()
        self.bridge = CvBridge()

        # TF 리스너
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 카메라 구독
        self.create_subscription(Image, '/front_stereo_camera/left/image_raw', self.image_callback, 10)
        self.create_subscription(CameraInfo, '/front_stereo_camera/left/camera_info', self.info_callback, 10)

        # RMPFlow 타겟 퍼블리셔
        self.pose_pub = self.create_publisher(PoseStamped, '/rmp_target_pose', 10)

        # 그리퍼 명령 퍼블리셔
        self.gripper_pub = self.create_publisher(String, '/gripper_command', 10)

        self.camera_matrix = None
        self.dist_coeffs = None

        # 그리퍼가 바닥을 향하는 orientation
        euler = np.array([0, np.pi/2, 0])  # roll, pitch, yaw
        rot = Rotation.from_euler('xyz', euler)
        self.default_quat = rot.as_quat()  # [x, y, z, w]

        # USD 센티미터 단위 -> 미터 변환
        self.usd_scale = 0.01

        # ArUco 마커 -> 그리퍼 목표 위치 오프셋 (USD 센티미터 단위)
        offset_x_cm = 4.2
        offset_y_cm = 0.0
        offset_z_cm = 2.9

        # 미터 단위로 변환
        self.offset_x = offset_x_cm * self.usd_scale
        self.offset_y = offset_y_cm * self.usd_scale
        self.offset_z = offset_z_cm * self.usd_scale

        # 단계별 높이 설정
        self.approach_height = 0.15  # 목표 위 15cm에서 접근
        self.lift_height = 0.10      # 잡은 후 10cm 들어올리기

        # ✅ 안정화 / 놓기 파라미터
        self.retreat_up_dz = 0.15                   # 픽업 후 위로 더 올려서 안정화
        self.place_position = np.array([0.4, 0.0, 0.8], dtype=float)  # 놓을 위치
        self.hold_before_release_ticks = 30         # 0.1s timer 기준 3초? -> 30이면 3초(0.1s*30)

        # State Machine
        self.state = GripState.IDLE
        self.target_position = None
        self.lift_position = None      # LIFT 때 실제로 간 위치 저장
        self.retreat_position = None   # RETREAT 위치 저장

        self.state_timer = 0

        self.wait_duration = 30

        # 마커 감지 제어
        self.detection_enabled = True

        self.get_logger().info("Ready for ArUco detection with staged approach.")
        self.get_logger().info(f"Offset (cm): X={offset_x_cm}, Y={offset_y_cm}, Z={offset_z_cm}")
        self.get_logger().info(f"Offset (m):  X={self.offset_x:.4f}, Y={self.offset_y:.4f}, Z={self.offset_z:.4f}")
        self.get_logger().info(f"Approach height: {self.approach_height}m, Lift height: {self.lift_height}m")
        self.get_logger().info(f"Retreat dz: {self.retreat_up_dz}m, Place pos: {self.place_position.tolist()}")

        # 타이머로 주기적으로 state machine 실행
        self.create_timer(0.1, self.state_machine_update)

    def info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        if self.camera_matrix is None or not self.detection_enabled:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.params)

        if ids is not None and self.state == GripState.IDLE:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )

            for i in range(len(ids)):
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.1)

                try:
                    target_frame = "base_link"
                    source_frame = "front_stereo_camera_left_rgb"

                    p_cam = PoseStamped()
                    p_cam.header.frame_id = source_frame
                    p_cam.header.stamp = msg.header.stamp
                    p_cam.pose.position.x = float(tvecs[i][0][0])
                    p_cam.pose.position.y = float(tvecs[i][0][1])
                    p_cam.pose.position.z = float(tvecs[i][0][2])
                    p_cam.pose.orientation.w = 1.0

                    transform = self.tf_buffer.lookup_transform(
                        target_frame,
                        source_frame,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.1)
                    )

                    p_robot_pose = tf2_geometry_msgs.do_transform_pose(p_cam.pose, transform)

                    marker_x = p_robot_pose.position.x
                    marker_y = p_robot_pose.position.y
                    marker_z = p_robot_pose.position.z

                    final_x = marker_x + self.offset_x
                    final_y = marker_y + self.offset_y
                    final_z = marker_z + self.offset_z

                    self.get_logger().info(
                        f"ID {ids[i][0]}: Final Target -> X:{final_x:.3f}m, Y:{final_y:.3f}m, Z:{final_z:.3f}m"
                    )

                    self.target_position = np.array([final_x, final_y, final_z], dtype=float)
                    self.state = GripState.APPROACH
                    self.state_timer = 0
                    self.detection_enabled = False

                    self.get_logger().info("🎯 Target acquired! Starting approach sequence...")
                    break

                except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
                    continue

        cv2.imshow("Aruco View", frame)
        cv2.waitKey(1)

    def state_machine_update(self):
        if self.state == GripState.IDLE:
            return

        self.state_timer += 1

        if self.state == GripState.APPROACH:
            if self.state_timer == 1:
                approach_pos = self.target_position.copy()
                approach_pos[2] += self.approach_height
                self.publish_target_pose(approach_pos)
                self.get_logger().info(
                    f"📍 APPROACH: X:{approach_pos[0]:.3f}, Y:{approach_pos[1]:.3f}, Z:{approach_pos[2]:.3f}"
                )

            if self.state_timer >= self.wait_duration:
                self.state = GripState.DESCEND
                self.state_timer = 0
                self.get_logger().info("⬇️  DESCEND: Lowering to target...")

        elif self.state == GripState.DESCEND:
            if self.state_timer == 1:
                self.publish_target_pose(self.target_position)
                self.get_logger().info(
                    f"📍 DESCEND: X:{self.target_position[0]:.3f}, Y:{self.target_position[1]:.3f}, Z:{self.target_position[2]:.3f}"
                )

            if self.state_timer >= self.wait_duration:
                self.state = GripState.GRIP
                self.state_timer = 0
                self.get_logger().info("✊ GRIP: Closing gripper...")

        elif self.state == GripState.GRIP:
            if self.state_timer == 1:
                self.publish_gripper_command("close")

            if self.state_timer >= max(5, self.wait_duration // 2):
                self.state = GripState.LIFT
                self.state_timer = 0
                self.get_logger().info("⬆️  LIFT: Lifting object...")

        elif self.state == GripState.LIFT:
            if self.state_timer == 1:
                lift_pos = self.target_position.copy()
                lift_pos[2] += self.lift_height
                self.lift_position = lift_pos.copy()   # ✅ 저장
                self.publish_target_pose(lift_pos)
                self.get_logger().info(
                    f"📍 LIFT: X:{lift_pos[0]:.3f}, Y:{lift_pos[1]:.3f}, Z:{lift_pos[2]:.3f}"
                )

            if self.state_timer >= self.wait_duration:
                # ✅ 픽업 직후 불안정 → retreat_up 먼저
                self.state = GripState.RETREAT_UP
                self.state_timer = 0
                self.get_logger().info("⬆️  RETREAT_UP: Stabilize before moving to place...")

        elif self.state == GripState.RETREAT_UP:
            if self.state_timer == 1:
                # lift_position 기반으로 z만 추가 상승
                base = self.lift_position if self.lift_position is not None else self.target_position
                retreat = np.array(base, dtype=float)
                retreat[2] += self.retreat_up_dz
                self.retreat_position = retreat.copy()
                self.publish_target_pose(retreat)
                self.get_logger().info(
                    f"📍 RETREAT_UP: X:{retreat[0]:.3f}, Y:{retreat[1]:.3f}, Z:{retreat[2]:.3f}"
                )

            if self.state_timer >= self.wait_duration:
                self.state = GripState.PLACE_MOVE
                self.state_timer = 0
                self.get_logger().info("📦 PLACE_MOVE: Moving to place position...")

        elif self.state == GripState.PLACE_MOVE:
            if self.state_timer == 1:
                self.publish_target_pose(self.place_position)
                self.get_logger().info(
                    f"📍 PLACE_MOVE: X:{self.place_position[0]:.3f}, Y:{self.place_position[1]:.3f}, Z:{self.place_position[2]:.3f}"
                )

            if self.state_timer >= self.wait_duration:
                self.state = GripState.HOLD_BEFORE_RELEASE
                self.state_timer = 0
                self.get_logger().info("⏸️  HOLD_BEFORE_RELEASE: settling...")

        elif self.state == GripState.HOLD_BEFORE_RELEASE:
            # ✅ 별도 publish 없이 대기만
            if self.state_timer >= self.hold_before_release_ticks:
                self.state = GripState.RELEASE
                self.state_timer = 0
                self.get_logger().info("🖐️  RELEASE: Opening gripper...")

        elif self.state == GripState.RELEASE:
            if self.state_timer == 1:
                self.publish_gripper_command("open")

            if self.state_timer >= max(5, self.wait_duration // 2):
                self.state = GripState.COMPLETE
                self.state_timer = 0
                self.get_logger().info("✅ COMPLETE: Pick + Place finished!")

        elif self.state == GripState.COMPLETE:
            # 완료 후 다시 감지 가능하도록
            if self.state_timer >= 20:  # 2초 정도 쉬고 재시작 (0.1s*20)
                self.state = GripState.IDLE
                self.state_timer = 0
                self.detection_enabled = True
                self.target_position = None
                self.lift_position = None
                self.retreat_position = None
                self.get_logger().info("🔄 Ready for next detection...")

    def publish_target_pose(self, position):
        target_msg = PoseStamped()
        target_msg.header.frame_id = "base_link"
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.pose.position.x = float(position[0])
        target_msg.pose.position.y = float(position[1])
        target_msg.pose.position.z = float(position[2])

        target_msg.pose.orientation.x = float(self.default_quat[0])
        target_msg.pose.orientation.y = float(self.default_quat[1])
        target_msg.pose.orientation.z = float(self.default_quat[2])
        target_msg.pose.orientation.w = float(self.default_quat[3])

        self.pose_pub.publish(target_msg)

    def publish_gripper_command(self, command):
        msg = String()
        msg.data = command
        self.gripper_pub.publish(msg)


def main():
    rclpy.init()
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
