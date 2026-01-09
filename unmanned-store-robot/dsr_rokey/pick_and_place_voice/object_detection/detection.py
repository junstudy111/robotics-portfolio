import numpy as np
import rclpy
from rclpy.node import Node
from typing import Any, Callable, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
from od_msg.srv import SrvDepthPosition
from object_detection.realsense import ImgNode
from object_detection.yolo import YoloModel


PACKAGE_NAME = 'pick_and_place_voice'
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)


class ObjectDetectionNode(Node):
    def __init__(self, model_name='yolo'):
        super().__init__('object_detection_node')
        self.img_node = ImgNode()
        self.model = self._load_model(model_name)
        self.intrinsics = self._wait_for_valid_data(
            self.img_node.get_camera_intrinsic, "camera intrinsics"
        )
        self.create_service(
            SrvDepthPosition,
            'get_3d_position',
            self.handle_get_depth
        )
        self.get_logger().info("ObjectDetectionNode initialized.")

    def _load_model(self, name):
        if name.lower() == 'yolo':
            return YoloModel()
        raise ValueError(f"Unsupported model: {name}")

    def handle_get_depth(self, request, response):
        """클라이언트 요청을 처리해 3D 좌표와 클래스명을 반환"""
        self.get_logger().info(f"Received request: {request}")

        if request.target:  # 목적어가 있으면 기존 메서드 사용
            coords, object_name = self._compute_position(request.target)
        else:  # 목적어가 없으면 화면에서 가장 높은 score 객체 사용
            coords, object_name = self._compute_position_overall()

        response.depth_position = [float(x) for x in coords]
        response.shoe_name = object_name
        return response

    # 목적어가 있을 때
    def _compute_position(self, target):
        rclpy.spin_once(self.img_node)
        box, score = self.model.get_best_detection(self.img_node, target)
        if box is None or score is None:
            return (0.0, 0.0, 0.0), target
        cx, cy = map(int, [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2])
        cz = self._get_depth(cx, cy)
        if cz is None:
            return (0.0, 0.0, 0.0), target
        coords = self._pixel_to_camera_coords(cx, cy, cz)
        return coords, target

    # 목적어 없이 전체 화면에서 최고 점수 객체를 찾을 때
    def _compute_position_overall(self):
        rclpy.spin_once(self.img_node)
        box, score, class_name = self.model.get_best_detection_overall(self.img_node)
        if box is None or score is None:
            print("실패1")
            return (0.0, 0.0, 0.0), "None"
        cx, cy = map(int, [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2])
        cz = self._get_depth(cx, cy)
        if cz is None:
            return (0.0, 0.0, 0.0), class_name
        coords = self._pixel_to_camera_coords(cx, cy, cz)
        return coords, class_name


    def _get_depth(self, x, y):
        frame = self._wait_for_valid_data(self.img_node.get_depth_frame, "depth frame")
        try:
            return frame[y, x]
        except IndexError:
            self.get_logger().warn(f"Coordinates ({x},{y}) out of range.")
            return None

    def _wait_for_valid_data(self, getter, description):
        data = getter()
        while data is None or (isinstance(data, np.ndarray) and not data.any()):
            rclpy.spin_once(self.img_node)
            self.get_logger().info(f"Retry getting {description}.")
            data = getter()
        return data

    def _pixel_to_camera_coords(self, x, y, z):
        fx = self.intrinsics['fx']
        fy = self.intrinsics['fy']
        ppx = self.intrinsics['ppx']
        ppy = self.intrinsics['ppy']
        return ( (x - ppx) * z / fx, (y - ppy) * z / fy, z )


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

