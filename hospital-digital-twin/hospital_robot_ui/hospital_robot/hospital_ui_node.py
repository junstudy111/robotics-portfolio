import threading
import os

import rclpy
from rclpy.node import Node

from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
import uvicorn

from ament_index_python.packages import get_package_share_directory

from hospital_robot.data import PATIENTS
from hospital_interfaces.srv import CreateTransportTask
from hospital_interfaces.msg import TransportStatus


class HospitalUINode(Node):
    def __init__(self):
        super().__init__("hospital_ui_node")

        # =====================================================
        # Service Client (UI → Task Manager)
        # =====================================================
        self.transport_client = self.create_client(
            CreateTransportTask,
            "/create_transport_task"
        )

        # =====================================================
        # Subscriber (Task Manager → UI)
        # =====================================================
        self.status_sub = self.create_subscription(
            TransportStatus,
            "/transport_status",
            self.on_transport_status,
            10
        )

        # =====================================================
        # FastAPI
        # =====================================================
        self.app = FastAPI()

        # 🔥 수정: static과 templates가 hospital_robot 폴더 안에 있는 경우
        base_dir = "/home/jun/hospital_ws/src/hospital_robot_ui/hospital_robot"
        static_dir = os.path.join(base_dir, "static")
        templates_dir = os.path.join(base_dir, "templates")
        
        # 디버그: 경로 확인
        print(f"🔍 Static directory: {static_dir}")
        print(f"🔍 Templates directory: {templates_dir}")
        print(f"🔍 Static exists: {os.path.exists(static_dir)}")
        print(f"🔍 Templates exists: {os.path.exists(templates_dir)}")
        
        self.app.mount(
            "/static",
            StaticFiles(directory=static_dir),
            name="static"
        )
        self.templates = Jinja2Templates(directory=templates_dir)

        # =====================================================
        # UI Data
        # =====================================================
        self.patients = PATIENTS

        # =====================================================
        # Routes
        # =====================================================
        @self.app.get("/", response_class=HTMLResponse)
        async def index(request: Request):
            return self.templates.TemplateResponse(
                "index.html",
                {"request": request}
            )

        @self.app.get("/api/patients")
        async def get_patients():
            return self.patients

        @self.app.post("/api/send_batch")
        async def send_batch(tasks: list[dict]):
            if not tasks:
                return {"status": "empty"}

            # 1. 서비스 여부와 상관없이 '무조건' 상태부터 변경!
            for task in tasks:
                sample_id = task.get("sample_id")
                for p in self.patients.values():
                    for s in p["samples"]:
                        if s["id"] == sample_id:
                            s["status"] = "이동중"
                            self.get_logger().info(f"🚚 테스트 모드: {sample_id} 상태를 '이동중'으로 변경")

            # 2. 서비스 노드가 있는지 체크 (없어도 위에서 이미 상태는 바뀜)
            if not self.transport_client.service_is_ready():
                self.get_logger().warn("🚫 로봇 노드가 없지만, UI 상태는 '이동중'으로 변경함")
                return {"status": "sent_test_mode"}

            # 3. 로봇 노드가 있다면 실제 서비스 호출 (이건 보너스)
            from_location = tasks[0].get("pickup_from", "")
            to_location = tasks[0].get("lab", "")

            request = CreateTransportTask.Request()
            request.from_location = from_location
            request.to_location = to_location
            future = self.transport_client.call_async(request)
            
            self.get_logger().info(f"📤 UI → Service 요청 전송 완료")
            return {"status": "sent"}

        # =====================================================
        # FastAPI Thread
        # =====================================================
        threading.Thread(
            target=lambda: uvicorn.run(
                self.app,
                host="0.0.0.0",
                port=8000,
                log_level="warning",
            ),
            daemon=True,
        ).start()

        self.get_logger().info("🖥️ Hospital UI Node started")

    # =====================================================
    # TransportStatus Callback
    # =====================================================
    def on_transport_status(self, msg: TransportStatus):
        self.get_logger().info(
            f"📥 TransportStatus 수신: {msg.status}"
        )

        if msg.status == "ARRIVED":
            for p in self.patients.values():
                for s in p["samples"]:
                    if s["status"] == "이동중":
                        s["status"] = "검사 완료"
                        self.get_logger().info(
                            f"🧪 검사 완료 처리: {s['id']}"
                        )


def main():
    rclpy.init()
    node = HospitalUINode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()