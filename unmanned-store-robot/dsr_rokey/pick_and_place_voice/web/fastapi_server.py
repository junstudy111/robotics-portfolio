import os
import threading
import socket
import uvicorn
import qrcode
import time
import cv2
import json
import requests # UI 업데이트용 (Self-POST)

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Body
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.middleware.cors import CORSMiddleware

# ROS2 Imports
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String  # ⭐ 기존 퍼블리셔용 메시지 타입
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from action_msgs.msg import GoalStatus

# Action Interface
from od_msg.action import PickAndPlace 


# ==========================================================
# 전역 변수
# ==========================================================
latest_jpeg_frame = None
ws_clients = []


# ==========================================================
# ROS2 Node (WebInterfaceNode)
# ==========================================================
class WebInterfaceNode(Node):
    def __init__(self):
        super().__init__('web_interface_node')

        self.bridge = CvBridge()
        self.callback_group = ReentrantCallbackGroup() # 멀티스레드 허용

        # ------------------------------------------------------
        # ⭐ [복구됨] 기존 문자열 퍼블리셔 (UI → 로봇 명령)
        # ------------------------------------------------------
        self.publisher_ = self.create_publisher(String, '/web_command', 10)

        # ------------------------------------------------------
        # 1. Action Clients 생성 (로봇 제어용)
        # ------------------------------------------------------
        self.bring_client = ActionClient(self, PickAndPlace, "bring_action", callback_group=self.callback_group)
        self.clear_client = ActionClient(self, PickAndPlace, "clear_action", callback_group=self.callback_group)
        self.purchase_client = ActionClient(self, PickAndPlace, "purchase_action", callback_group=self.callback_group)
        self.return_client = ActionClient(self, PickAndPlace, "return_action", callback_group=self.callback_group)

        # RealSense 이미지 구독
        self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info("📡 Web Interface Node Started (Action + Publisher)")

    def image_callback(self, msg):
        global latest_jpeg_frame
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ok, buffer = cv2.imencode('.jpg', img)
            if ok:
                latest_jpeg_frame = buffer.tobytes()
        except Exception as e:
            self.get_logger().error(f"이미지 변환 실패: {e}")

    # ------------------------------------------------------
    # ⭐ [복구됨] 기존 문자열 전송 함수
    # ------------------------------------------------------
    def send_command(self, text):
        msg = String()
        msg.data = text
        self.publisher_.publish(msg)
        self.get_logger().info(f"📡 Published to /web_command: {text}")

    # ------------------------------------------------------
    # 2. 웹 명령 처리 (Action Goal 전송 + 문자열 퍼블리시)
    # ------------------------------------------------------
    def handle_web_action(self, command, obj):
        self.get_logger().info(f"🖱️ Web Touch Command: command='{command}', object='{obj}'")

        # ⭐ 1) 기존 리스너들을 위해 토픽으로도 쏴줍니다.
        self.send_command(command)

        # ⭐ 2) 액션 서버 호출 시작
        target_client = None

        # 명령어 분기
        if command == "갖다줘":
            target_client = self.bring_client
        elif command == "구매":
            target_client = self.purchase_client
        elif command == "정리":
            target_client = self.clear_client
        elif command == "반품":
            target_client = self.return_client
        else:
            self.get_logger().warn(f"⚠️ Unknown command: {command}")
            return

        # 서버 연결 확인
        if not target_client.server_is_ready():
            self.get_logger().warn(f"⏳ Waiting for {command} server...")
            if not target_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error(f"❌ Server not ready: {command}")
                self.notify_ui_event("action_finished", "failed", "서버 연결 실패")
                return

        # Goal 메시지 생성 및 전송
        goal_msg = PickAndPlace.Goal()
        goal_msg.target_command = command
        goal_msg.target_object = obj if obj else ""

        future = target_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    # ------------------------------------------------------
    # 3. Action Callbacks
    # ------------------------------------------------------
    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"❌ Goal request failed: {e}")
            self.notify_ui_event("action_finished", "failed", "명령 전송 실패")
            return

        if not goal_handle.accepted:
            self.get_logger().warn("⚠️ Goal rejected by server")
            self.notify_ui_event("action_finished", "failed", "서버가 명령 거절")
            return

        self.get_logger().info("✅ Goal accepted. Processing...")
        
        # 🔥 UI에 "작업 시작" 알림 (카메라 켜기 위해)
        self.notify_ui_event("action_started", "success", "로봇이 움직입니다")

        # 결과 대기
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        try:
            result = future.result()
            status = result.status
            msg = result.result.success_message
        except Exception as e:
            self.get_logger().error(f"❌ Result retrieval failed: {e}")
            self.notify_ui_event("action_finished", "failed", "결과 수신 실패")
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"🎉 Success: {msg}")
            self.notify_ui_event("action_finished", "success", msg)
        else:
            self.get_logger().warn(f"❌ Action Failed: {msg}")
            self.notify_ui_event("action_finished", "failed", msg)

    # ------------------------------------------------------
    # 4. UI 알림 헬퍼 (Thread-Safe 방식)
    # ------------------------------------------------------
    def notify_ui_event(self, event_type, status, message=""):
        try:
            payload = {
                "event": event_type,
                "status": status,
                "message": message
            }
            # 자기 자신(FastAPI)에게 POST 전송
            requests.post("http://localhost:8000/voice_event", json=payload, timeout=0.2)
        except Exception:
            pass # 타임아웃 등 무시

    # ------------------------------------------------------
    # 5. 기존 WebSocket 브로드캐스트 (get_keyword.py 호환)
    # ------------------------------------------------------
    async def broadcast_voice_event(self, event: dict):
        dead = []
        for ws in ws_clients:
            try:
                await ws.send_json(event)
            except:
                dead.append(ws)
        for dc in dead:
            ws_clients.remove(dc)


# ==========================================================
# FastAPI 설정
# ==========================================================
app = FastAPI()
ros_node = None

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"], allow_credentials=True,
    allow_methods=["*"], allow_headers=["*"],
)

STATIC_DIR = "/home/rokey/ros2_ws/src/DoosanBootcamp3rd/dsr_rokey/pick_and_place_voice/web/static"
os.makedirs(STATIC_DIR, exist_ok=True)
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


@app.get("/")
async def root():
    return FileResponse(os.path.join(STATIC_DIR, "dashboard_final.html"))


@app.get("/video_feed")
async def video_feed():
    def generate_frames():
        global latest_jpeg_frame
        while True:
            if latest_jpeg_frame is not None:
                yield (b"--frame\r\n"
                       b"Content-Type: image/jpeg\r\n\r\n" + latest_jpeg_frame + b"\r\n")
            time.sleep(0.016)
    return StreamingResponse(generate_frames(), media_type="multipart/x-mixed-replace; boundary=frame")


# ==========================================================
# WebSocket Endpoint (JSON 수신 가능하도록 변경)
# ==========================================================
@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    ws_clients.append(ws)
    print("🔌 WebSocket Connected")

    try:
        while True:
            # 1. 텍스트인지 JSON인지 확인
            raw_data = await ws.receive_text()
            
            try:
                # 2. JSON 파싱 시도 (신규 로직)
                data = json.loads(raw_data)
                command = data.get("command")
                obj = data.get("object")

                if ros_node and command:
                    # 액션 실행 + 문자열 퍼블리시 모두 수행
                    ros_node.handle_web_action(command, obj)

            except json.JSONDecodeError:
                # 3. JSON이 아니면 단순 텍스트로 처리 (기존 로직 호환)
                # 예: 기존에 단순 문자열만 보내던 클라이언트 대응
                if ros_node:
                    ros_node.send_command(raw_data)

    except WebSocketDisconnect:
        print("⚠️ WebSocket Disconnected")
        ws_clients.remove(ws)
    except Exception as e:
        print(f"WS Error: {e}")
        if ws in ws_clients:
            ws_clients.remove(ws)


# ==========================================================
# UI 이벤트 수신 (get_keyword & Self-Post 공용)
# ==========================================================
@app.post("/voice_event")
async def voice_event(data: dict = Body(...)):
    # print(f"📡 UI Update: {data}")
    if ros_node:
        await ros_node.broadcast_voice_event(data)
    return {"ok": True}


# ==========================================================
# 실행부 (MultiThreadedExecutor 적용)
# ==========================================================
def run_ros():
    # 액션 클라이언트 콜백을 처리하려면 MultiThreadedExecutor 필수
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        ros_node.destroy_node()


def main():
    global ros_node

    rclpy.init()
    ros_node = WebInterfaceNode()

    # ROS 스레드 시작
    t = threading.Thread(target=run_ros, daemon=True)
    t.start()

    # IP 확인 및 QR 출력
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except:
        ip = "127.0.0.1"
    s.close()

    url = f"http://{ip}:8000"
    print(f"\n🚀 UI 서버 시작됨: {url}")

    qr = qrcode.QRCode()
    qr.add_data(url)
    qr.make(fit=True)
    qr.print_ascii(invert=True)

    try:
        uvicorn.run(app, host="0.0.0.0", port=8000)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()