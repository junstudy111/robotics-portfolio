import os
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
from langchain.prompts import PromptTemplate
from voice_processing.MicController import MicController, MicConfig
from voice_processing.wakeup_word import WakeupWord
from voice_processing.stt import STT
import pyaudio
import re
import pygame
import threading

# 액션 클라이언트
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from od_msg.action import PickAndPlace

# TTS, 로고송
from voice_processing.tts import play_tts, play_logo_music
import requests

# ==========================================================
# 환경 변수 로드
# ==========================================================
package_path = get_package_share_directory("pick_and_place_voice")
load_dotenv(dotenv_path=os.path.join(f"{package_path}/resource/.env"))
openai_api_key = os.getenv("OPENAI_API_KEY")


# ==========================================================
# 🔥 GetKeyword Node
# ==========================================================
class GetKeyword(Node):
    def __init__(self):
        super().__init__("get_keyword_node")

        self.action_callback_group = ReentrantCallbackGroup()
        # ============================================================
        # 🔥 [핵심 변경 1] 노드 시작과 동시에 BGM 재생 (무한 루프)
        # ============================================================
        # volume을 작게(0.1 ~ 0.2) 설정하여 TTS가 잘 들리게 합니다.
        self.get_logger().info("🎶 Starting Background Music...")
        # 별도 스레드 필요 없이 호출만 하면 pygame 내부에서 비동기 재생됨
        play_logo_music(volume=0.5)

        # -------- LLM 초기화 --------
        self.llm = ChatOpenAI(
            model="gpt-4o",
            temperature=0.3,
            openai_api_key=openai_api_key
        )

        # -------- Prompt Template --------
        prompt_content = """
            사용자 명령에서 반드시 추출하세요:

             <목표>
            - 문장에서 신발 리스트에 정의된 신발을 최대한 정확하게 추출하세요.
            - 문장에서 행위 리스트에 정의된 행위도 함께 추출하세요.

            <신발 리스트>
            - nike, adidas, new_balance, nike_box, adidas_box, shoes

            <행위 리스트>
            - '갖다줘', '구매', '정리', '반품'

            <출력 형식>
            - 다음 형식을 반드시 따르세요: [command: 갖다줘 | object: nike]

            <특수 규칙>
            - 신발, 신발이, 아디 등과 같은 한국어 또는 줄임말 표현이 나오면 영어이면서 신발 리스트에 있는 목록으로 변환하여 출력하세요(예: "나이키 가져와" → nike, "아디다스박스 구매" → adidas_box).
            - 행위 리스트에 있는 명확한 표현이 아니더라도 문맥상 유추 가능한 경우("구매할게" → "구매", "사고싶어" → "구매", "신어볼게" → "갖다줘")
            - 다수의 도구와 행위가 동시에 등장할 경우 첫번째로 나온 도구와 첫번째로 나온 행위만 반환하세요.
            - 아무런 행위가 지정되지 않은 경우는 None으로 출력하세요.
            - "신발 정리", "신발 반품"처럼 '신발'만 언급된 경우 object는 shoes로 설정한다.
            - 입력 문장에 "신발"이라는 단어가 포함되어 있으면 반드시 object는 shoes 로 설정하세요.
        
            <예시>
            - 입력: "망치 메롱"
            출력: command: None | object: None

            - 입력: "정리"
            출력: command: 정리 | object: shoes

            - 입력: "나이키 갖다줘"
            출력: command: 갖다줘 | object: nike

            - 입력: "아디다스 가져와"
            출력: command: 갖다줘 | object: adidas

            - 입력: "나이키 박스 살게"
            출력: command: 구매 | object: nike

            - 입력: "신발 정리"
            출력: command: 정리 | object: 신발

            - 입력: "신발 반품"
            출력: command: 반품 | object: 신발

            <사용자 입력>
            "{user_input}"
        """

        self.prompt_template = PromptTemplate(
            input_variables=["user_input"],
            template=prompt_content
        )
        self.lang_chain = self.prompt_template | self.llm

        # -------- Mic 설정 --------
        self.mic_config = MicConfig(
            chunk=12000,
            rate=48000,
            channels=1,
            record_seconds=5,
            fmt=pyaudio.paInt16,
            device_index=0,
            buffer_size=24000,
        )

        self.mic_controller = MicController(self.mic_config)
        self.wakeup_word = None
        self.stt = STT(openai_api_key=openai_api_key)

        # -------- 액션 클라이언트 --------
        self.bring_client = ActionClient(self, PickAndPlace, "bring_action", callback_group=self.action_callback_group)
        self.clear_client = ActionClient(self, PickAndPlace, "clear_action", callback_group=self.action_callback_group)
        self.purchase_client = ActionClient(self, PickAndPlace, "purchase_action", callback_group=self.action_callback_group)
        self.return_client = ActionClient(self, PickAndPlace, "return_action", callback_group=self.action_callback_group)

        # -------- 상태 플래그 --------
        self.action_in_progress = False
        self.is_listening = False
        self.cooldown_until = None
        self.COOLDOWN_SECONDS = 5.0
        self.last_wakeup_time = 0
        self.MIN_WAKEUP_INTERVAL = 2.0
        
        self.current_command = None

        self.get_logger().info("===== GetKeyword Node Started =====")

        # -------- 타이머 시작 --------
        self.timer = self.create_timer(0.1, self.main_loop)

    # ------------------------------------------------------
    # 🔥 Feedback Callback
    # ------------------------------------------------------
    def feedback_callback(self, feedback_msg):
        try:
            fb = feedback_msg.feedback
            msg = getattr(fb, "status_message", fb)
            self.get_logger().info(f"📡 Feedback: {msg}")
        except Exception as e:
            self.get_logger().error(f"Feedback parse error: {e}")

    # ------------------------------------------------------
    # 🔥 메인 루프 (웨이크업 감지)
    # ------------------------------------------------------
    def main_loop(self):

        if self.action_in_progress:
            return

        # 쿨다운 처리
        if self.cooldown_until is not None:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns < self.cooldown_until:
                return
            else:
                self.cooldown_until = None
                try:
                    if self.is_listening:
                        self.mic_controller.close_stream()
                        self.is_listening = False
                        self.wakeup_word = None
                except:
                    pass
                return

        # 마이크 초기화
        if not self.is_listening:
            try:
                self.mic_controller.open_stream()
                self.wakeup_word = WakeupWord(self.mic_config.buffer_size)
                self.wakeup_word.set_stream(self.mic_controller.stream)
                self.is_listening = True
            except Exception as e:
                self.get_logger().error(f"Stream open error: {e}")
                return

        # 웨이크업 감지
        try:
            if self.wakeup_word and self.wakeup_word.is_wakeup():

                now = self.get_clock().now().nanoseconds / 1e9
                if now - self.last_wakeup_time < self.MIN_WAKEUP_INTERVAL:
                    return
                self.last_wakeup_time = now

                self.get_logger().info("🎤 Wakeup detected!")

                threading.Thread(
                    target=play_tts,
                    args=("어서오세요 슈슈박스입니다!",),
                    daemon=True
                ).start()

                # 사용자 음성 즉시 녹음 시작
                self.handle_voice_command()

        except Exception as e:
            self.get_logger().error(f"Wakeup error: {e}")
            self.is_listening = False
            self.wakeup_word = None

    # ------------------------------------------------------
    # 🔥 음성 녹음 → STT → 파싱
    # ------------------------------------------------------
    def handle_voice_command(self):
        self.action_in_progress = True

        try:
            self.get_logger().info("🔴 Recording...")
            self.mic_controller.record_audio()

            user_text = self.stt.speech2text()
            self.get_logger().info(f"📝 STT: {user_text}")

            command, obj = self.extract_command_object(user_text)

            if not command and not obj:
                self.get_logger().warn("⚠️ No command detected")
                self.action_in_progress = False
                return
            
            self.dispatch_command(command, obj)
            # try:
            #     event = {
            #         "command": command,
            #         "object": obj
            #     }
            #     requests.post(
            #         "http://localhost:8000/voice_event",
            #         json=event,
            #         timeout=0.3
            #     )
            #     self.get_logger().info(f"📡 UI Event Sent: {event}")
            # except Exception as e:
            #     self.get_logger().warn(f"⚠️ UI event send failed: {e}")
            

        except Exception as e:
            self.get_logger().error(f"Voice error: {e}")
            self.action_in_progress = False

    # ------------------------------------------------------
    # 🔥 LLM 파싱
    # ------------------------------------------------------
    def extract_command_object(self, user_text):
        response = self.lang_chain.invoke({"user_input": user_text})
        raw = response.content.strip()

        self.get_logger().info(f"🤖 LLM Response: {raw}")

        command = None
        obj = None

        m = re.search(r"command[:=]\s*([^\|\n\]]+)", raw)
        if m:
            c = m.group(1).strip()
            if c.lower() not in ["none", "null", ""]:
                command = c

        m = re.search(r"object[:=]\s*([^\|\n\]]+)", raw)
        if m:
            o = m.group(1).strip()
            if o.lower() not in ["none", "null", ""]:
                obj = o
        return command, obj

    # ------------------------------------------------------
    # 🔥 명령 라우팅
    # ------------------------------------------------------
    # ------------------------------------------------------
    # 🔥 명령 라우팅 (TTS 멘트 추가됨)
    # ------------------------------------------------------
    def dispatch_command(self, command, obj):
        
        self.get_logger().info(f"🔍 dispatch_command called: command='{command}', obj='{obj}'")
        self.current_command = command
        if obj and not command:
            command = "갖다줘"
            self.get_logger().info(f"🔄 No command but object exists, defaulting to '갖다줘'")

        # ---------------------------------------------------------
        # 🗣️ 1. 영어 객체명을 한글로 변환 (발음을 자연스럽게 하기 위해)
        # ---------------------------------------------------------
        korean_names = {
            "nike": "나이키",
            "adidas": "아디다스",
            "new_balance": "뉴발란스"
        }
        # 사전에 없으면 그냥 원래 영어 단어(obj)를 씀
        obj_kr = korean_names.get(obj, obj)

        # ---------------------------------------------------------
        # 🗣️ 2. 명령별 멘트 설정
        # ---------------------------------------------------------
        tts_msg = ""
        target_client = None

        if command == "갖다줘":
            tts_msg = f"{obj_kr} 신발 가져오는 중입니다."
            target_client = self.bring_client

        elif command == "구매":
            # 요청하신 대로 '가져오는 중입니다'로 설정
            tts_msg = f"{obj_kr} 신발 가져오는 중입니다."
            target_client = self.purchase_client

        elif command == "정리":
            tts_msg = "정리하겠습니다."
            target_client = self.clear_client

        elif command == "반품":
            tts_msg = "신발 반품하겠습니다."
            target_client = self.return_client
            
        else:
            self.get_logger().warn(f"⚠️ Unknown command: '{command}'")
            self.action_in_progress = False
            return

        # ---------------------------------------------------------
        # 🗣️ 3. TTS 재생 (로봇 멈추지 않게 스레드로 실행)
        # ---------------------------------------------------------
        if tts_msg:
            self.get_logger().info(f"🗣️ TTS 발화: {tts_msg}")
            threading.Thread(target=play_tts, args=(tts_msg,), daemon=True).start()

        # ---------------------------------------------------------
        # 🚀 4. 액션 전송
        # ---------------------------------------------------------
        if target_client:
            if obj is None:
                self.get_logger().warn("⚠️ 객체(object)가 없어 action을 실행할 수 없습니다.")
                self.action_in_progress = False
                return

            # 이제 정리/반품도 obj가 shoes로 들어오므로 항상 action 실행 가능
            self.send_action_goal(target_client, command, obj)


    # ------------------------------------------------------
    # 🔥 액션 Goal 전송
    # ------------------------------------------------------
    def send_action_goal(self, client, command, obj):

        goal_msg = PickAndPlace.Goal()
        goal_msg.target_command = command
        goal_msg.target_object = obj if obj else ""

        self.get_logger().info(f"📤 Sending → command={command}, object={obj}")

        # 서버 준비 확인
        if not client.server_is_ready():
            self.get_logger().warn("⏳ Waiting for action server...")
            ready = client.wait_for_server(timeout_sec=5.0)
            if not ready:
                self.get_logger().error("❌ Action server not available!")
                self.start_cooldown()
                return
            self.get_logger().info("✅ Action server ready!")

        # Goal 전송
        future = client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        future.add_done_callback(self.goal_response_callback)

    # ------------------------------------------------------
    # 🔥 Goal Response
    # ------------------------------------------------------
    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"❌ Goal request failed: {e}")
            self.start_cooldown()
            return

        if not goal_handle.accepted:
            self.get_logger().warn("⚠️ Goal rejected by server")
            self.start_cooldown()
            return

        # 🔥🔥🔥 여기서 UI에 action_started 알림 전송!
        try:
            requests.post(
                "http://localhost:8000/voice_event",
                json={"event": "action_started"},
                timeout=0.3
            )
            self.get_logger().info("📡 UI notified: action_started")
        except Exception as e:
            self.get_logger().warn(f"⚠️ UI event send failed: {e}")

        self.get_logger().info("✅ Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)


    # ------------------------------------------------------
    # 🔥 최종 결과 콜백
    # ------------------------------------------------------
    # ------------------------------------------------------
    # 🔥 최종 결과 콜백 (완료 멘트 로직 추가됨)
    # ------------------------------------------------------
    def get_result_callback(self, future):
        try:
            result = future.result()
            status = result.status
            msg = result.result.success_message
        except Exception as e:
            self.get_logger().error(f"❌ Result retrieval failed: {e}")
            self.start_cooldown()
            return

        # -------- 성공 / 실패 로그 --------
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"🎉 SUCCESS: {msg}")

            # ==========================================================
            # 🗣️ [핵심] 기억해둔 명령에 따라 완료 멘트 재생
            # ==========================================================
            finish_ment = ""

            # 1. 갖다줘 / 구매 -> "신발이 도착했습니다."
            if self.current_command in ["갖다줘", "구매"]:
                finish_ment = "신발이 도착했습니다."
            
            # 2. 정리 -> "정리를 완료했습니다."
            elif self.current_command == "정리":
                finish_ment = "정리를 완료했습니다."

            # 3. 반품 -> "반품을 완료했습니다."
            elif self.current_command == "반품":
                finish_ment = "반품을 완료했습니다."
            
            # 4. 그 외 (혹시 모를 예외 처리)
            else:
                finish_ment = "작업을 완료했습니다."

            # TTS 재생 (스레드로 실행하여 멈춤 방지)
            if finish_ment:
                threading.Thread(target=play_tts, args=(finish_ment,), daemon=True).start()

        else:
            self.get_logger().warn(f"❌ FAILED (status={status}): {msg}")
            # 실패 시 멘트
            threading.Thread(target=play_tts, args=("작업 도중 문제가 발생했습니다.",), daemon=True).start()

        # UI 알림 전송 (기존 유지)
        try:
            requests.post(
                "http://localhost:8000/voice_event",
                json={
                    "event": "action_finished",
                    "message": msg,
                    "status": "success" if status == GoalStatus.STATUS_SUCCEEDED else "failed"
                },
                timeout=0.3
            )
            self.get_logger().info("📡 UI notified: action_finished")
        except Exception as e:
            self.get_logger().warn(f"⚠️ UI event send failed: {e}")

        # 다음 작업을 위해 기억 초기화 및 쿨다운
        self.current_command = None
        self.start_cooldown()


    # ------------------------------------------------------
    # 🔥 쿨다운
    # ------------------------------------------------------
    def start_cooldown(self):
        self.cooldown_until = (
            self.get_clock().now().nanoseconds + int(self.COOLDOWN_SECONDS * 1e9)
        )
        self.action_in_progress = False


# ==========================================================
# 🔥 main
# ==========================================================
def main():
    rclpy.init()
    node = GetKeyword()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
