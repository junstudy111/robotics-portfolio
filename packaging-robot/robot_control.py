import rclpy
import DR_init
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
import subprocess
import signal
import threading
import time
import sys

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

# 이동 속도 및 가속도
VELOCITY = 100
ACC = 100

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# Firebase 설정
SERVICE_ACCOUNT_KEY_PATH = "./rokey-550f7-firebase-adminsdk-fbsvc-eba1fa0ef4.json"
DATABASE_URL = "https://rokey-550f7-default-rtdb.asia-southeast1.firebasedatabase.app"

# Global 변수
current_step = 0
launch_process = None
collision_detected = False
emergency_stop_flag = False


# ⭐ 긴급정지 예외 클래스
class EmergencyStopException(Exception):
    """긴급정지가 눌렸을 때 발생하는 예외"""
    pass


print("=" * 60)
print("Firebase 초기화 시작...")
print("=" * 60)

# Firebase 초기화
try:
    cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
    firebase_admin.initialize_app(cred, {
        'databaseURL': DATABASE_URL
    })
    print("✅ Firebase 초기화 완료!")
except ValueError:
    print("⚠️  Firebase 앱이 이미 초기화되었습니다.")
except Exception as e:
    print(f"❌ Firebase 초기화 실패: {e}")
    exit(1)

# Firebase 참조
robot_ref = db.reference('/robot')
print(f"✅ Firebase 경로 설정: /robot")


def check_emergency_stop():
    """긴급정지 + 충돌 신호 확인 - 0.01초 안에 체크"""
    try:
        data = robot_ref.get()
        if data:
            # ⭐ 긴급정지 체크
            if data.get('emergency_stop', False):
                print("\n" + "🚨" * 30)
                print("⚠️  긴급정지 감지! 즉시 중단합니다!")
                print("🚨" * 30 + "\n")
                raise EmergencyStopException("긴급정지!")
            
            # ⭐⭐⭐ 충돌 상태도 체크! (새로 추가!)
            if data.get('robot_status') == 'collision':
                print("\n" + "💥" * 30)
                print("⚠️  충돌 상태 감지! 즉시 중단합니다!")
                print("💥" * 30 + "\n")
                raise EmergencyStopException("충돌!")
                
    except EmergencyStopException:
        raise  # 긴급정지/충돌은 그대로 전파
    except Exception as e:
        # Firebase 연결 에러는 무시 (작업 계속)
        pass


def safe_wait(seconds):
    """긴급정지를 체크하면서 대기하는 함수"""
    if seconds <= 0:
        return
    
    # 0.1초 단위로 쪼개서 체크
    steps = int(seconds * 10)
    for i in range(steps):
        check_emergency_stop()
        time.sleep(0.1)
    
    # 남은 시간 처리
    remaining = seconds - (steps * 0.1)
    if remaining > 0:
        check_emergency_stop()
        time.sleep(remaining)


def check_and_reset_previous_state():
    """이전 세션의 비정상 종료 감지 및 자동 리셋"""
    print("\n" + "🔍" * 30)
    print("이전 상태 확인 중...")
    print("🔍" * 30 + "\n")
    
    try:
        data = robot_ref.get()
        
        if not data:
            print("✅ Firebase 데이터 없음 - 정상")
            return
        
        robot_status = data.get('robot_status', 'waiting')
        
        # 비정상 상태 감지
        abnormal_states = ['working', 'processing', 'recovering', 'collision']
        
        if robot_status in abnormal_states:
            print("\n" + "⚠️ " * 30)
            print("경고: 이전 세션이 비정상 종료되었습니다!")
            print(f"이전 상태: {robot_status}")
            print("⚠️ " * 30)
            
            print("\n🔄 3초 후 자동으로 안전 상태로 리셋합니다...")
            for i in range(3, 0, -1):
                print(f"   {i}...")
                time.sleep(1)
            
            # 안전 상태로 강제 리셋
            robot_ref.update({
                'robot_status': 'waiting',
                'robot_command': 'idle',
                'emergency_stop': False,
                'needs_recovery': False,
                'manual_recovery': False,
                'current_step': 0
            })
            
            print("\n✅ 상태 리셋 완료! 정상 작동 가능합니다.\n")
        else:
            print(f"✅ 이전 상태 정상: {robot_status}\n")
            
    except Exception as e:
        print(f"❌ 상태 확인 중 에러: {e}")
        print("⚠️  계속 진행합니다...\n")


def start_launch_process():
    """ROS2 launch 프로세스 시작"""
    global launch_process
    
    try:
        # 기존 프로세스 종료
        if launch_process and launch_process.poll() is None:
            print("⚠️  기존 launch 프로세스 종료 중...")
            launch_process.send_signal(signal.SIGINT)
            try:
                launch_process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                print("⚠️  강제 종료 시도...")
                launch_process.kill()
                launch_process.wait()
        
        launch_cmd = [
            'ros2', 'launch',
            'dsr_bringup2', 'dsr_bringup2_rviz.launch.py',
            'mode:=real',
            'host:=192.168.1.100',
            'port:=12345',
            f'model:={ROBOT_MODEL}'
        ]
        
        print("\n" + "🔌" * 30)
        print("Launch 프로세스 시작...")
        print(f"명령: {' '.join(launch_cmd)}")
        print("🔌" * 30 + "\n")
        
        # 백그라운드 실행
        launch_process = subprocess.Popen(
            launch_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
            bufsize=1
        )
        
        # 연결 대기
        time.sleep(3)
        
        if launch_process.poll() is None:
            print("✅ Launch 프로세스 시작 완료!\n")
            
            # 로그 모니터링 스레드 시작
            log_thread = threading.Thread(target=monitor_launch_logs, daemon=True)
            log_thread.start()
            
            return True
        else:
            print("❌ Launch 프로세스 시작 실패")
            return False
            
    except Exception as e:
        print(f"❌ Launch 시작 실패: {e}")
        return False


def monitor_launch_logs():
    """Launch 프로세스의 로그를 모니터링하여 충돌 감지"""
    global collision_detected, launch_process
    
    print("👀 Launch 로그 모니터링 시작...\n")
    
    while launch_process and launch_process.poll() is None:
        try:
            # ⭐ stdout 먼저 시도
            line = launch_process.stdout.readline()
            if not line or line.strip() == '':
                line = launch_process.stderr.readline()
            
            if not line or line.strip() == '':
                continue
            
            # ⭐⭐⭐ 터미널에도 출력! (디버깅용)
            print(f"[LAUNCH] {line.strip()}")
            
            line_lower = line.lower()
            
            # ⭐⭐⭐ 충돌 관련 키워드 감지 (수정!)
            collision_keywords = [
                'collision occurred',  # ⬅️ 핵심!
                'protective stop',
                'emergency stop',
                'safety stop',
                'external force',
                'external_force'
            ]
            
            if any(keyword in line_lower for keyword in collision_keywords):
                print("\n" + "💥" * 30)
                print(f"💥 충돌/외력 감지: {line.strip()}")
                print("💥" * 30 + "\n")
                
                collision_detected = True
                
                # ⭐⭐⭐ Firebase 업데이트
                robot_ref.update({
                    'robot_status': 'collision',
                    'needs_recovery': True,
                    'emergency_stop': True,  # ⬅️ 핵심!
                    'collision_message': line.strip()
                })
                print("   ✅ Firebase 충돌 상태 업데이트 완료!")
                
                # ⭐⭐⭐ Launch 프로세스 강제 종료
                print("\n🛑 Launch 프로세스 강제 종료 중...")
                try:
                    if launch_process and launch_process.poll() is None:
                        print("   1️⃣ SIGINT 전송...")
                        launch_process.send_signal(signal.SIGINT)
                        time.sleep(1)
                        
                        if launch_process.poll() is None:
                            print("   2️⃣ SIGTERM 전송...")
                            launch_process.terminate()
                            time.sleep(1)
                        
                        if launch_process.poll() is None:
                            print("   3️⃣ SIGKILL 전송 (강제 종료!)...")
                            launch_process.kill()
                            launch_process.wait(timeout=2)
                            
                    print("   ✅ Launch 프로세스 종료 완료!")
                    
                except Exception as e:
                    print(f"   ⚠️  Launch 정지 중 에러: {e}")
                
                print("\n✅ 충돌 처리 완료!")
                print("⚠️  로봇 정지됨. 대시보드에서 [로봇 초기화] 버튼을 기다립니다...")
                
                break
                    
        except Exception as e:
            print(f"⚠️  로그 모니터링 에러: {e}")
            break


def initialize_robot_with_retry(max_retries=3):
    """재시도 로직으로 초기화 - Tool/TCP + 로봇 모드 + 속도 설정"""
    print("\n" + "🔧" * 60)
    print("DEBUG: initialize_robot_with_retry() 시작")
    print("🔧" * 60 + "\n")
    
    from DSR_ROBOT2 import set_tool, set_tcp
    from DSR_ROBOT2 import set_singular_handling, DR_AVOID
    from DSR_ROBOT2 import set_velj, set_accj, set_velx, set_accx
    from DSR_ROBOT2 import set_robot_mode, ROBOT_MODE_AUTONOMOUS

    for i in range(max_retries):
        try:
            print(f"\n🔧 로봇 초기화 시도 {i+1}/{max_retries}...")
            
            # ⭐ 로봇 모드 먼저 설정!
            print("  DEBUG: set_robot_mode(AUTONOMOUS) 호출...")
            try:
                set_robot_mode(ROBOT_MODE_AUTONOMOUS)
                print("  DEBUG: 로봇 모드 = AUTONOMOUS 완료!")
            except Exception as e:
                print(f"  ⚠️  로봇 모드 설정 실패: {e}")
            
            time.sleep(1)  # 모드 전환 대기
            
            # Tool/TCP 설정
            print("  DEBUG: set_tool() 호출 시작...")
            set_tool(ROBOT_TOOL)
            print("  DEBUG: set_tool() 완료!")
            
            print("  DEBUG: set_tcp() 호출 시작...")
            set_tcp(ROBOT_TCP)
            print("  DEBUG: set_tcp() 완료!")
            print("  ✅ Tool/TCP 설정 완료!")
            
            time.sleep(1)  # 설정 반영 대기
            
            # ⭐ 속도 설정 (2번 반복!)
            print("\n  DEBUG: 속도 설정 시작 (1차)...")
            print("  DEBUG: set_singular_handling() 호출...")
            set_singular_handling(DR_AVOID)
            
            print("  DEBUG: set_velj(100.0) 호출...")
            set_velj(100.0)
            
            print("  DEBUG: set_accj(100.0) 호출...")
            set_accj(100.0)
            
            print("  DEBUG: set_velx(250.0, 80.625) 호출...")
            set_velx(250.0, 80.625)
            
            print("  DEBUG: set_accx(1000.0, 322.5) 호출...")
            set_accx(1000.0, 322.5)
            
            print("  ✅ 속도 설정 1차 완료!")
            
            # ⭐⭐⭐ 2초 대기 후 한 번 더!
            time.sleep(2)
            
            print("\n  DEBUG: 속도 설정 시작 (2차 - 확실하게!)...")
            set_velj(100.0)
            set_accj(100.0)
            set_velx(250.0, 80.625)
            set_accx(1000.0, 322.5)
            
            print("  ✅ 속도 설정 2차 완료! (velj=100.0, velx=250.0)")
            
            print("\n✅ 로봇 초기화 완료!")
            print("=" * 60 + "\n")
            return True
            
        except Exception as e:
            print(f"⚠️  시도 {i+1}/{max_retries} 실패: {e}")
            import traceback
            traceback.print_exc()
            time.sleep(3)
    
    print("❌ 로봇 초기화 최종 실패!")
    return False


def update_step(step):
    """현재 작업 단계를 Firebase에 업데이트"""
    global current_step
    current_step = step
    robot_ref.update({'current_step': step})
    print(f"📍 Step {step} 업데이트")


def perform_task():
    """로봇 작업 수행 - 긴급정지 체크 포함"""
    global current_step
    
    print('\n' + '🤖' * 20)
    print('로봇 작업 시작...')
    print('🤖' * 20 + '\n')
    
    from DSR_ROBOT2 import movej, posj, set_robot_mode, ROBOT_MODE_AUTONOMOUS
    from DSR_ROBOT2 import get_digital_output, ON, OFF, set_digital_output, wait
    from DSR_ROBOT2 import set_singular_handling, DR_AVOID
    from DSR_ROBOT2 import set_velj, set_accj, set_velx, set_accx
    from DSR_ROBOT2 import movej, movel, posx, posj, movec, movesj
    from DSR_ROBOT2 import DR_MV_RA_DUPLICATE, DR_MV_MOD_ABS

    def opengripper():
        set_digital_output(1, OFF)
        set_digital_output(2, ON)

    def closegripper():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)

    try:
        print("\n" + "⚡" * 60)
        print("DEBUG: perform_task() 속도 설정 시작")
        print("⚡" * 60)
        
        print("DEBUG: set_singular_handling(DR_AVOID) 호출...")
        set_singular_handling(DR_AVOID)
        
        print("DEBUG: set_velj(100.0) 호출...")
        set_velj(100.0)
        
        print("DEBUG: set_accj(100.0) 호출...")
        set_accj(100.0)
        
        print("DEBUG: set_velx(250.0, 80.625) 호출...")
        set_velx(250.0, 80.625)
        
        print("DEBUG: set_accx(1000.0, 322.5) 호출...")
        set_accx(1000.0, 322.5)
        
        print("✅ perform_task() 속도 설정 완료!")
        print("   velj=100.0, accj=100.0")
        print("   velx=250.0, accx=1000.0")
        print("⚡" * 60 + "\n")

        # Step 0: 시작
        update_step(0)
        check_emergency_stop()
        
        #### 구간 1: 옷 잡는 구간
        update_step(1)
        check_emergency_stop()
        
        opengripper()
        check_emergency_stop()
        
        print("DEBUG: 첫 번째 movej() 호출 - 초기 위치")
        movej(posj(-6.95, 3.12, 43.02, -0.07, 133.85, -4.94), radius=0.00, ra=DR_MV_RA_DUPLICATE)
        print("DEBUG: 첫 번째 movej() 완료")
        check_emergency_stop()
        
        safe_wait(0.10)
        
        movel(posx(451.46, 197.99, 252.51, 96.19, -179.73, 97.75), radius=0.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        check_emergency_stop()
        
        closegripper()
        check_emergency_stop()
    
        safe_wait(0.10)
        
        update_step(2)
        check_emergency_stop()
        
        movec(posx(453.46, 171.18, 310.12, 154.51, -179.76, 156.86), posx(451.17, 98.68, 346.55, 174.33, -179.14, 178.03), vel=[200.00, 76.50], acc=[800.00, 306.00], radius=0.00, ref=0, angle=[110.00,0.00], ra=DR_MV_RA_DUPLICATE)
        check_emergency_stop()
        
        safe_wait(0.10)
        
        opengripper()
        check_emergency_stop()
        
        safe_wait(0.10)
        
        movel(posx(453.75, -213.74, 252.23, 154.68, -179.81, 153.08), radius=0.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        check_emergency_stop()
        
        safe_wait(0.10)
        
        closegripper()
        check_emergency_stop()
        
        safe_wait(0.10)
        
        movec(posx(453.28, -174.30, 293.52, 95.73, -172.71, 86.05), posx(451.29, -122.55, 321.28, 96.47, -172.14, 86.51), vel=[200.00, 76.50], acc=[800.00, 306.00], radius=0.00, ref=0, angle=[70.00,0.00], ra=DR_MV_RA_DUPLICATE)
        check_emergency_stop()
        
        safe_wait(0.10)
        
        opengripper()
        check_emergency_stop()
        
        movej(posj(0.53, 29.50, 84.70, -8.45, 33.41, -83.26), radius=0.00, ra=DR_MV_RA_DUPLICATE)
        check_emergency_stop()
        
        closegripper()
        check_emergency_stop()
        
        safe_wait(0.10)
        
        movesj([posj(0.53, 29.50, 84.70, -8.44, 33.41, -83.26), posj(0.49, 19.79, 89.04, -8.45, 33.22, -83.26), posj(0.49, 5.57, 98.44, -8.45, 32.31, -83.26), posj(0.07, -15.90, 118.84, -8.45, 22.59, -83.26), posj(0.07, -27.08, 126.55, -8.45, 22.83, -83.26), posj(0.07, -38.78, 133.33, -8.45, 26.27, -83.26)], vel=141.17, acc=510.00)
        check_emergency_stop()
        
        safe_wait(0.10)
        
        opengripper()
        check_emergency_stop()

        #### 구간 3: 받침대 잡는 구간
        update_step(3)
        check_emergency_stop()
        
        safe_wait(0.10)
        
        movesj([posj(0.00, 0.00, 90.00, 0.00, 90.00, 0.00), posj(-0.18, -18.72, 106.63, 0.00, 92.02, 0.00), posj(-4.00, -14.82, 123.06, -0.11, 71.06, -3.72)])
        check_emergency_stop()
        
        closegripper()
        check_emergency_stop()
        
        safe_wait(0.10)

        # Step 4: 받침대 처리
        update_step(4)
        check_emergency_stop()
        
        movej(posj(-4.27, -19.31, 113.36, -0.14, 86.39, -4.00), radius=0.00, ra=DR_MV_RA_DUPLICATE)
        check_emergency_stop()
        
        movej(posj(-4.25, -1.68, 99.17, -0.14, 82.30, 86.00), vel=60.00, acc=100.00, radius=0.00, ra=DR_MV_RA_DUPLICATE)
        check_emergency_stop()
        
        movej(posj(-25.63, 2.31, 95.55, -0.20, 82.07, 64.47), vel=30.00, acc=100.00, radius=0.00, ra=DR_MV_RA_DUPLICATE)
        check_emergency_stop()
        
        movej(posj(-51.58, 23.21, 72.23, -0.11, 84.40, 38.76), vel=5.00, acc=100.00, radius=0.00, ra=DR_MV_RA_DUPLICATE)
        check_emergency_stop()
        
        movej(posj(-57.88, 50.49, 22.10, 13.13, 125.61, 36.21), vel=50.00, acc=150.00, radius=0.00, ra=DR_MV_RA_DUPLICATE)
        check_emergency_stop()
        
        movej(posj(-45.06, 19.61, 54.33, 19.48, 124.70, 50.13), vel=50.00, acc=200.00, radius=0.00, ra=DR_MV_RA_DUPLICATE)
        check_emergency_stop()
        
        movej(posj(-4.27, -2.42, 86.41, -0.14, 95.51, -4.00), radius=0.00, ra=DR_MV_RA_DUPLICATE)
        check_emergency_stop()
        
        movej(posj(-4.00, -14.82, 123.06, -0.11, 71.06, -3.72), radius=0.00, ra=DR_MV_RA_DUPLICATE)
        check_emergency_stop()
        
        opengripper()
        check_emergency_stop()
        
        safe_wait(0.10)
        
        # Step 5: 완료 - 초기 위치
        update_step(5)
        check_emergency_stop()
        
        movej(posj(-6.95, 3.12, 43.02, -0.07, 133.85, -4.94), radius=0.00, ra=DR_MV_RA_DUPLICATE)
        check_emergency_stop()
        
        print('\n' + '✅' * 20)
        print('로봇 작업 완료!')
        print('✅' * 20 + '\n')
        
        return True
        
    except EmergencyStopException:
        # 긴급정지 발생!
        print("\n" + "🛑" * 30)
        print("긴급정지로 작업 중단됨!")
        print(f"중단된 위치: Step {current_step}")
        print("🛑" * 30 + "\n")
        
        # Firebase 업데이트
        robot_ref.update({
            'robot_status': 'emergency_stopped',
            'robot_command': 'idle'
        })
        
        return False


def recovery_mode_1():
    """복구 모드 1: 옷을 집은 상태 - 풀고 초기화"""
    print("\n" + "🔧" * 30)
    print("복구 모드 1: 옷 집음 → 풀고 초기화")
    print("🔧" * 30 + "\n")
    
    from DSR_ROBOT2 import movej, posj, set_digital_output, ON, OFF
    from DSR_ROBOT2 import DR_MV_RA_DUPLICATE
    
    def opengripper():
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
    
    try:
        print("DEBUG: recovery_mode_1 시작 - 그리퍼 열기")
        opengripper()
        time.sleep(1.0)
        
        print("DEBUG: recovery_mode_1 - 초기 위치로 이동")
        movej(posj(-6.95, 3.12, 43.02, -0.07, 133.85, -4.94), radius=0.00, ra=DR_MV_RA_DUPLICATE)
        
        print("✅ 복구 모드 1 완료!\n")
        return True
    except Exception as e:
        print(f"❌ 복구 모드 1 실패: {e}")
        import traceback
        traceback.print_exc()
        return False


def recovery_mode_2():
    """복구 모드 2: 물건 안 집음 - 바로 초기화"""
    print("\n" + "🔧" * 30)
    print("복구 모드 2: 물건 안 집음 → 초기화")
    print("🔧" * 30 + "\n")
    
    from DSR_ROBOT2 import movej, posj
    from DSR_ROBOT2 import DR_MV_RA_DUPLICATE
    
    try:
        print("DEBUG: recovery_mode_2 - 초기 위치로 이동")
        movej(posj(-6.95, 3.12, 43.02, -0.07, 133.85, -4.94), radius=0.00, ra=DR_MV_RA_DUPLICATE)
        
        print("✅ 복구 모드 2 완료!\n")
        return True
    except Exception as e:
        print(f"❌ 복구 모드 2 실패: {e}")
        import traceback
        traceback.print_exc()
        return False


def recovery_mode_3():
    """복구 모드 3: 받침대 집음 - 원위치 후 초기화"""
    print("\n" + "🔧" * 30)
    print("복구 모드 3: 받침대 집음 → 원위치 후 초기화")
    print("🔧" * 30 + "\n")
    
    from DSR_ROBOT2 import movej, posj, set_digital_output, ON, OFF, movel, posx
    from DSR_ROBOT2 import DR_MV_RA_DUPLICATE, DR_MV_MOD_ABS
    
    def opengripper():
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
    
    try:
        print("DEBUG: recovery_mode_3 - 받침대 원위치로 이동")
        movel(posx(332.88, -82.50, 354.64, 108.18, -179.83, -161.48), radius=0.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        movej(posj(-4.27, -2.42, 86.41, -0.14, 95.51, -4.00), radius=0.00, ra=DR_MV_RA_DUPLICATE)
        movej(posj(-4.00, -14.82, 123.06, -0.11, 71.06, -3.72), radius=0.00, ra=DR_MV_RA_DUPLICATE)
        
        print("DEBUG: recovery_mode_3 - 그리퍼 열기")
        opengripper()
        time.sleep(1.0)
        
        print("DEBUG: recovery_mode_3 - 초기 위치로 이동")
        movej(posj(-6.95, 3.12, 43.02, -0.07, 133.85, -4.94), radius=0.00, ra=DR_MV_RA_DUPLICATE)
        
        print("✅ 복구 모드 3 완료!\n")
        return True
    except Exception as e:
        print(f"❌ 복구 모드 3 실패: {e}")
        import traceback
        traceback.print_exc()
        return False


def auto_recovery():
    """자동 복구: Python 프로세스 완전 재시작 (Ctrl+C와 동일)"""
    global collision_detected, current_step, launch_process
    
    print("\n" + "🚨" * 30)
    print("auto_recovery() 시작!")
    print("완전 재시작을 위해 Python 프로세스를 재실행합니다!")
    print("🚨" * 30 + "\n")
    
    # Firebase에서 마지막 step 확인
    data = robot_ref.get()
    last_step = data.get('current_step', 0)
    
    print(f"📊 마지막 Step: {last_step}")
    
    # Step 기반 복구 모드 자동 결정
    if last_step == 0:
        recovery_mode = 2
    elif 1 <= last_step <= 2:
        recovery_mode = 1
    elif last_step >= 3:
        recovery_mode = 3
    else:
        recovery_mode = 2
    
    print(f"🎯 복구 모드 결정: Mode {recovery_mode}")
    
    # Firebase에 복구 정보 저장 (재시작 후 사용)
    robot_ref.update({
        'robot_status': 'restarting',
        'recovery_mode': recovery_mode,
        'needs_recovery': True,
        'restart_timestamp': time.time()
    })

    print("collision detected: ",collision_detected )
    if collision_detected:
        print("\n🧹 Launch 프로세스 종료 중...")
        try:
            launch_process.send_signal(signal.SIGINT)
            launch_process.wait(timeout=10)
            print("✅ Launch 프로세스 정상 종료")
        except subprocess.TimeoutExpired:
            print("⚠️  강제 종료...")
            launch_process.kill()
            launch_process.wait()
    # 1️⃣ RViz2 종료
    print("   1️⃣ RViz2 프로세스 종료 중...")
    subprocess.run(['pkill', '-9', '-f', 'rviz2'], check=False, 
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
    # 2️⃣ dsr_bringup 관련 프로세스 종료
    print("   2️⃣ dsr_bringup 프로세스 종료 중...")
    subprocess.run(['pkill', '-9', '-f', 'dsr_bringup'], check=False,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    

    # 3️⃣ controller_manager 종료
    print("   3️⃣ controller_manager 프로세스 종료...")
    subprocess.run(['pkill', '-9', '-f', 'controller_manager'], check=False,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
    # 4️⃣ ros2_control_node 종료
    print("   4️⃣ ros2_control_node 프로세스 종료...")
    subprocess.run(['pkill', '-9', '-f', 'ros2_control_node'], check=False,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
    # 5️⃣ spawner 프로세스 종료
    print("   5️⃣ spawner 프로세스 종료...")
    subprocess.run(['pkill', '-9', '-f', 'spawner'], check=False,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
    # 6️⃣ ros2 launch 프로세스 종료
    print("   6️⃣ ros2 launch 프로세스 종료...")
    subprocess.run(['pkill', '-9', '-f', 'ros2 launch'], check=False,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
    # 7️⃣ ⭐⭐⭐ ROS2 데몬 재시작!
    print("   7️⃣ ROS2 데몬 재시작...")
    subprocess.run(['ros2', 'daemon', 'stop'], check=False,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)           

    print("\n🔄 5초 후 프로그램을 재시작합니다...")
    print("⚠️  이것은 정상 동작입니다. Ctrl+C 재실행과 동일한 효과입니다.")
    for i in range(5, 0, -1):
        print(f"   {i}...")
        time.sleep(1)
    
    # Launch 프로세스 정리
    if launch_process and launch_process.poll() is None:
        print("\n🧹 Launch 프로세스 종료 중...")
        try:
            launch_process.send_signal(signal.SIGINT)
            launch_process.wait(timeout=10)
            print("✅ Launch 프로세스 정상 종료")
        except subprocess.TimeoutExpired:
            print("⚠️  강제 종료...")
            launch_process.kill()
            launch_process.wait()
            


    
    # ⭐⭐⭐ Python 프로세스 완전 재시작 (Ctrl+C와 동일!)
    print("\n🔄 Python 프로세스 재시작 중...\n")
    
    import os
    python = sys.executable
    
    # 현재 스크립트로 프로세스 교체
    os.execv(python, [python] + sys.argv)




def check_firebase_command():
    """Firebase 명령 확인 (폴링 방식) - 수정된 순서"""
    global emergency_stop_flag
    
    try:
        data = robot_ref.get()
        
        if not data:
            return False
        
        # ⭐ 1순위: 강제 리셋 명령 확인 (최우선!)
        if data.get('force_reset', False):
            print("\n" + "⚡" * 30)
            print("강제 리셋 명령 수신!")
            print("⚡" * 30 + "\n")
            
            robot_ref.update({
                'robot_status': 'waiting',
                'robot_command': 'idle',
                'emergency_stop': False,
                'needs_recovery': False,
                'manual_recovery': False,
                'force_reset': False,
                'current_step': 0
            })
            
            emergency_stop_flag = False
            print("✅ 강제 리셋 완료!\n")
            return True
        
        # ⭐ 2순위: 로봇 초기화 명령 확인 (긴급정지 및 충돌 상태에서도 실행!)
        if data.get('manual_recovery', False):
            print("\n" + "🔧" * 30)
            print("수동 복구 명령 수신!")
            print("🔧" * 30 + "\n")
            
            robot_ref.update({
                'manual_recovery': False,
                'emergency_stop': False
            })
            
            emergency_stop_flag = False
            
            # 자동 복구 실행 (수동으로 트리거됨)
            recovery_thread = threading.Thread(target=auto_recovery, daemon=False)
            recovery_thread.start()
            return True
        
        # ⭐ 3순위: 긴급정지 확인 (마지막에 체크)
        if data.get('emergency_stop', False):
            if not emergency_stop_flag:
                print("\n" + "🛑" * 30)
                print("긴급정지 신호 수신!")
                print("🛑" * 30 + "\n")
                emergency_stop_flag = True
                robot_ref.update({
                    'robot_status': 'emergency_stopped'
                })
            return False
        
        command = data.get('robot_command', 'idle')
        
        # 'start' 명령 감지
        if command == 'start':
            print("\n" + "🔔" * 30)
            print("로봇 시작 명령 수신!")
            print("🔔" * 30)
            
            robot_ref.update({
                'robot_status': 'working',
                'robot_command': 'processing'
            })
            print("📤 Firebase 업데이트: 상태 = working")
            
            try:
                # 로봇 작업 수행
                success = perform_task()
                
                if success:
                    # 완료 카운트 증가
                    current_count = data.get('completed_count', 0)
                    new_count = current_count + 1
                    
                    robot_ref.update({
                        'completed_count': new_count,
                        'robot_status': 'waiting',
                        'robot_command': 'idle',
                        'current_step': 0,
                        'last_completed_time': time.time()
                    })
                    
                    print(f"\n✅ 작업 완료! (총 {new_count}개)")
                    print(f"📤 Firebase 업데이트: completed_count = {new_count}")
                else:
                    # 긴급정지로 중단됨
                    print("\n⚠️  작업이 긴급정지로 중단되었습니다.")
                
                return True
                
            except Exception as e:
                print(f"\n❌ 로봇 작업 중 에러: {e}")
                import traceback
                traceback.print_exc()
                robot_ref.update({
                    'robot_status': 'error',
                    'robot_command': 'idle',
                    'error_message': str(e)
                })
                print("📤 Firebase 업데이트: 상태 = error")
                return False
        
        return False
        
    except Exception as e:
        print(f"⚠️  Firebase 확인 중 에러: {e}")
        return False


def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 Firebase 폴링"""
    global launch_process
    
    rclpy.init(args=args)
    node = rclpy.create_node("robot_firebase_controller", namespace=ROBOT_ID)
    
    DR_init.__dsr__node = node

    try:
        # ⭐⭐⭐ 재시작 후 복구 작업이 필요한지 확인
        data = robot_ref.get() or {}
        needs_recovery = data.get('needs_recovery', False)
        
        if needs_recovery:
            recovery_mode = data.get('recovery_mode', 2)
            print("\n" + "🔄" * 60)
            print("재시작 감지!")
            print(f"복구 모드 {recovery_mode} 실행 준비 중...")
            print("🔄" * 60 + "\n")
            
            # Launch 시작
            if not start_launch_process():
                print("❌ 초기 Launch 시작 실패!")
                robot_ref.update({
                    'robot_status': 'error',
                    'needs_recovery': False,
                    'error_message': 'Launch failed after restart'
                })
                return
            
            # ⭐ 로봇 연결 대기 (10초 → 15초)
            print("\n⏳ 로봇 연결 대기 중... (15초)")
            time.sleep(15)  # 10초 → 15초!
            
            print("\n" + "=" * 60)
            print("DEBUG: 재시작 후 initialize_robot_with_retry() 호출")
            print("=" * 60)
            
            if not initialize_robot_with_retry():
                print("❌ 로봇 초기화 실패!")
                robot_ref.update({
                    'robot_status': 'error',
                    'needs_recovery': False,
                    'error_message': 'Robot initialization failed after restart'
                })
                return
            
            # 복구 모드 실행
            print(f"\n🔧 복구 모드 {recovery_mode} 실행 중...\n")
            success = False
            try:
                if recovery_mode == 1:
                    success = recovery_mode_1()
                elif recovery_mode == 2:
                    success = recovery_mode_2()
                elif recovery_mode == 3:
                    success = recovery_mode_3()
            except Exception as e:
                print(f"\n❌ 복구 모드 실행 중 에러: {e}")
                import traceback
                traceback.print_exc()
            
            # 복구 완료
            if success:
                robot_ref.update({
                    'robot_status': 'waiting',
                    'robot_command': 'idle',
                    'needs_recovery': False,
                    'recovery_mode': 0,
                    'current_step': 0,
                    'emergency_stop': False
                })
                print("\n" + "✅" * 30)
                print("재시작 후 복구 완료!")
                print("✅" * 30 + "\n")
            else:
                robot_ref.update({
                    'robot_status': 'error',
                    'needs_recovery': False,
                    'error_message': f'Recovery mode {recovery_mode} failed after restart'
                })
                print("\n❌ 복구 실패!")
                return
        
        else:
            # ⭐ 정상 시작 (재시작 아님)
            print("\n" + "🔍" * 30)
            print("정상 시작 - 이전 상태 확인 중...")
            print("🔍" * 30 + "\n")
            
            check_and_reset_previous_state()
            
            # Launch 프로세스 시작
            if not start_launch_process():
                print("❌ 초기 Launch 시작 실패!")
                return
            
            # ⭐ 로봇 초기화 (10초 → 15초)
            print("\n⏳ 로봇 연결 대기 중... (15초)")
            time.sleep(15)  # 10초 → 15초!
            
            print("\n" + "=" * 60)
            print("DEBUG: main()에서 initialize_robot_with_retry() 호출")
            print("=" * 60)
            
            initialize_robot_with_retry()
            
            # Firebase 초기 상태 설정
            current_data = robot_ref.get() or {}
            initial_sales = current_data.get('sales_count', 0)
            initial_completed = current_data.get('completed_count', 0)
            
            robot_ref.update({
                'robot_status': 'waiting',
                'robot_command': 'idle',
                'sales_count': initial_sales,
                'completed_count': initial_completed,
                'current_step': 0,
                'needs_recovery': False,
                'emergency_stop': False,
                'force_reset': False
            })
        
        # ⭐ 공통: Firebase 폴링 시작
        current_data = robot_ref.get() or {}
        initial_sales = current_data.get('sales_count', 0)
        initial_completed = current_data.get('completed_count', 0)
        
        print("\n" + "=" * 60)
        print("🚀 로봇 Firebase 폴링 시작!")
        print(f"📊 현재 상태:")
        print(f"   - 판매: {initial_sales}개")
        print(f"   - 완료: {initial_completed}개")
        print(f"   - 상태: 대기 중")
        print(f"\n💡 0.5초마다 Firebase 확인 중...")
        print("   HTML에서 버튼을 눌러보세요!")
        print("   ⚡ 긴급정지 반응 속도: 0.1초 이내!")
        print("=" * 60 + "\n")
        
        # 폴링 루프
        last_check_time = time.time()
        check_interval = 0.5
        
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            
            current_time = time.time()
            if current_time - last_check_time >= check_interval:
                check_firebase_command()
                last_check_time = current_time

    except KeyboardInterrupt:
        print("\n\n" + "🛑" * 20)
        print("프로그램 종료 중...")
        print("🛑" * 20)
    except Exception as e:
        print(f"\n❌예상치 못한 에러: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 프로세스 정리
        if launch_process and launch_process.poll() is None:
            print("🧹 Launch 프로세스 종료 중...")
            launch_process.send_signal(signal.SIGINT)
            try:
                launch_process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                launch_process.kill()
        
        rclpy.shutdown()
        print("\n👋 프로그램 종료 완료")


if __name__ == "__main__":
    main()

