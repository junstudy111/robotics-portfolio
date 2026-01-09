import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import pyfirmata
import serial.tools.list_ports
import time

# ==========================================
# 설정
# ==========================================
SERVO_PIN = 9
STOP_VALUE = 94
ROTATE_TIME = 0.34
STABILIZE_TIME = 0.3

# ==========================================
# 하드웨어 드라이버
# ==========================================
class ServoDriver:
    def __init__(self):
        self.board = None
        self.servo = None
        self.is_connected = False
        
        # ⭐ 위치 추적 변수
        self.position_state = 0  # 0: 고객 쪽(초기), -2: 로봇 쪽
        
        print("[Init] 위치 추적 시스템 초기화")
        print("[Init] position_state = 0 (고객 쪽)")

    def find_arduino_port(self):
        """자동으로 아두이노 포트 찾기"""
        ports = list(serial.tools.list_ports.comports())
        
        for port in ports:
            if 'Arduino' in port.description:
                print(f"[Auto] Arduino 발견: {port.device}")
                return port.device
        
        for port in ports:
            if 'ACM' in port.device or 'USB' in port.device:
                print(f"[Auto] USB Serial 발견: {port.device}")
                return port.device
        
        print("[Auto] ⚠️  자동 탐색 실패, 기본 포트 시도...")
        return '/dev/ttyACM0'

    def connect(self):
        if self.is_connected: 
            return True
        
        port = self.find_arduino_port()
        
        try:
            print(f"[Hardware] 아두이노({port}) 연결 시도...")
            self.board = pyfirmata.Arduino(port)
            
            it = pyfirmata.util.Iterator(self.board)
            it.start()
            time.sleep(1)
            
            self.servo = self.board.get_pin(f'd:{SERVO_PIN}:s')
            self.servo.write(STOP_VALUE)
            self.is_connected = True
            
            print(f"[Hardware] ✓ 연결 성공! 포트: {port}")
            print(f"[Hardware] ⚠️  턴테이블을 고객 쪽(초기 위치)에 맞춰주세요!")
            return True
            
        except Exception as e:
            print(f"[Error] 연결 실패: {e}")
            print(f"[Error] 포트: {port}")
            return False

    def get_position_name(self):
        """현재 위치를 사람이 읽을 수 있는 이름으로 반환"""
        if self.position_state == 0:
            return "고객 쪽 (초기 위치)"
        elif self.position_state == -2:
            return "로봇 쪽"
        else:
            return f"알 수 없음 (state={self.position_state})"

    def rotate_to_robot(self):
        """
        고객 쪽 → 로봇 쪽 (시계 90도)
        ⭐ 반대로 변경: 180 → 0
        """
        if not self.servo:
            print("[ERROR] 서보 없음!")
            return False
        
        print("="*60)
        print(f"[Before] position_state = {self.position_state} ({self.get_position_name()})")
        
        # ⭐ 이미 로봇 쪽이면 회전 안 함
        if self.position_state == -2:
            print("[Servo] 이미 로봇 쪽 위치 - 회전 생략")
            print("="*60)
            return True
        
        print("[Servo] 회전 시작: 고객 → 로봇 (시계 90도)")  # ⭐ 수정
        print(f"[Servo] 명령: write(0)")  # ⭐ 수정
        
        self.servo.write(0)  # ⭐ 변경: 180 → 0 (시계방향)
        time.sleep(ROTATE_TIME)
        self.servo.write(STOP_VALUE)
        
        # ⭐ 위치 업데이트
        self.position_state -= 2
        
        # ⭐ 안전 범위 체크
        if self.position_state < -2:
            print(f"[Warning] position_state 범위 초과! {self.position_state} → -2")
            self.position_state = -2
        
        print(f"[After] position_state = {self.position_state} ({self.get_position_name()})")
        print("[Servo] ✓ 회전 완료")
        print("="*60)
        
        time.sleep(STABILIZE_TIME)
        return True

    def rotate_to_customer(self):
        """
        로봇 쪽 → 고객 쪽 (반시계 90도)
        ⭐ 반대로 변경: 0 → 180
        """
        if not self.servo:
            print("[ERROR] 서보 없음!")
            return False
        
        print("="*60)
        print(f"[Before] position_state = {self.position_state} ({self.get_position_name()})")
        
        # ⭐ 이미 고객 쪽이면 회전 안 함
        if self.position_state == 0:
            print("[Servo] 이미 고객 쪽 위치 - 회전 생략")
            print("="*60)
            return True
        
        print("[Servo] 회전 시작: 로봇 → 고객 (반시계 90도)")  # ⭐ 수정
        print(f"[Servo] 명령: write(180)")  # ⭐ 수정
        
        self.servo.write(180)  # ⭐ 변경: 0 → 180 (반시계방향)
        time.sleep(ROTATE_TIME)
        self.servo.write(STOP_VALUE)
        
        # ⭐ 위치 업데이트
        self.position_state += 2
        
        # ⭐ 안전 범위 체크
        if self.position_state > 0:
            print(f"[Warning] position_state 범위 초과! {self.position_state} → 0")
            self.position_state = 0
        
        print(f"[After] position_state = {self.position_state} ({self.get_position_name()})")
        print("[Servo] ✓ 회전 완료")
        print("="*60)
        
        time.sleep(STABILIZE_TIME)
        return True

    def close(self):
        if self.board:
            # 종료 시 고객 쪽으로 복귀
            if self.position_state != 0:
                print(f"[Shutdown] 현재 위치: {self.get_position_name()}")
                print("[Shutdown] 고객 쪽으로 복귀 중...")
                self.rotate_to_customer()
            
            self.servo.write(STOP_VALUE)
            self.board.exit()
            print("[Shutdown] 아두이노 연결 종료")

# ==========================================
# ROS 2 서비스 노드
# ==========================================
class ArduinoServiceNode(Node):
    def __init__(self):
        super().__init__('arduino_control_node')
        
        self.driver = ServoDriver()
        if not self.driver.connect():
            self.get_logger().error("❌ 아두이노 연결 실패!")
            self.get_logger().error("   노드는 계속 실행되지만 서비스는 작동하지 않습니다.")
        
        # ⭐ 2개의 서비스
        self.srv_to_robot = self.create_service(
            Trigger, 
            'turntable_to_robot',
            self.to_robot_callback
        )
        self.srv_to_customer = self.create_service(
            Trigger, 
            'turntable_to_customer',
            self.to_customer_callback
        )
        
        # ⭐ 주기적으로 상태 출력 (10초마다)
        self.create_timer(10.0, self.print_status)
        
        self.get_logger().info("="*60)
        self.get_logger().info("=== 아두이노 터미널 제어 서비스 대기 중 ===")
        self.get_logger().info("Services:")
        self.get_logger().info("  • /turntable_to_robot    (고객 → 로봇, 시계)")
        self.get_logger().info("  • /turntable_to_customer (로봇 → 고객, 반시계)")
        self.get_logger().info("")
        self.get_logger().info("위치 추적 시스템: 활성화")
        self.get_logger().info(f"현재 위치: {self.driver.get_position_name()}")
        self.get_logger().info("="*60)

    def print_status(self):
        """주기적으로 상태 출력"""
        if self.driver.is_connected:
            self.get_logger().info(
                f"[Status] position={self.driver.position_state} "
                f"({self.driver.get_position_name()})"
            )

    def to_robot_callback(self, request, response):
        """고객 → 로봇 (시계)"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("[요청] 턴테이블 → 로봇 쪽 (시계)")
        self.get_logger().info("="*60)
        
        if self.driver.is_connected:
            success = self.driver.rotate_to_robot()
            response.success = success
            response.message = (
                f"로봇 쪽 회전 완료 (state={self.driver.position_state})" 
                if success else "회전 실패"
            )
            self.get_logger().info(f"[결과] {response.message}")
        else:
            response.success = False
            response.message = "하드웨어 연결 안됨"
            self.get_logger().error("[결과] 하드웨어 미연결!")
        
        return response

    def to_customer_callback(self, request, response):
        """로봇 → 고객 (반시계)"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("[요청] 턴테이블 → 고객 쪽 (반시계)")
        self.get_logger().info("="*60)
        
        if self.driver.is_connected:
            success = self.driver.rotate_to_customer()
            response.success = success
            response.message = (
                f"고객 쪽 회전 완료 (state={self.driver.position_state})" 
                if success else "회전 실패"
            )
            self.get_logger().info(f"[결과] {response.message}")
        else:
            response.success = False
            response.message = "하드웨어 연결 안됨"
            self.get_logger().error("[결과] 하드웨어 미연결!")
        
        return response

    def destroy_node(self):
        self.driver.close()
        super().destroy_node()

# ==========================================
# 메인
# ==========================================
def main(args=None):
    rclpy.init(args=args)
    node = ArduinoServiceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n종료합니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
