import time
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy
from rclpy.node import Node

class NavCommander(Node):
    def __init__(self):
        super().__init__('nav_commander_node')
        
        # 1. Nav2 BasicNavigator 초기화
        self.nav = BasicNavigator()
        
        # 2. 초기 위치 설정 (Initial Pose) - 기존 코드 유지
        init_pose = PoseStamped()
        init_pose.header.frame_id = 'map'
        init_pose.header.stamp = self.nav.get_clock().now().to_msg()
        init_pose.pose.position.x = 0.0
        init_pose.pose.position.y = 0.0
        init_pose.pose.orientation.w = 1.0 
        
        # Nav2 활성화 대기
        self.nav.setInitialPose(init_pose)
        self.nav.waitUntilNav2Active()
        
        print("✅ [Nav2] Ready for Navigation Commands!")

        # 3. 명령 수신을 위한 Subscriber 생성
        # 토픽 이름: /nav_target
        self.subscription = self.create_subscription(
            PoseStamped,
            '/nav_target',
            self.goal_callback,
            10
        )
        
        # 4. 상태 확인용 타이머 (0.5초마다 실행)
        self.timer = self.create_timer(0.5, self.feedback_callback)
        self.is_moving = False

    def goal_callback(self, msg):
        """외부에서 목표 좌표가 오면 실행되는 함수"""
        print(f"📩 [Received Goal] x: {msg.pose.position.x:.2f}, y: {msg.pose.position.y:.2f}")
        
        # 좌표계 시간 동기화 (필수)
        msg.header.stamp = self.nav.get_clock().now().to_msg()
        
        # Nav2에게 이동 명령 전달
        self.nav.goToPose(msg)
        self.is_moving = True

    def feedback_callback(self):
        """이동 중 남은 거리를 주기적으로 출력"""
        if not self.is_moving:
            return

        # 태스크 완료 여부 확인
        if self.nav.isTaskComplete():
            self.is_moving = False
            result = self.nav.getResult()
            if result == TaskResult.SUCCEEDED:
                print("🏁 [Result] Goal Reached!")
            elif result == TaskResult.CANCELED:
                print("🛑 [Result] Goal Canceled!")
            elif result == TaskResult.FAILED:
                print("⚠️ [Result] Goal Failed!")
            return

        # 이동 중 피드백 출력
        feedback = self.nav.getFeedback()
        if feedback:
            # 남은 거리가 너무 자주 출력되면 정신없으니 1초에 한 번 정도만 봐도 됨
            print(f"🚗 Moving... Distance remaining: {feedback.distance_remaining:.2f} m")

def main():
    rclpy.init()
    
    # 노드 생성 및 실행
    commander = NavCommander()
    
    try:
        rclpy.spin(commander)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()