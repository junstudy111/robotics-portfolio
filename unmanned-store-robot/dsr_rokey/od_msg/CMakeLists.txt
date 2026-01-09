cmake_minimum_required(VERSION 3.8)
project(od_msg)

# 1. 필수 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 2. 메시지/서비스/액션 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SrvDepthPosition.srv"
  "action/PickAndPlace.action"
  "srv/SetMode.srv"
  "srv/UiCommand.srv"
)

# 3. 인터페이스 설치(rosidl_generate_interfaces가 자동 설치 처리함)
install(
  DIRECTORY srv action
  DESTINATION share/${PROJECT_NAME}
)

# 4. 런타임 의존성 export
ament_export_dependencies(rosidl_default_runtime)

# 반드시 마지막
ament_package()
