cmake_minimum_required(VERSION 3.8)
project(kebi_follow_me)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
# 조정 화면 (tune_server.py 는 PC, tune_agent.py 는 로봇에서 실행)
install(DIRECTORY tools DESTINATION share/${PROJECT_NAME} USE_SOURCE_PERMISSIONS)

ament_package()
