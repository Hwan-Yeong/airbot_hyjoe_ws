cmake_minimum_required(VERSION 3.8)
project(airbot_log)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)

# 실행 파일 추가
add_executable(logging_amr src/logging_amr.cpp)

# 올바른 타겟 이름 사용 및 링크 설정
target_include_directories(logging_amr PRIVATE ${rclcpp_INCLUDE_DIRS})

target_link_libraries(logging_amr fmt::fmt spdlog::spdlog)

ament_target_dependencies(logging_amr rclcpp spdlog)

# 설치 설정
install(TARGETS
    logging_amr
    DESTINATION lib/${PROJECT_NAME})

ament_package()
