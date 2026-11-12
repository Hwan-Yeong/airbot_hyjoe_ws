// follow_controller 노드 생성 (노드 단위 시험용). 실제 실행은 컴포넌트 등록을 쓴다.
#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace kebi_follow {

std::shared_ptr<rclcpp::Node> createFollowController(
    const rclcpp::NodeOptions& options);

}  // namespace kebi_follow
