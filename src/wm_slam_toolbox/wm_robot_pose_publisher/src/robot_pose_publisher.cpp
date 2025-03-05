#include "robot_pose_publisher/robot_pose_publisher.hpp"
#include "robot_pose_publisher/df_logger.hpp"

RobotPosePublisher::RobotPosePublisher() : Node(NODE_NAME), is_stamped_(false), base_frame_(F_BASE_LINK), map_frame_(F_MAP), tp_robot_pose_(TP_ROBOT_POSE){
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_buffer_->canTransform(map_frame_, base_frame_, tf2_ros::fromMsg(this->now()), std::chrono::seconds(1));

  this->declare_parameter<std::string>("map_frame","map");
  this->declare_parameter<std::string>("base_frame","base_link");
  this->declare_parameter<bool>("is_stamped",false);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("is_stamped", is_stamped_);

  auto qos_profile = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

  publisher_robot_pose_ = this->create_publisher<PoseStamped>(tp_robot_pose_, qos_profile);
  timer_ = this->create_wall_timer(10ms, std::bind(&RobotPosePublisher::FnTimerCallback, this));
}

void RobotPosePublisher::FnTimerCallback(){
  geometry_msgs::msg::TransformStamped transformStamped;
  try{
    transformStamped = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
  }
  catch (tf2::TransformException &ex){
    //RCLCPP_INFO(this->get_logger(),"fn_timer_callback : %s","tf2_transform_exception");
    return;
  }

  PoseStamped robot_pose_stamped;
  robot_pose_stamped.header.frame_id = map_frame_;
  robot_pose_stamped.header.stamp = this->now();

  robot_pose_stamped.pose.pose.orientation.x = transformStamped.transform.rotation.x;
  robot_pose_stamped.pose.pose.orientation.y = transformStamped.transform.rotation.y;
  robot_pose_stamped.pose.pose.orientation.z = transformStamped.transform.rotation.z;
  robot_pose_stamped.pose.pose.orientation.w = transformStamped.transform.rotation.w;

  robot_pose_stamped.pose.pose.position.x = transformStamped.transform.translation.x;
  robot_pose_stamped.pose.pose.position.y = transformStamped.transform.translation.y;
  robot_pose_stamped.pose.pose.position.z = transformStamped.transform.translation.z;

  RCL_LOG_INFO(this->get_logger(),"x : %lf, y : %lf, z : %lf, w : %lf",robot_pose_stamped.pose.pose.position.x, robot_pose_stamped.pose.pose.position.y,  robot_pose_stamped.pose.pose.orientation.z,robot_pose_stamped.pose.pose.orientation.w);
  publisher_robot_pose_->publish(robot_pose_stamped);

}


int main(int argc, char *argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RobotPosePublisher>());
	rclcpp::shutdown();
	return 0;
}
