#ifndef AIRBOT_SENSOR_MANAGER_UTILS_FRAME_CONVERTER_HPP_
#define AIRBOT_SENSOR_MANAGER_UTILS_FRAME_CONVERTER_HPP_

#include <cmath>
#include <map>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "robot_custom_msgs/msg/abnormal_event_data.hpp"
#include "robot_custom_msgs/msg/bottom_ir_data.hpp"
#include "robot_custom_msgs/msg/camera_data.hpp"
#include "robot_custom_msgs/msg/camera_data_array.hpp"
#include "robot_custom_msgs/msg/tof_data.hpp"
#include "utils/common_struct.hpp"
#include "vision_msgs/msg/bounding_box2_d.hpp"
#include "vision_msgs/msg/bounding_box2_d_array.hpp"

namespace sensor_manager {

/**
 * @brief Structure to manage Camera object information and ID together.
 */
struct CameraObject {
  uint32_t id;
  vision_msgs::msg::BoundingBox2D bbox;
};

class FrameConverter {
 public:
  FrameConverter();
  ~FrameConverter();

  /**
   * @brief Converts Mono ToF sensor coordinate data to Robot coordinate system.
   *
   * @param[in] input_dist 1D ToF distance data in sensor coordinate system.
   * @param[in] mono_tof_sensor_frame_pose Sensor frame position information relative to base_link.
   * @return Position based on robot coordinate system (base_link).
   */
  Point TfMonoTofSensor2RobotFrame(const double input_dist,
                                   Pose mono_tof_sensor_frame_pose);

  /**
   * @brief Converts Multi ToF sensor distance data to Robot coordinate system.
   *
   * @param[in] tof_dists Sensor distance data.
   * @param[in] multi_tof_sensor_frame_pose Sensor frame position information relative to base_link.
   * @return Position based on robot coordinate system (base_link).
   */
  std::vector<Point> TfMultiTofSensor2RobotFrame(
      const std::vector<double>& tof_dists, const std::vector<double>& y_tan,
      const std::vector<double>& z_tan,
      const Pose& multi_tof_sensor_frame_pose);

  /**
   * @brief Converts Multi ToF sensor distance data to position in Sensor coordinate system.
   *
   * @param[in] tof_dists Sensor distance data.
   * @return Position based on sensor coordinate system.
   */
  std::vector<Point> TfMultiTofDistance2SensorFrame(
      const std::vector<double>& tof_dists, const std::vector<double>& y_tan,
      const std::vector<double>& z_tan);

  /**
   * @brief Converts Camera sensor data to object list based on Sensor Frame.
   *
   * @param[in] camera_msg Camera sensor data (CameraDataArray).
   * @param[in] direction Object recognition direction (CCW+, currently fixed to true).
   * @param[in] object_max_distance Object recognition max distance limit [m].
   * @return Object list based on sensor coordinate system.
   */
  std::vector<CameraObject> TfCameraSensor2SensorFrame(
      const robot_custom_msgs::msg::CameraDataArray* camera_msg, bool direction,
      double object_max_distance);

  /**
   * @brief Converts object list based on Sensor coordinate system to Robot coordinate system (base_link).
   *
   * @param[in] objects_sensor Object list based on sensor coordinate system.
   * @param[in] sensor_frame_pose Sensor frame position information relative to base_link.
   * @return Object list based on robot coordinate system (base_link).
   */
  std::vector<CameraObject> TfCameraObjects2RobotFrame(
      const std::vector<CameraObject>& objects_sensor,
      const Pose& sensor_frame_pose);

  /**
   * @brief Converts object list based on Robot coordinate system to Global coordinate system (map).
   *
   * @param[in] objects_robot Object list based on robot coordinate system.
   * @param[in] robot_pose Robot position information relative to map.
   * @return Object list based on global coordinate system (map).
   */
  std::vector<CameraObject> TfCameraObjects2GlobalFrame(
      const std::vector<CameraObject>& objects_robot, const Pose& robot_pose);

  /**
   * @brief Converts CameraObject list to BoundingBox2DArray message.
   *
   * @param[in] objects Object list to convert.
   * @return BoundingBox2DArray
   */
  vision_msgs::msg::BoundingBox2DArray ToBBoxArray(
      const std::vector<CameraObject>& objects);

  /**
   * @brief Converts Sensor coordinate system position to Robot coordinate system position (Single Point).
   *
   * @param[in] pt_sensor Position data based on sensor coordinate system.
   * @param[in] sensor_frame_pose Sensor frame position information relative to base_link.
   * @return Position based on robot coordinate system.
   */
  Point TfSensorFrame2RobotFrame(const Point& pt_sensor,
                                 const Pose& sensor_frame_pose);

  /**
   * @brief Converts Sensor coordinate system position to Robot coordinate system position (Point List).
   *
   * @param[in] pts_sensor Position data based on sensor coordinate system.
   * @param[in] sensor_frame_pose Sensor frame position information relative to base_link.
   * @return Position based on robot coordinate system.
   */
  std::vector<Point> TfSensorFrame2RobotFrame(
      const std::vector<Point>& pts_sensor, const Pose& sensor_frame_pose);

  /**
   * @brief Converts Camera sensor data to Bounding Box Array based on Sensor Frame.
   *
   * @param[in] camera_msg Camera sensor data (CameraDataArray).
   * @param[in] direction Object recognition direction (CCW+, currently fixed to true).
   * @param[in] object_max_distance Object recognition max distance limit [m].
   * @param[in] child_frame Sensor coordinate system name.
   * @return Bounding Box Array data based on Sensor Frame.
   */
  vision_msgs::msg::BoundingBox2DArray TfCameraSensor2SensorFrameBBoxArray(
      const robot_custom_msgs::msg::CameraDataArray* camera_msg, bool direction,
      double object_max_distance, std::string child_frame);

  /**
   * @brief Converts Bounding Box Array based on Sensor coordinate system to Robot/Target coordinate system (Camera only).
   *
   * @param[in] bbox_array_sensor BBox Array based on sensor coordinate system.
   * @param[in] robot_pose Current position of the robot (Required when Target Frame is map).
   * @param[in] target_frame Target coordinate system to convert ("base_link" or "map").
   * @param[in] camera_sensor_frame_pose Position information of camera sensor relative to robot (base_link).
   * @param[in] class_id_confidence_th Object id and confidence threshold matching data.
   * @param[in] use_object_logger Whether to use object recognition logging.
   * @param[in] logger_dist_margin Logging distance threshold.
   * @param[in] logger rclcpp logger object.
   * @return Bounding Box Array based on Target Frame.
   */
  vision_msgs::msg::BoundingBox2DArray TfCameraSensorFrameBBoxArray2TargetFrame(
      const vision_msgs::msg::BoundingBox2DArray& bbox_array_sensor,
      Pose& robot_pose, std::string target_frame, Pose camera_sensor_frame_pose,
      std::map<int, int> class_id_confidence_th = {},
      bool use_object_logger = false, double logger_dist_margin = 0.0,
      rclcpp::Logger logger = rclcpp::get_logger("FrameConverter"));

  /**
   * @brief Returns the activation position of the Bottom IR sensor based on the Sensor Frame.
   *
   * @param[in] cliff_msg IR sensor data (BottomIrData).
   * @param[in] distance_center_to_front_ir_sensor Distance from robot center to front IR sensor [m].
   * @param[in] angle_to_next_ir_sensor Angle interval between sensors [deg].
   * @return List of activated sensor positions based on Sensor Frame.
   */
  std::vector<Point> TfBottomIrSensor2SensorFrame(
      const robot_custom_msgs::msg::BottomIrData* cliff_msg,
      double distance_center_to_front_ir_sensor,
      double angle_to_next_ir_sensor);

  /**
   * @brief Returns the activation position of the Bottom IR sensor based on the Robot Frame.
   *
   * @param[in] cliff_msg IR sensor data (BottomIrData).
   * @param[in] distance_center_to_front_ir_sensor Distance from robot center to front IR sensor [m].
   * @param[in] angle_to_next_ir_sensor Angle interval between sensors [deg].
   * @return List of activated sensor positions based on Robot Frame.
   */
  std::vector<Point> TfBottomIrSensor2RobotFrame(
      const robot_custom_msgs::msg::BottomIrData* cliff_msg,
      double distance_center_to_front_ir_sensor,
      double angle_to_next_ir_sensor);

  /**
   * @brief Converts Collision event to coordinates based on Sensor Frame.
   *
   * @param[in] collision_msg Collision event data.
   * @param[in] offset_m Front obstacle generation offset based on sensor [m].
   * @return Collision detection point based on Sensor Frame.
   */
  Point TfCollisionData2SensorFrame(
      const robot_custom_msgs::msg::AbnormalEventData* collision_msg,
      double offset_m);

  /**
   * @brief Converts Collision event to coordinates based on Robot Frame.
   *
   * @param[in] collision_msg Collision event data.
   * @param[in] offset_m Front obstacle generation offset based on sensor [m].
   * @return Collision detection point based on Robot Frame.
   */
  Point TfCollisionData2RobotFrame(
      const robot_custom_msgs::msg::AbnormalEventData* collision_msg,
      double offset_m);

  /**
   * @brief Robot coordinate system -> Global coordinate system conversion function.
   *
   * @param[in] input_points_on_robot_frame Coordinate data based on robot coordinate system.
   * @param[in] robot_pose Current position data of the robot.
   * @return Position based on Global (map).
   */
  std::vector<Point> TfRobot2GlobalFrame(
      const std::vector<Point>& input_points_on_robot_frame, Pose robot_pose);
  std::vector<Point> TfRobot2GlobalFrame(
      const Point& input_point_on_robot_frame, Pose robot_pose);

  /**
   * @brief Clears the history of logged object information.
   */
  void LoggedObjectInfoClear() { logged_objects_.clear(); };

 private:
  // tof mono
  bool tof_mono_extrinsics_updated_ = false;
  double tof_mono_sensor_frame_pitch_cosine_ = 0.0;
  double tof_mono_sensor_frame_pitch_sine_ = 0.0;

  // tof multi
  bool tof_multi_extrinsics_updated_ = false;
  double multi_tof_sensor_frame_yaw_cosine_ = 0.0;
  double multi_tof_sensor_frame_yaw_sine_ = 0.0;
  double multi_tof_sensor_frame_pitch_cosine_ = 0.0;
  double multi_tof_sensor_frame_pitch_sine_ = 0.0;

  // camera
  std::map<int, std::vector<vision_msgs::msg::BoundingBox2D>> logged_objects_;

  // bottom ir
  bool bottom_ir_extrinsics_updated_ = false;
  std::vector<Point> bottom_ir_sensor_positions_;
};

}  // namespace sensor_manager

#endif  // AIRBOT_SENSOR_MANAGER_UTILS_FRAME_CONVERTER_HPP_
