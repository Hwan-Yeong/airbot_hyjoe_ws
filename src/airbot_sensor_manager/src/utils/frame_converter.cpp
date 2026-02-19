#include "utils/frame_converter.hpp"

namespace sensor_manager {

FrameConverter::FrameConverter() {}

FrameConverter::FrameConverter(std::shared_ptr<tf2_ros::Buffer> tf_buffer)
    : tf_buffer_(tf_buffer) {}

FrameConverter::~FrameConverter() {}

Point FrameConverter::TfMonoTofSensor2RobotFrame(
    const double input_dist, Pose mono_tof_sensor_frame_pose) {
  Point point_s(input_dist, 0.0, 0.0);
  return TfSensorFrame2RobotFrame(point_s, mono_tof_sensor_frame_pose);
}

std::vector<Point> FrameConverter::TfMultiTofSensor2RobotFrame(
    const std::vector<double>& tof_dists, const std::vector<double>& y_tan,
    const std::vector<double>& z_tan, const Pose& multi_tof_sensor_frame_pose) {
  std::vector<Point> points_s =
      TfMultiTofDistance2SensorFrame(tof_dists, y_tan, z_tan);
  return TfSensorFrame2RobotFrame(points_s, multi_tof_sensor_frame_pose);
}

std::vector<Point> FrameConverter::TfMultiTofDistance2SensorFrame(
    const std::vector<double>& tof_dists, const std::vector<double>& y_tan,
    const std::vector<double>& z_tan) {
  std::vector<Point> points;
  if (tof_dists.size() != y_tan.size() || tof_dists.size() != z_tan.size())
    return points;

  for (size_t i = 0; i < tof_dists.size(); ++i) {
    double dist = tof_dists[i];
    Point p_sensor;

    if (dist <= 1e-6) {
      p_sensor.x = std::numeric_limits<double>::quiet_NaN();
      p_sensor.y = std::numeric_limits<double>::quiet_NaN();
      p_sensor.z = std::numeric_limits<double>::quiet_NaN();
    } else {
      p_sensor.x = dist;
      p_sensor.y = dist * y_tan[i];
      p_sensor.z = dist * z_tan[i];
    }
    points.push_back(p_sensor);
  }
  return points;
}

Point FrameConverter::TfSensorFrame2RobotFrame(const Point& p_s,
                                               const Pose& sensor_frame_pose) {
  if (std::isnan(p_s.x)) return p_s;

  geometry_msgs::msg::PointStamped ps_in, ps_out;
  ps_in.point.x = p_s.x;
  ps_in.point.y = p_s.y;
  ps_in.point.z = p_s.z;

  tf2::Transform tf;
  tf.setOrigin(tf2::Vector3(sensor_frame_pose.position.x,
                            sensor_frame_pose.position.y,
                            sensor_frame_pose.position.z));
  tf2::Quaternion q;
  q.setRPY(sensor_frame_pose.orientation.roll,
           sensor_frame_pose.orientation.pitch,
           sensor_frame_pose.orientation.yaw);
  tf.setRotation(q);

  tf2::Vector3 pt_in(p_s.x, p_s.y, p_s.z);
  tf2::Vector3 pt_out = tf * pt_in; // p_robot = R * p_sensor + t

  return Point(pt_out.x(), pt_out.y(), pt_out.z());
}

std::vector<Point> FrameConverter::TfSensorFrame2RobotFrame(
    const std::vector<Point>& pts_sensor, const Pose& sensor_frame_pose) {
  std::vector<Point> pts_robot;
  pts_robot.reserve(pts_sensor.size());

  tf2::Transform tf;
  tf.setOrigin(tf2::Vector3(sensor_frame_pose.position.x,
                            sensor_frame_pose.position.y,
                            sensor_frame_pose.position.z));
  tf2::Quaternion q;
  q.setRPY(sensor_frame_pose.orientation.roll,
           sensor_frame_pose.orientation.pitch,
           sensor_frame_pose.orientation.yaw);
  tf.setRotation(q);

  for (const auto& p_s : pts_sensor) {
    if (std::isnan(p_s.x)) {
      pts_robot.push_back(p_s);
      continue;
    }
    tf2::Vector3 pt_in(p_s.x, p_s.y, p_s.z);
    tf2::Vector3 pt_out = tf * pt_in;
    pts_robot.emplace_back(pt_out.x(), pt_out.y(), pt_out.z());
  }
  return pts_robot;
}

std::vector<CameraObject> FrameConverter::TfCameraSensor2SensorFrame(
    const robot_custom_msgs::msg::CameraDataArray* camera_msg, bool direction,
    double object_max_distance) {
  std::vector<CameraObject> objects;
  if (camera_msg->data_array.empty() || camera_msg->num == 0)
    return objects;

  for (const auto& obj : camera_msg->data_array) {
    if (obj.distance > object_max_distance)
      continue;
    if (obj.height < 0.0 || obj.width < 0.0)
      continue;

    CameraObject cam_obj;
    cam_obj.id = obj.id;

    double calculated_height = std::min(static_cast<double>(obj.width), 0.3);
    double theta_val = direction ? obj.theta : -obj.theta;

    cam_obj.bbox.center.position.x =
        obj.distance * std::cos(theta_val) + calculated_height / 2.0;
    cam_obj.bbox.center.position.y = obj.distance * std::sin(theta_val);
    cam_obj.bbox.center.theta = 0.0;
    cam_obj.bbox.size_x = calculated_height;
    if (obj.id == 17)
      cam_obj.bbox.size_y = std::min(static_cast<double>(obj.width), 0.55);
    else
      cam_obj.bbox.size_y = obj.width;

    objects.push_back(cam_obj);
  }
  return objects;
}

std::vector<CameraObject> FrameConverter::TfCameraObjects2RobotFrame(
    const std::vector<CameraObject>& objects_sensor,
    const Pose& sensor_frame_pose) {
  std::vector<CameraObject> objects_robot;
  for (auto obj_r : objects_sensor) {
    Point p_s;
    p_s.x = obj_r.bbox.center.position.x;
    p_s.y = obj_r.bbox.center.position.y;
    p_s.z = 0.0;

    Point p_r = TfSensorFrame2RobotFrame(p_s, sensor_frame_pose);
    obj_r.bbox.center.position.x = p_r.x;
    obj_r.bbox.center.position.y = p_r.y;

    objects_robot.push_back(obj_r);
  }
  return objects_robot;
}

std::vector<CameraObject> FrameConverter::TfCameraObjects2GlobalFrame(
    const std::vector<CameraObject>& objects_robot, const Pose& robot_pose) {
  std::vector<CameraObject> objects_global;
  objects_global.reserve(objects_robot.size());

  tf2::Transform tf;
  tf.setOrigin(tf2::Vector3(robot_pose.position.x, robot_pose.position.y,
                            robot_pose.position.z));
  tf2::Quaternion q;
  q.setRPY(robot_pose.orientation.roll, robot_pose.orientation.pitch,
           robot_pose.orientation.yaw);
  tf.setRotation(q);

  for (auto obj_g : objects_robot) {
    tf2::Vector3 pt_in(obj_g.bbox.center.position.x,
                       obj_g.bbox.center.position.y, 0.0);
    tf2::Vector3 pt_out = tf * pt_in;
    obj_g.bbox.center.position.x = pt_out.x();
    obj_g.bbox.center.position.y = pt_out.y();
    objects_global.push_back(obj_g);
  }
  return objects_global;
}

vision_msgs::msg::BoundingBox2DArray FrameConverter::ToBBoxArray(
    const std::vector<CameraObject>& objects) {
  vision_msgs::msg::BoundingBox2DArray bbox_array;

  for (const auto& obj : objects) {
    bbox_array.boxes.push_back(obj.bbox);
  }
  return bbox_array;
}

std::vector<Point> FrameConverter::TfBottomIrSensor2SensorFrame(
    const robot_custom_msgs::msg::BottomIrData* cliff_msg,
    double distance_center_to_front_ir_sensor, double angle_to_next_ir_sensor) {
  std::vector<Point> active_sensor_points;
  if (!bottom_ir_extrinsics_updated_) {
    double d = distance_center_to_front_ir_sensor;
    double deg_1 = 0;
    double deg_2 = angle_to_next_ir_sensor;
    double deg_3 = 180 - angle_to_next_ir_sensor;
    double deg_4 = 180;
    double deg_5 = 180 + angle_to_next_ir_sensor;
    double deg_6 = 360 - angle_to_next_ir_sensor;

    bottom_ir_sensor_positions_ = {
        Point(d * std::cos(deg_1 * M_PI / 180),
              d * std::sin(deg_1 * M_PI / 180), 0.0),
        Point(d * std::cos(deg_2 * M_PI / 180),
              d * std::sin(deg_2 * M_PI / 180), 0.0),
        Point(d * std::cos(deg_3 * M_PI / 180),
              d * std::sin(deg_3 * M_PI / 180), 0.0),
        Point(d * std::cos(deg_4 * M_PI / 180),
              d * std::sin(deg_4 * M_PI / 180), 0.0),
        Point(d * std::cos(deg_5 * M_PI / 180),
              d * std::sin(deg_5 * M_PI / 180), 0.0),
        Point(d * std::cos(deg_6 * M_PI / 180),
              d * std::sin(deg_6 * M_PI / 180), 0.0)};
    bottom_ir_extrinsics_updated_ = true;
  }

  if (cliff_msg->ff)
    active_sensor_points.push_back(bottom_ir_sensor_positions_[0]);
  if (cliff_msg->fl)
    active_sensor_points.push_back(bottom_ir_sensor_positions_[1]);
  if (cliff_msg->bl)
    active_sensor_points.push_back(bottom_ir_sensor_positions_[2]);
  if (cliff_msg->bb)
    active_sensor_points.push_back(bottom_ir_sensor_positions_[3]);
  if (cliff_msg->br)
    active_sensor_points.push_back(bottom_ir_sensor_positions_[4]);
  if (cliff_msg->fr)
    active_sensor_points.push_back(bottom_ir_sensor_positions_[5]);

  return active_sensor_points;
}

Point FrameConverter::TfCollisionData2SensorFrame(
    const robot_custom_msgs::msg::AbnormalEventData* collision_msg,
    double offset_m) {
  Point point;
  // 1: 전방 충돌, -1: 후방 충돌
  if (collision_msg->event_trigger == 1 || collision_msg->event_trigger == -1) {
    point.x = offset_m;
    point.y = 0.0;
    point.z = 0.0;
  }
  return point;
}

Point FrameConverter::TfCollisionData2RobotFrame(
    const robot_custom_msgs::msg::AbnormalEventData* collision_msg,
    double offset_m) {
  Point p_s = TfCollisionData2SensorFrame(collision_msg, offset_m);

  return p_s;
}

std::vector<Point> FrameConverter::TfRobot2GlobalFrame(
    const std::vector<Point>& input_points_on_robot_frame, Pose robot_pose) {
  std::vector<Point> global_points;
  global_points.reserve(input_points_on_robot_frame.size());

  tf2::Transform tf;
  tf.setOrigin(tf2::Vector3(robot_pose.position.x, robot_pose.position.y,
                            robot_pose.position.z));
  tf2::Quaternion q;
  q.setRPY(robot_pose.orientation.roll, robot_pose.orientation.pitch,
           robot_pose.orientation.yaw);
  tf.setRotation(q);

  for (const auto& local_point : input_points_on_robot_frame) {
    tf2::Vector3 pt_in(local_point.x, local_point.y, local_point.z);
    tf2::Vector3 pt_out = tf * pt_in;
    global_points.emplace_back(pt_out.x(), pt_out.y(), pt_out.z());
  }

  return global_points;
}

std::vector<Point> FrameConverter::TfRobot2GlobalFrame(
    const Point& input_point_on_robot_frame, Pose robot_pose) {
  return TfRobot2GlobalFrame(std::vector<Point>{input_point_on_robot_frame},
                             robot_pose);
}

vision_msgs::msg::BoundingBox2DArray
FrameConverter::TfCameraSensor2SensorFrameBBoxArray(
    const robot_custom_msgs::msg::CameraDataArray* camera_msg, bool direction,
    double object_max_distance, std::string child_frame) {
  vision_msgs::msg::BoundingBox2DArray bbox_array;
  std::vector<CameraObject> objects =
      TfCameraSensor2SensorFrame(camera_msg, direction, object_max_distance);

  bbox_array = ToBBoxArray(objects);
  bbox_array.header.frame_id = child_frame;
  bbox_array.header.stamp = camera_msg->timestamp;

  return bbox_array;
}

vision_msgs::msg::BoundingBox2DArray
FrameConverter::TfCameraSensorFrameBBoxArray2TargetFrame(
    const vision_msgs::msg::BoundingBox2DArray& bbox_array_sensor,
    Pose& robot_pose, std::string target_frame, Pose camera_sensor_frame_pose) {
  vision_msgs::msg::BoundingBox2DArray bbox_array_target;
  bbox_array_target.header = bbox_array_sensor.header;
  bbox_array_target.header.frame_id = target_frame;

  std::vector<CameraObject> objects_sensor;
  for (const auto& bbox : bbox_array_sensor.boxes) {
    CameraObject obj;
    obj.bbox = bbox;
    objects_sensor.push_back(obj);
  }

  std::vector<CameraObject> objects_target;

  if (target_frame == "map") {
    // Sensor -> Robot -> Map
    std::vector<CameraObject> objects_robot =
        TfCameraObjects2RobotFrame(objects_sensor, camera_sensor_frame_pose);
    objects_target = TfCameraObjects2GlobalFrame(objects_robot, robot_pose);
  } else if (target_frame == "base_link") {
    // Sensor -> Robot
    objects_target =
        TfCameraObjects2RobotFrame(objects_sensor, camera_sensor_frame_pose);
  } else {
    return bbox_array_target;
  }

  bbox_array_target = ToBBoxArray(objects_target);
  return bbox_array_target;
}

}  // namespace sensor_manager
