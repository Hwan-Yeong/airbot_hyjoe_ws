// Depth fusion: person/leg ground positions from the depth image and the RGB detections.
//
// Depth samples (every `depth_stride` pixels) are back-projected in the depth camera, moved to the RGB
// camera with the static TF, projected into the RGB image (plumb_bob distortion) and kept when they fall
// inside the leg region of a detection:
//   - pose model with a visible knee/ankle: hull of the visible knees/ankles (+ margin, down to the box
//     bottom); every leg with a visible knee or ankle also gets its own capsule region (knee-ankle)
//   - otherwise: the lower `leg_region_fraction` of the box, central `leg_region_width_fraction`
// Points are moved to base_link (pitch correction included by the caller), floor / robot body points are
// dropped, and the nearest dense range cluster gives the position (median x, y = ground projection,
// median z = height). The person position is the midpoint of both legs when both legs have a position.
#pragma once

#include <array>
#include <vector>

#include <opencv2/core.hpp>

#include "kebi_follow_detector/yolo_decode.hpp"

namespace kebi_follow
{

struct CameraModel
{
  int width{0};
  int height{0};
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  std::array<double, 5> dist{};  // plumb_bob k1 k2 p1 p2 k3

  bool valid() const {return width > 0 && height > 0 && fx > 0.0 && fy > 0.0;}
  bool has_distortion() const;
  // point in the optical frame (x right, y down, z forward) -> pixel; false if behind the camera
  bool project(double x, double y, double z, double & u, double & v) const;
};

// Rigid transform p_out = R p_in + t (R row-major)
struct Rigid3
{
  std::array<double, 9> r{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  std::array<double, 3> t{0.0, 0.0, 0.0};

  void apply(const double in[3], double out[3]) const;
  Rigid3 operator*(const Rigid3 & other) const;  // (a * b)(p) = a(b(p))
  static Rigid3 from_quaternion(
    double qx, double qy, double qz, double qw, double tx, double ty,
    double tz);
  static Rigid3 from_xyz_rpy(double x, double y, double z, double roll, double pitch, double yaw);
  static Rigid3 rotation_y(double angle);
};

struct FusionConfig
{
  int depth_stride{3};                  // use every n-th depth pixel in u and v
  double depth_unit_m{0.001};           // 16UC1 depth unit
  double depth_min_m{0.2};
  double depth_max_m{6.0};
  double nominal_depth_m{1.5};          // to place invalid depth pixels (valid ratio)
  double floor_height_m{0.05};          // base_link z below this is floor
  double max_height_m{2.0};
  double robot_exclusion_radius_m{0.30};  // points this close to base_link are the robot itself
  double leg_region_fraction{0.5};      // without keypoints: lower part of the box
  double leg_region_width_fraction{0.8};
  double keypoint_vis_threshold{0.5};
  double keypoint_margin_ratio{0.15};   // hull margin, relative to the box width
  double leg_half_width_ratio{0.3};     // leg capsule half width, relative to the knee-ankle length
  double leg_min_half_width_px{6.0};
  double leg_foot_extension_ratio{0.25};  // capsule extends below the ankle (feet), rel. to length
  double cluster_window_m{0.10};        // a cluster starts where this range window is dense enough
  int cluster_min_points{8};
  double cluster_min_fraction{0.05};
  double person_cluster_extent_m{0.45};  // range extent kept behind the near edge (person)
  double leg_cluster_extent_m{0.25};    // (single leg)
  double depth_noise_k{0.01};           // depth noise std [m] = k * range^2
  double spread_gain{0.5};              // weight of the point spread in position_std
  double position_std_min_m{0.05};
  double position_std_max_m{0.5};
  bool appearance{true};
  int hue_bins{8};
  int sat_bins{4};
  int gray_bins{4};
};

struct LegEstimate
{
  bool valid{false};
  int side{0};  // 0 left, 1 right (COCO keypoint sides)
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double position_std{0.0};
  double valid_ratio{0.0};
  int points{0};
  float score{0.0f};
  float u_min{0.0f};
  float v_min{0.0f};
  float u_max{0.0f};
  float v_max{0.0f};
};

struct PersonEstimate
{
  bool has_position{false};
  bool from_legs{false};  // midpoint of two leg positions
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double position_std{0.0};
  double valid_ratio{0.0};
  int points{0};
  std::array<LegEstimate, 2> legs{};
  std::vector<float> appearance;
};

class DepthFusion
{
public:
  explicit DepthFusion(const FusionConfig & config = FusionConfig());

  // rgb_from_depth maps depth-optical points to the RGB optical frame, base_from_rgb maps RGB optical
  // points to base_link (include any pitch correction).
  void set_geometry(
    const CameraModel & rgb, const CameraModel & depth, const Rigid3 & rgb_from_depth,
    const Rigid3 & base_from_rgb);
  bool ready() const {return ready_;}
  const FusionConfig & config() const {return config_;}

  // depth: CV_16UC1 (x depth_unit_m) or CV_32FC1 [m] at the depth camera_info size.
  // color: image of the RGB camera (CV_8UC3, BGR if color_is_bgr else RGB) for the appearance; may be empty.
  // detections: in RGB image pixels. Returns one estimate per detection (same order).
  std::vector<PersonEstimate> fuse(
    const cv::Mat & depth, const cv::Mat & color, bool color_is_bgr,
    const std::vector<Detection> & detections) const;

private:
  FusionConfig config_;
  bool ready_{false};
  CameraModel rgb_;
  CameraModel depth_;
  Rigid3 rgb_from_depth_;
  Rigid3 base_from_rgb_;
  int grid_w_{0};
  int grid_h_{0};
  float parallax_margin_px_{0.0f};
  std::vector<float> ray_x_;      // normalized depth-camera rays of the sample grid
  std::vector<float> ray_y_;
  std::vector<float> nominal_u_;  // RGB pixel of each sample at nominal_depth_m (NaN if not visible)
  std::vector<float> nominal_v_;
};

}  // namespace kebi_follow
