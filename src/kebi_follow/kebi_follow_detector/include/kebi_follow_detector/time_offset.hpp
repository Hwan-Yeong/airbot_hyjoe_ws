// Camera clock -> robot (AMR) clock offset estimator (SPEC section 3).
//
// Camera stamps use the Jetson clock, /odom and TF use the AMR clock (+9 h by design). For every image
// we take one sample  value = (latest /odom stamp when the image arrives) - (image stamp)  and use the
// minimum over a sliding window (the sample with the least transport delay). Output stamps are
//   stamp_out = stamp_cam + offset - camera_min_latency_s.
// Only message stamps are used, so live operation and bag replay (--clock) behave the same.
#pragma once

#include <cstddef>
#include <deque>

namespace kebi_follow
{

class CameraTimeOffset
{
public:
  // window_s: sliding window length on the reference (odom) time axis.
  // reset_jump_s: a backwards jump of the reference time larger than this clears the window
  //               (bag restart / clock reset).
  explicit CameraTimeOffset(double window_s = 5.0, double reset_jump_s = 1.0);

  // ref_stamp: latest /odom stamp [s] at image arrival, image_stamp: image header stamp [s]
  void add(double ref_stamp, double image_stamp);
  void reset();

  bool valid() const {return !samples_.empty();}
  // offset [s] (robot clock - camera clock, including the minimum transport delay); 0 if !valid()
  double offset() const;
  double to_robot_time(double image_stamp, double min_latency_s) const
  {
    return image_stamp + offset() - min_latency_s;
  }
  size_t size() const {return samples_.size();}

private:
  struct Sample
  {
    double ref;
    double value;
  };
  double window_s_;
  double reset_jump_s_;
  double last_ref_{0.0};
  bool has_last_{false};
  std::deque<Sample> samples_;  // values increase from front to back (monotonic queue)
};

}  // namespace kebi_follow
