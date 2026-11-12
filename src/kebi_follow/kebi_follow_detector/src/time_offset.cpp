#include "kebi_follow_detector/time_offset.hpp"

namespace kebi_follow
{

CameraTimeOffset::CameraTimeOffset(double window_s, double reset_jump_s)
: window_s_(window_s), reset_jump_s_(reset_jump_s)
{
}

void CameraTimeOffset::reset()
{
  samples_.clear();
  has_last_ = false;
}

void CameraTimeOffset::add(double ref_stamp, double image_stamp)
{
  if (has_last_ && ref_stamp < last_ref_ - reset_jump_s_) {
    reset();
  }
  has_last_ = true;
  last_ref_ = ref_stamp;
  const double value = ref_stamp - image_stamp;
  // monotonic queue: drop samples that can never be the minimum again
  while (!samples_.empty() && samples_.back().value >= value) {
    samples_.pop_back();
  }
  samples_.push_back({ref_stamp, value});
  while (samples_.front().ref < ref_stamp - window_s_) {
    samples_.pop_front();
  }
}

double CameraTimeOffset::offset() const
{
  return samples_.empty() ? 0.0 : samples_.front().value;
}

}  // namespace kebi_follow
