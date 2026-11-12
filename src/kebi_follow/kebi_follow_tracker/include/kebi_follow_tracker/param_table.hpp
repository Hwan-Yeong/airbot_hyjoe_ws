// 파라미터 이름 ↔ 필드 목록. ROS 노드 선언, bag 재생 도구 덮어쓰기, config 일치 시험이 같이 쓴다.
#pragma once

#include "kebi_follow_tracker/params.hpp"

namespace kebi_follow {
namespace tracker {

// visit(name, field&) 를 모든 파라미터에 호출한다 (field: double, int, bool)
template <typename Visitor>
void visitParams(TrackerParams& p, Visitor&& visit) {
  visit("scan.robot_radius", p.scan.robot_radius);
  visit("scan.body_margin", p.scan.body_margin);
  visit("scan.max_range", p.scan.max_range);
  visit("scan.jump_min", p.scan.jump_min);
  visit("scan.jump_ratio", p.scan.jump_ratio);
  visit("scan.max_invalid_gap", p.scan.max_invalid_gap);

  visit("leg.min_width", p.leg.min_width);
  visit("leg.max_width", p.leg.max_width);
  visit("leg.min_points", p.leg.min_points);
  visit("leg.min_point_width", p.leg.min_point_width);
  visit("leg.concave_tol", p.leg.concave_tol);
  visit("leg.max_occluder_gap", p.leg.max_occluder_gap);
  visit("leg.arc_min_points", p.leg.arc_min_points);
  visit("leg.arc_max_radius", p.leg.arc_max_radius);
  visit("leg.arc_max_residual", p.leg.arc_max_residual);
  visit("leg.arc_fail_score", p.leg.arc_fail_score);
  visit("leg.blob_min_width", p.leg.blob_min_width);
  visit("leg.blob_max_width", p.leg.blob_max_width);
  visit("leg.blob_min_points", p.leg.blob_min_points);
  visit("leg.pair_min_sep", p.leg.pair_min_sep);
  visit("leg.pair_max_sep", p.leg.pair_max_sep);
  visit("leg.max_leg_spread", p.leg.max_leg_spread);
  visit("leg.center_offset_ratio", p.leg.center_offset_ratio);
  visit("leg.center_offset_max_radius", p.leg.center_offset_max_radius);

  visit("kf.accel_std", p.kf.accel_std);
  visit("kf.init_vel_std", p.kf.init_vel_std);
  visit("kf.meas_std_pair", p.kf.meas_std_pair);
  visit("kf.meas_std_merged", p.kf.meas_std_merged);
  visit("kf.meas_std_single", p.kf.meas_std_single);
  visit("kf.meas_std_single_standing", p.kf.meas_std_single_standing);
  visit("kf.single_std_speed", p.kf.single_std_speed);
  visit("kf.max_speed", p.kf.max_speed);
  visit("kf.coast_tau_s", p.kf.coast_tau_s);
  visit("kf.static_meas_std", p.kf.static_meas_std);
  visit("kf.static_accel_std", p.kf.static_accel_std);

  visit("assoc.gate_chi2", p.assoc.gate_chi2);
  visit("assoc.max_gate_dist", p.assoc.max_gate_dist);
  visit("assoc.segment_bonus", p.assoc.segment_bonus);

  visit("track.confirm_hits", p.track.confirm_hits);
  visit("track.tentative_max_misses", p.track.tentative_max_misses);
  visit("track.delete_s", p.track.delete_s);
  visit("track.delete_std", p.track.delete_std);
  visit("track.birth_suppress_radius", p.track.birth_suppress_radius);
  visit("track.duplicate_radius", p.track.duplicate_radius);
  visit("track.allow_single_birth", p.track.allow_single_birth);
  visit("track.shape_alpha", p.track.shape_alpha);
  visit("track.pair_shape", p.track.pair_shape);
  visit("track.merged_shape", p.track.merged_shape);
  visit("track.single_shape", p.track.single_shape);
  visit("track.static_shape_factor", p.track.static_shape_factor);
  visit("track.moved_dist", p.track.moved_dist);
  visit("track.moved_scans", p.track.moved_scans);
  visit("track.static_min_s", p.track.static_min_s);
  visit("track.static_min_hits", p.track.static_min_hits);
  visit("track.merge_radius", p.track.merge_radius);
  visit("track.merge_scans", p.track.merge_scans);
  visit("track.max_tracks", p.track.max_tracks);

  visit("camera.gate", p.camera.gate);
  visit("camera.confirm_window_s", p.camera.confirm_window_s);
  visit("camera.max_age_s", p.camera.max_age_s);
  visit("camera.min_score", p.camera.min_score);
  visit("camera.person_weight", p.camera.person_weight);
  visit("camera.alive_s", p.camera.alive_s);
  visit("camera.check_half_fov_deg", p.camera.check_half_fov_deg);
  visit("camera.check_min_range", p.camera.check_min_range);
  visit("camera.check_max_range", p.camera.check_max_range);

  visit("acquire.require_camera", p.acquire.require_camera);
  visit("acquire.min_range", p.acquire.min_range);
  visit("acquire.max_range", p.acquire.max_range);
  visit("acquire.max_bearing_deg", p.acquire.max_bearing_deg);
  visit("acquire.confirm_s", p.acquire.confirm_s);
  visit("acquire.min_shape_score", p.acquire.min_shape_score);
  visit("acquire.max_coast_s", p.acquire.max_coast_s);
  visit("acquire.competitor_margin", p.acquire.competitor_margin);
  visit("acquire.competitor_margin_deg", p.acquire.competitor_margin_deg);

  visit("target.lost_s", p.target.lost_s);
  visit("target.max_std", p.target.max_std);
  visit("target.reacquire_radius", p.target.reacquire_radius);
  visit("target.reacquire_confirm_s", p.target.reacquire_confirm_s);
  visit("target.reacquire_from_last_seen", p.target.reacquire_from_last_seen);
  visit("target.camera_lost_s", p.target.camera_lost_s);

  visit("ambiguity.radius", p.ambiguity.radius);
  visit("ambiguity.cost_margin", p.ambiguity.cost_margin);
  visit("ambiguity.min_person_score", p.ambiguity.min_person_score);
  visit("ambiguity.lost_s", p.ambiguity.lost_s);
}

}  // namespace tracker
}  // namespace kebi_follow
