// 합성 시나리오 평가: 표준 시나리오 x (라이다만, 카메라) x seed 를 돌려 지표 표를 출력하고 CSV 를 쓴다.
// 사용: scenario_eval [--seeds N] [--out DIR] [--only 이름일부] [--dump] [--set 이름=값]...
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <type_traits>
#include <vector>

#include "kebi_follow_tracker/param_table.hpp"
#include "sim/scenario_library.hpp"

namespace ks = kebi_follow::tracker::sim;

namespace {

struct Aggregate {
  int runs{0};
  int locked_runs{0};
  int correct_first_lock{0};
  double lock_time_sum{0.0};
  double lock_time_max{0.0};
  int false_locks{0};
  int id_switches{0};
  int reacquires{0};
  int wrong_scans{0};
  int locked_scans{0};
  int occluded_scans{0};
  int lost_scans{0};
  int ambiguous_scans{0};
  double coverage_sum{0.0};
  double coverage_min{1.0};
  double err_rms_sum{0.0};
  double err_p95_max{0.0};
  double vel_err_sum{0.0};
  double lag_sum{0.0};
  double false_person_sum{0.0};
  double proc_mean_sum{0.0};
  double proc_p99_max{0.0};
  double proc_max{0.0};
};

void writeRecords(const std::string& path, const ks::ScenarioResult& res) {
  FILE* f = std::fopen(path.c_str(), "w");
  if (f == nullptr) {
    return;
  }
  std::fprintf(
      f,
      "t,mode,status,track_id,ambiguous,camera_confirmed,est_x,est_y,truth_x,"
      "truth_y,"
      "matched,n_tracks,n_confirmed,n_false_person,n_target_legs,proc_ms\n");
  for (const auto& r : res.records) {
    std::fprintf(
        f, "%.3f,%d,%d,%d,%d,%d,%.4f,%.4f,%.4f,%.4f,%d,%d,%d,%d,%d,%.4f\n", r.t,
        static_cast<int>(r.mode), static_cast<int>(r.status), r.track_id,
        r.ambiguous ? 1 : 0, r.camera_confirmed ? 1 : 0, r.est.x, r.est.y,
        r.truth.x, r.truth.y, r.matched_walker, r.n_tracks, r.n_confirmed,
        r.n_false_person, r.n_target_legs, r.proc_ms);
  }
  std::fclose(f);
}

bool setParam(kebi_follow::tracker::TrackerParams& p,
              const std::string& assignment) {
  const auto eq = assignment.find('=');
  if (eq == std::string::npos) {
    return false;
  }
  const std::string name = assignment.substr(0, eq);
  const std::string value = assignment.substr(eq + 1);
  bool found = false;
  kebi_follow::tracker::visitParams(p, [&](const char* key, auto& field) {
    if (name != key) {
      return;
    }
    found = true;
    using T = std::decay_t<decltype(field)>;
    if constexpr (std::is_same_v<T, bool>) {
      field = value == "true" || value == "1";
    } else if constexpr (std::is_same_v<T, int>) {
      field = std::atoi(value.c_str());
    } else {
      field = std::atof(value.c_str());
    }
  });
  return found;
}

}  // namespace

int main(int argc, char** argv) {
  int seeds = 10;
  std::string out_dir;
  std::string only;
  bool dump = false;
  std::vector<std::string> sets;
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--set") && i + 1 < argc) {
      sets.push_back(argv[++i]);
      continue;
    }
    if (!std::strcmp(argv[i], "--seeds") && i + 1 < argc) {
      seeds = std::atoi(argv[++i]);
    } else if (!std::strcmp(argv[i], "--out") && i + 1 < argc) {
      out_dir = argv[++i];
    } else if (!std::strcmp(argv[i], "--only") && i + 1 < argc) {
      only = argv[++i];
    } else if (!std::strcmp(argv[i], "--dump")) {
      dump = true;
    }
  }

  std::vector<std::string> order;
  std::map<std::string, Aggregate> agg;
  FILE* runs_csv = nullptr;
  if (!out_dir.empty()) {
    runs_csv = std::fopen((out_dir + "/scenario_runs.csv").c_str(), "w");
    if (runs_csv != nullptr) {
      std::fprintf(runs_csv,
                   "scenario,seed,lock_time,lock_walker,false_locks,id_"
                   "switches,reacquires,"
                   "wrong_scans,locked_scans,occluded_scans,lost_scans,"
                   "ambiguous_scans,coverage,err_rms,"
                   "err_p95,err_max,vel_err_rms,lag_ms,false_person_mean,false_"
                   "person_ids,proc_mean_ms,"
                   "proc_p99_ms,proc_max_ms\n");
    }
  }

  for (bool camera : {false, true}) {
    for (int seed = 1; seed <= seeds; ++seed) {
      for (auto spec : ks::allScenarios(camera, static_cast<unsigned>(seed))) {
        if (!only.empty() && spec.name.find(only) == std::string::npos) {
          continue;
        }
        for (const auto& kv : sets) {
          if (!setParam(spec.params, kv)) {
            std::fprintf(stderr, "unknown parameter: %s\n", kv.c_str());
            return 1;
          }
        }
        const ks::ScenarioResult res = ks::runScenario(spec);
        const ks::ScenarioMetrics& m = res.metrics;
        if (agg.find(m.name) == agg.end()) {
          order.push_back(m.name);
        }
        Aggregate& a = agg[m.name];
        ++a.runs;
        if (m.lock_time >= 0.0) {
          ++a.locked_runs;
          a.lock_time_sum += m.lock_time;
          a.lock_time_max = std::max(a.lock_time_max, m.lock_time);
          if (m.lock_walker == spec.target_walker) {
            ++a.correct_first_lock;
          }
        }
        a.false_locks += m.false_locks;
        a.id_switches += m.id_switches;
        a.reacquires += m.reacquires;
        a.wrong_scans += m.wrong_scans;
        a.locked_scans += m.locked_scans;
        a.occluded_scans += m.occluded_scans;
        a.lost_scans += m.lost_scans;
        a.ambiguous_scans += m.ambiguous_scans;
        a.coverage_sum += m.track_coverage;
        a.coverage_min = std::min(a.coverage_min, m.track_coverage);
        a.err_rms_sum += m.err_rms;
        a.err_p95_max = std::max(a.err_p95_max, m.err_p95);
        a.vel_err_sum += m.vel_err_rms;
        a.lag_sum += m.lag_ms;
        a.false_person_sum += m.false_person_mean;
        a.proc_mean_sum += m.proc_mean_ms;
        a.proc_p99_max = std::max(a.proc_p99_max, m.proc_p99_ms);
        a.proc_max = std::max(a.proc_max, m.proc_max_ms);
        if (runs_csv != nullptr) {
          std::fprintf(runs_csv,
                       "%s,%d,%.3f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.4f,%.4f,%.4f,%."
                       "4f,%.4f,%.1f,%.4f,%d,%.4f,"
                       "%.4f,%.4f\n",
                       m.name.c_str(), seed, m.lock_time, m.lock_walker,
                       m.false_locks, m.id_switches, m.reacquires,
                       m.wrong_scans, m.locked_scans, m.occluded_scans,
                       m.lost_scans, m.ambiguous_scans, m.track_coverage,
                       m.err_rms, m.err_p95, m.err_max, m.vel_err_rms, m.lag_ms,
                       m.false_person_mean, m.false_person_ids, m.proc_mean_ms,
                       m.proc_p99_ms, m.proc_max_ms);
        }
        if (dump && seed == 1 && !out_dir.empty()) {
          writeRecords(out_dir + "/rec_" + m.name + ".csv", res);
        }
      }
    }
  }
  if (runs_csv != nullptr) {
    std::fclose(runs_csv);
  }

  std::printf(
      "| scenario | runs | locked | correct 1st lock | lock time mean/max [s] "
      "| false locks | "
      "ID switches | reacq | wrong scans | coverage mean/min | err RMS [m] | "
      "err p95 max [m] | "
      "vel err [m/s] | lag [ms] | occl/lost/ambig scans | false person "
      "trk/scan | "
      "proc mean/p99/max [ms] |\n");
  std::printf(
      "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"
      "\n");
  for (const auto& name : order) {
    const Aggregate& a = agg[name];
    const double n = a.runs;
    std::printf(
        "| %s | %d | %d | %d | %.2f / %.2f | %d | %d | %d | %d | %.3f / %.3f | "
        "%.3f | %.3f | %.3f | "
        "%.0f | %d/%d/%d | %.3f | %.3f / %.3f / %.3f |\n",
        name.c_str(), a.runs, a.locked_runs, a.correct_first_lock,
        a.locked_runs ? a.lock_time_sum / a.locked_runs : -1.0, a.lock_time_max,
        a.false_locks, a.id_switches, a.reacquires, a.wrong_scans,
        a.coverage_sum / n, a.coverage_min, a.err_rms_sum / n, a.err_p95_max,
        a.vel_err_sum / n, a.lag_sum / n, a.occluded_scans, a.lost_scans,
        a.ambiguous_scans, a.false_person_sum / n, a.proc_mean_sum / n,
        a.proc_p99_max, a.proc_max);
  }
  return 0;
}
