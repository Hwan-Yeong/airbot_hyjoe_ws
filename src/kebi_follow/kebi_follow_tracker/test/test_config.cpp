// config/leg_tracker.yaml 이 코드 기본값(params.hpp)과 같은지 확인한다
#include <gtest/gtest.h>

#include <cstdlib>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <type_traits>
#include <vector>

#include "kebi_follow_tracker/param_table.hpp"

namespace kt = kebi_follow::tracker;

#ifndef KEBI_TRACKER_CONFIG
#error "KEBI_TRACKER_CONFIG must point to config/leg_tracker.yaml"
#endif

namespace {

std::string trim(const std::string& s) {
  const auto b = s.find_first_not_of(" \t");
  if (b == std::string::npos) {
    return "";
  }
  const auto e = s.find_last_not_of(" \t");
  return s.substr(b, e - b + 1);
}

// ros__parameters 아래 2단계 키만 읽는다: section.key -> 값 문자열
std::map<std::string, std::string> readNested(const std::string& path) {
  std::map<std::string, std::string> out;
  std::ifstream in(path);
  std::string line;
  std::string section;
  int section_indent = -1;
  bool in_params = false;
  while (std::getline(in, line)) {
    const auto hash = line.find('#');
    if (hash != std::string::npos) {
      line = line.substr(0, hash);
    }
    if (trim(line).empty()) {
      continue;
    }
    const int indent = static_cast<int>(line.find_first_not_of(' '));
    const std::string body = trim(line);
    const auto colon = body.find(':');
    const std::string key = body.substr(0, colon);
    const std::string value = trim(body.substr(colon + 1));
    if (key == "ros__parameters") {
      in_params = true;
      continue;
    }
    if (!in_params) {
      continue;
    }
    if (value.empty()) {
      section = key;
      section_indent = indent;
      continue;
    }
    if (!section.empty() && indent > section_indent) {
      out[section + "." + key] = value;
    } else {
      section.clear();
    }
  }
  return out;
}

}  // namespace

TEST(Config, YamlMatchesCodeDefaults) {
  const auto yaml = readNested(KEBI_TRACKER_CONFIG);
  ASSERT_FALSE(yaml.empty());
  kt::TrackerParams defaults;
  std::set<std::string> names;
  kt::visitParams(defaults, [&](const char* name, auto& field) {
    names.insert(name);
    const auto it = yaml.find(name);
    ASSERT_NE(it, yaml.end()) << "missing in yaml: " << name;
    const std::string& v = it->second;
    using T = std::decay_t<decltype(field)>;
    if constexpr (std::is_same_v<T, bool>) {
      EXPECT_EQ(v == "true", field) << name;
    } else if constexpr (std::is_same_v<T, int>) {
      EXPECT_EQ(v.find('.'), std::string::npos)
          << name << " must be an integer";
      EXPECT_EQ(std::atoi(v.c_str()), field) << name;
    } else {
      // 실수형은 소수점이 있어야 ROS 가 double 로 읽는다
      EXPECT_NE(v.find_first_of(".eE"), std::string::npos)
          << name << " needs a decimal point";
      EXPECT_DOUBLE_EQ(std::atof(v.c_str()), field) << name;
    }
  });
  for (const auto& kv : yaml) {
    EXPECT_TRUE(names.count(kv.first)) << "unknown key in yaml: " << kv.first;
  }
}

TEST(Config, SpecValues) {
  // SPEC 6.2 의 숫자 (바꾸면 SPEC 과 같이 고친다)
  const kt::TrackerParams p;
  EXPECT_DOUBLE_EQ(p.leg.min_width, 0.05);
  EXPECT_DOUBLE_EQ(p.leg.max_width, 0.25);
  EXPECT_DOUBLE_EQ(p.leg.pair_min_sep, 0.10);
  EXPECT_DOUBLE_EQ(p.leg.pair_max_sep, 0.60);
  EXPECT_DOUBLE_EQ(p.camera.gate, 0.4);
  EXPECT_DOUBLE_EQ(p.acquire.min_range, 0.5);
  EXPECT_DOUBLE_EQ(p.acquire.max_range, 1.5);
  EXPECT_DOUBLE_EQ(p.acquire.max_bearing_deg, 20.0);
  EXPECT_DOUBLE_EQ(p.acquire.confirm_s, 1.0);
  EXPECT_TRUE(p.acquire.require_camera);
  EXPECT_DOUBLE_EQ(p.target.lost_s, 2.0);
  EXPECT_DOUBLE_EQ(p.target.max_std, 0.5);
  EXPECT_DOUBLE_EQ(p.target.reacquire_radius, 1.0);
  EXPECT_DOUBLE_EQ(p.target.reacquire_confirm_s, 0.5);
  EXPECT_DOUBLE_EQ(p.ambiguity.radius, 0.6);
  EXPECT_DOUBLE_EQ(p.scan.robot_radius, 0.22);
}
