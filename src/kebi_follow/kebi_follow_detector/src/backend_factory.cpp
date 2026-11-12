#include <memory>
#include <string>
#include <vector>

#include "kebi_follow_detector/inference_backend.hpp"

namespace kebi_follow
{

#ifdef KEBI_HAVE_RKNN
std::unique_ptr<InferenceBackend> make_rknn_backend(const BackendConfig & config, std::string & error);
#endif
#ifdef KEBI_HAVE_OPENCV_DNN
std::unique_ptr<InferenceBackend> make_opencv_backend(
  const BackendConfig & config,
  std::string & error);
#endif

bool rknn_backend_available()
{
#ifdef KEBI_HAVE_RKNN
  return true;
#else
  return false;
#endif
}

bool opencv_backend_available()
{
#ifdef KEBI_HAVE_OPENCV_DNN
  return true;
#else
  return false;
#endif
}

std::vector<std::string> canonical_output_names(int outputs_per_level)
{
  std::vector<std::string> names;
  for (const int s : {8, 16, 32}) {
    names.push_back("box_s" + std::to_string(s));
    names.push_back("score_s" + std::to_string(s));
    if (outputs_per_level == 3) {
      names.push_back("kpt_s" + std::to_string(s));
    }
  }
  return names;
}

std::unique_ptr<InferenceBackend> create_backend(const BackendConfig & config, std::string & error)
{
  if (config.model_type != "pose" && config.model_type != "det") {
    error = "model_type must be 'pose' or 'det', got '" + config.model_type + "'";
    return nullptr;
  }
  if (config.model_path.empty()) {
    error = "model_path is empty (models are not part of the repository; set the parameter)";
    return nullptr;
  }
  std::string backend = config.backend;
  if (backend == "auto") {
    backend = rknn_backend_available() ? "rknn" : "opencv";
  }
  if (backend == "rknn") {
#ifdef KEBI_HAVE_RKNN
    return make_rknn_backend(config, error);
#else
    error = "built without the RKNN backend (needs aarch64 and -DRKNN_RT_DIR)";
    return nullptr;
#endif
  }
  if (backend == "opencv") {
#ifdef KEBI_HAVE_OPENCV_DNN
    return make_opencv_backend(config, error);
#else
    error = "built without the OpenCV DNN backend";
    return nullptr;
#endif
  }
  error = "unknown backend '" + config.backend + "' (auto | rknn | opencv)";
  return nullptr;
}

}  // namespace kebi_follow
