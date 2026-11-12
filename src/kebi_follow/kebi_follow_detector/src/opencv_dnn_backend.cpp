// CPU backend for PC tests: OpenCV DNN (4.5.4) with the exported ONNX model.
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

#include "kebi_follow_detector/inference_backend.hpp"

namespace kebi_follow
{

namespace
{

class OpencvDnnBackend final : public InferenceBackend
{
public:
  bool init(const BackendConfig & config, std::string & error)
  {
    const int per_level = config.model_type == "pose" ? 3 : 2;
    try {
      net_ = cv::dnn::readNetFromONNX(config.model_path);
    } catch (const cv::Exception & e) {
      error = std::string("readNetFromONNX failed: ") + e.what();
      return false;
    }
    if (net_.empty()) {
      error = "empty network: " + config.model_path;
      return false;
    }
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    if (config.cpu_threads > 0) {
      cv::setNumThreads(config.cpu_threads);
    }
    names_ = canonical_output_names(per_level);
    const std::vector<std::string> available = net_.getUnconnectedOutLayersNames();
    for (const auto & n : names_) {
      bool found = false;
      for (const auto & a : available) {
        found = found || a == n;
      }
      if (!found) {
        error = "model has no output '" + n + "' (model_type " + config.model_type +
          ", exporter mismatch?)";
        return false;
      }
    }
    info_.name = "opencv";
    info_.detail = std::string("OpenCV ") + CV_VERSION + " DNN (CPU)";
    info_.input_width = config.input_width;
    info_.input_height = config.input_height;
    info_.outputs_per_level = per_level;
    return true;
  }

  bool infer(const uint8_t * rgb, std::vector<TensorView> & outputs, std::string & error) override
  {
    try {
      const cv::Mat image(info_.input_height, info_.input_width, CV_8UC3,
        const_cast<uint8_t *>(rgb));
      const cv::Mat blob =
        cv::dnn::blobFromImage(image, 1.0 / 255.0, cv::Size(), cv::Scalar(), false, false, CV_32F);
      net_.setInput(blob);
      net_.forward(mats_, names_);
    } catch (const cv::Exception & e) {
      error = std::string("OpenCV DNN forward failed: ") + e.what();
      return false;
    }
    outputs.clear();
    for (const cv::Mat & m : mats_) {
      if (m.dims != 4 || m.type() != CV_32F || !m.isContinuous()) {
        error = "unexpected output tensor shape/type";
        return false;
      }
      TensorView v;
      v.data = m.ptr<float>();
      v.type = TensorType::kFloat32;
      v.channels = m.size[1];
      v.height = m.size[2];
      v.width = m.size[3];
      outputs.push_back(v);
    }
    return true;
  }

private:
  cv::dnn::Net net_;
  std::vector<std::string> names_;
  std::vector<cv::Mat> mats_;
};

}  // namespace

std::unique_ptr<InferenceBackend> make_opencv_backend(const BackendConfig & config, std::string & error)
{
  auto backend = std::make_unique<OpencvDnnBackend>();
  if (!backend->init(config, error)) {
    return nullptr;
  }
  return backend;
}

}  // namespace kebi_follow
