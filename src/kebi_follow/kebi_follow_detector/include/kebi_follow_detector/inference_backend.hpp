// Inference backends behind one interface: RKNN (RK3588 NPU, aarch64) and OpenCV DNN (CPU, PC tests).
// Both return the raw output tensors of the NPU-friendly YOLO head (see yolo_decode.hpp) in canonical
// order, so the same decode_yolo() serves both.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "kebi_follow_detector/yolo_decode.hpp"

namespace kebi_follow
{

struct BackendConfig
{
  std::string backend{"auto"};      // auto | rknn | opencv
  std::string model_path;           // .rknn for rknn, .onnx for opencv
  std::string model_type{"pose"};   // pose | det
  std::string rknn_core_mask{"auto"};  // auto | 0 | 1 | 2 | 01 | 012 | all
  int input_width{640};             // opencv backend only (rknn reads it from the model)
  int input_height{480};
  int cpu_threads{0};               // opencv backend: 0 = OpenCV default
};

struct BackendInfo
{
  std::string name;
  std::string detail;  // runtime / driver versions etc.
  int input_width{0};
  int input_height{0};
  int outputs_per_level{0};
};

class InferenceBackend
{
public:
  virtual ~InferenceBackend() = default;

  // rgb: input_height x input_width x 3, uint8 RGB, contiguous rows.
  // On success `outputs` holds views into backend buffers, valid until the next infer() call.
  virtual bool infer(const uint8_t * rgb, std::vector<TensorView> & outputs, std::string & error) = 0;

  const BackendInfo & info() const {return info_;}

protected:
  BackendInfo info_;
};

bool rknn_backend_available();
bool opencv_backend_available();

// Returns nullptr and sets `error` on failure.
std::unique_ptr<InferenceBackend> create_backend(const BackendConfig & config, std::string & error);

// Canonical output names of our exporter, in decode order.
std::vector<std::string> canonical_output_names(int outputs_per_level);

}  // namespace kebi_follow
