// RK3588 NPU backend (librknnrt). Normal API: rknn_inputs_set (uint8 RGB NHWC, normalization inside the
// model) -> rknn_run -> rknn_outputs_get into preallocated buffers. INT8 outputs stay INT8
// (want_float = 0) and are dequantized only where decode_yolo() reads them.
#include <rknn_api.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "kebi_follow_detector/inference_backend.hpp"

namespace kebi_follow
{

namespace
{

constexpr int kInputChannels = 3;
constexpr int kOutputDims = 4;

rknn_core_mask parse_core_mask(const std::string & s)
{
  if (s == "0") {
    return RKNN_NPU_CORE_0;
  }
  if (s == "1") {
    return RKNN_NPU_CORE_1;
  }
  if (s == "2") {
    return RKNN_NPU_CORE_2;
  }
  if (s == "01") {
    return RKNN_NPU_CORE_0_1;
  }
  if (s == "012") {
    return RKNN_NPU_CORE_0_1_2;
  }
  if (s == "all") {
    return RKNN_NPU_CORE_ALL;
  }
  return RKNN_NPU_CORE_AUTO;
}

class RknnBackend final : public InferenceBackend
{
public:
  ~RknnBackend() override
  {
    if (ctx_ != 0) {
      rknn_destroy(ctx_);
    }
  }

  bool init(const BackendConfig & config, std::string & error)
  {
    int ret = rknn_init(&ctx_, const_cast<char *>(config.model_path.c_str()), 0, 0, nullptr);
    if (ret != RKNN_SUCC) {
      ctx_ = 0;
      error = "rknn_init failed (" + std::to_string(ret) + "): " + config.model_path;
      return false;
    }
    rknn_sdk_version ver{};
    if (rknn_query(ctx_, RKNN_QUERY_SDK_VERSION, &ver, sizeof(ver)) == RKNN_SUCC) {
      info_.detail = std::string("librknnrt ") + ver.api_version + ", driver " + ver.drv_version;
    }
    ret = rknn_set_core_mask(ctx_, parse_core_mask(config.rknn_core_mask));
    if (ret != RKNN_SUCC) {
      error = "rknn_set_core_mask(" + config.rknn_core_mask + ") failed (" + std::to_string(ret) + ")";
      return false;
    }
    rknn_input_output_num io{};
    if (rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &io, sizeof(io)) != RKNN_SUCC || io.n_input != 1) {
      error = "model must have exactly one input";
      return false;
    }
    rknn_tensor_attr in{};
    in.index = 0;
    if (rknn_query(ctx_, RKNN_QUERY_INPUT_ATTR, &in, sizeof(in)) != RKNN_SUCC || in.n_dims != 4) {
      error = "cannot query the input tensor";
      return false;
    }
    if (in.fmt == RKNN_TENSOR_NHWC) {
      info_.input_height = static_cast<int>(in.dims[1]);
      info_.input_width = static_cast<int>(in.dims[2]);
    } else {
      info_.input_height = static_cast<int>(in.dims[2]);
      info_.input_width = static_cast<int>(in.dims[3]);
    }
    input_bytes_ = static_cast<size_t>(info_.input_width) * info_.input_height * kInputChannels;

    const int per_level = static_cast<int>(io.n_output) / 3;
    const int wanted = config.model_type == "pose" ? 3 : 2;
    if (per_level != wanted || static_cast<int>(io.n_output) != 3 * per_level) {
      error = "model has " + std::to_string(io.n_output) + " outputs, model_type '" +
        config.model_type + "' needs " + std::to_string(3 * wanted);
      return false;
    }
    info_.outputs_per_level = per_level;

    // map rknn output index -> canonical position by tensor name (falls back to model order)
    std::vector<rknn_tensor_attr> attrs(io.n_output);
    std::map<std::string, int> by_name;
    for (uint32_t k = 0; k < io.n_output; ++k) {
      attrs[k].index = k;
      if (rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &attrs[k], sizeof(rknn_tensor_attr)) != RKNN_SUCC ||
        attrs[k].n_dims != kOutputDims)
      {
        error = "cannot query output " + std::to_string(k);
        return false;
      }
      by_name[attrs[k].name] = static_cast<int>(k);
    }
    const std::vector<std::string> names = canonical_output_names(per_level);
    order_.assign(names.size(), -1);
    bool all_named = true;
    for (size_t c = 0; c < names.size(); ++c) {
      const auto it = by_name.find(names[c]);
      if (it == by_name.end()) {
        all_named = false;
        break;
      }
      order_[c] = it->second;
    }
    if (!all_named) {
      for (size_t c = 0; c < names.size(); ++c) {
        order_[c] = static_cast<int>(c);
      }
    }

    outputs_.assign(io.n_output, rknn_output{});
    buffers_.resize(io.n_output);
    views_.resize(io.n_output);
    for (uint32_t c = 0; c < io.n_output; ++c) {
      const rknn_tensor_attr & a = attrs[order_[c]];
      const bool quantized = a.type == RKNN_TENSOR_INT8 || a.type == RKNN_TENSOR_UINT8;
      buffers_[c].assign(static_cast<size_t>(a.n_elems) * (quantized ? 1 : sizeof(float)), 0);
      rknn_output & o = outputs_[order_[c]];
      o.index = static_cast<uint32_t>(order_[c]);
      o.want_float = quantized ? 0 : 1;
      o.is_prealloc = 1;
      o.buf = buffers_[c].data();
      o.size = static_cast<uint32_t>(buffers_[c].size());
      TensorView & v = views_[c];
      v.data = buffers_[c].data();
      v.type = a.type == RKNN_TENSOR_INT8 ? TensorType::kInt8 :
        (a.type == RKNN_TENSOR_UINT8 ? TensorType::kUint8 : TensorType::kFloat32);
      v.zero_point = quantized ? a.zp : 0;
      v.scale = quantized ? a.scale : 1.0f;
      v.channels = static_cast<int>(a.dims[1]);
      v.height = static_cast<int>(a.dims[2]);
      v.width = static_cast<int>(a.dims[3]);
    }
    info_.name = "rknn";
    return true;
  }

  bool infer(const uint8_t * rgb, std::vector<TensorView> & outputs, std::string & error) override
  {
    rknn_input in{};
    in.index = 0;
    in.buf = const_cast<uint8_t *>(rgb);
    in.size = static_cast<uint32_t>(input_bytes_);
    in.pass_through = 0;
    in.type = RKNN_TENSOR_UINT8;
    in.fmt = RKNN_TENSOR_NHWC;
    int ret = rknn_inputs_set(ctx_, 1, &in);
    if (ret == RKNN_SUCC) {
      ret = rknn_run(ctx_, nullptr);
    }
    if (ret == RKNN_SUCC) {
      for (size_t c = 0; c < buffers_.size(); ++c) {  // the runtime may touch the descriptors
        rknn_output & o = outputs_[order_[c]];
        o.is_prealloc = 1;
        o.buf = buffers_[c].data();
        o.size = static_cast<uint32_t>(buffers_[c].size());
      }
      ret = rknn_outputs_get(ctx_, static_cast<uint32_t>(outputs_.size()), outputs_.data(), nullptr);
      if (ret == RKNN_SUCC) {
        rknn_outputs_release(ctx_, static_cast<uint32_t>(outputs_.size()), outputs_.data());
      }
    }
    if (ret != RKNN_SUCC) {
      error = "rknn inference failed (" + std::to_string(ret) + ")";
      return false;
    }
    outputs = views_;
    return true;
  }

private:
  rknn_context ctx_{0};
  size_t input_bytes_{0};
  std::vector<int> order_;                   // canonical position -> rknn output index
  std::vector<rknn_output> outputs_;         // indexed by rknn output index
  std::vector<std::vector<uint8_t>> buffers_;  // indexed by canonical position
  std::vector<TensorView> views_;            // canonical order
};

}  // namespace

std::unique_ptr<InferenceBackend> make_rknn_backend(const BackendConfig & config, std::string & error)
{
  auto backend = std::make_unique<RknnBackend>();
  if (!backend->init(config, error)) {
    return nullptr;
  }
  return backend;
}

}  // namespace kebi_follow
