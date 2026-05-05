#include "algorithm/modules/gpu_cascade_hash/gpu_cascade_hash.h"

#include <cuda_runtime.h>
#include <glog/logging.h>

#include <algorithm>
#include <exception>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "algorithm/modules/cpu_cascade_hash/cpu_cascade_hash.h"
#include "algorithm/modules/gpu_cascade_hash/cuda_cascade_hash_kernel.h"
#include "algorithm/modules/matching/match_postprocess.h"

namespace insight {
namespace algorithm {
namespace gpu_cascade_hash {

struct GpuCascadeHashBlockMatcher::Impl {
  explicit Impl(const GpuCascadeHashOptions& opt) : options(opt) {
    if (options.mean_descriptor.size() != 128) {
      options.mean_descriptor.assign(128, 0.0f);
    }
    cudaSetDevice(options.cuda_device_id);
    model = cpu_cascade_hash::build_sample_model_from_mean_descriptor(options.mean_descriptor, hash_options);
  }

  struct StoredImage {
    uint32_t image_index = 0;
    const matching::FeatureData* host_features = nullptr;
    cpu_cascade_hash::ImageFeatures hash_index;
    CudaCascadeImageDeviceView device_view;
  };

  GpuCascadeHashOptions options;
  cpu_cascade_hash::CascadeHashOptions hash_options;
  cpu_cascade_hash::CascadeHashSampleModel model;
  std::vector<StoredImage> images;
  std::unordered_map<uint32_t, int> image_to_local;
  int max_num_features = 0;
  int* d_tmp_out_a = nullptr;
  int* d_tmp_out_b = nullptr;
  uint32_t* d_tmp_pairs_qj = nullptr;  // [max_num_features * 2]
  int* d_tmp_pair_count = nullptr;     // scalar
  cudaStream_t stream_a = nullptr;
  cudaStream_t stream_b = nullptr;
  std::vector<uint32_t> host_pairs_qj;
  std::vector<int> host_forward_best;
};

namespace {
template <typename T>
T* upload_vec_to_device(const std::vector<T>& src) {
  if (src.empty()) return nullptr;
  T* ptr = nullptr;
  cudaMalloc(&ptr, sizeof(T) * src.size());
  cudaMemcpy(ptr, src.data(), sizeof(T) * src.size(), cudaMemcpyHostToDevice);
  return ptr;
}
void free_device_view(CudaCascadeImageDeviceView* v) {
  if (v == nullptr) return;
  if (v->desc_u8) cudaFree(const_cast<uint8_t*>(v->desc_u8));
  if (v->hash) cudaFree(const_cast<uint64_t*>(v->hash));
  if (v->bucket_ids) cudaFree(const_cast<uint16_t*>(v->bucket_ids));
  if (v->bucket_offsets) cudaFree(const_cast<int*>(v->bucket_offsets));
  if (v->bucket_indices) cudaFree(const_cast<int*>(v->bucket_indices));
  *v = CudaCascadeImageDeviceView{};
}
}  // namespace

GpuCascadeHashBlockMatcher::GpuCascadeHashBlockMatcher(const GpuCascadeHashOptions& options)
    : impl_(new Impl(options)) {}

GpuCascadeHashBlockMatcher::~GpuCascadeHashBlockMatcher() {
  for (auto& im : impl_->images) {
    free_device_view(&im.device_view);
  }
  if (impl_->d_tmp_out_a != nullptr) cudaFree(impl_->d_tmp_out_a);
  if (impl_->d_tmp_out_b != nullptr) cudaFree(impl_->d_tmp_out_b);
  if (impl_->d_tmp_pairs_qj != nullptr) cudaFree(impl_->d_tmp_pairs_qj);
  if (impl_->d_tmp_pair_count != nullptr) cudaFree(impl_->d_tmp_pair_count);
  impl_->d_tmp_out_a = nullptr;
  impl_->d_tmp_out_b = nullptr;
  impl_->d_tmp_pairs_qj = nullptr;
  impl_->d_tmp_pair_count = nullptr;
  if (impl_->stream_a != nullptr) cudaStreamDestroy(impl_->stream_a);
  if (impl_->stream_b != nullptr) cudaStreamDestroy(impl_->stream_b);
  impl_->stream_a = nullptr;
  impl_->stream_b = nullptr;
  delete impl_;
  impl_ = nullptr;
}

bool GpuCascadeHashBlockMatcher::add_image(uint32_t image_index, const matching::FeatureData* features) {
  return add_image_with_index(image_index, features, nullptr);
}

bool GpuCascadeHashBlockMatcher::add_image_with_index(
    uint32_t image_index, const matching::FeatureData* features,
    const cpu_cascade_hash::ImageFeatures* hash_index) {
  if (features == nullptr || features->num_features == 0) {
    return false;
  }
  if (impl_->image_to_local.find(image_index) != impl_->image_to_local.end()) {
    return true;
  }

  Impl::StoredImage image;
  image.image_index = image_index;
  image.host_features = features;
  if (hash_index != nullptr) {
    image.hash_index = *hash_index;
  } else {
    image.hash_index = cpu_cascade_hash::compute_image_features(*features, impl_->model);
  }
  if (features->descriptor_type == matching::DescriptorType::kUInt8) {
    std::vector<uint64_t> hash_flat;
    hash_flat.reserve(image.hash_index.compressed_hashes.size() * 2);
    for (const auto& h : image.hash_index.compressed_hashes) {
      hash_flat.push_back(h[0]);
      hash_flat.push_back(h[1]);
    }
    image.device_view.desc_u8 = upload_vec_to_device(features->descriptors_uint8);
    image.device_view.hash = upload_vec_to_device(hash_flat);
    image.device_view.bucket_ids = upload_vec_to_device(image.hash_index.bucket_ids_flat);
    image.device_view.bucket_offsets = upload_vec_to_device(image.hash_index.bucket_offsets);
    image.device_view.bucket_indices = upload_vec_to_device(image.hash_index.bucket_indices);
    image.device_view.num_features = static_cast<int>(features->num_features);
    impl_->max_num_features = std::max(impl_->max_num_features, image.device_view.num_features);
  }

  impl_->image_to_local.emplace(image_index, static_cast<int>(impl_->images.size()));
  impl_->images.push_back(std::move(image));
  return true;
}

bool GpuCascadeHashBlockMatcher::finalize() {
  if (impl_->max_num_features > 0) {
    if (impl_->d_tmp_out_a == nullptr) {
      cudaMalloc(&impl_->d_tmp_out_a, sizeof(int) * static_cast<size_t>(impl_->max_num_features));
    }
    if (impl_->d_tmp_out_b == nullptr) {
      cudaMalloc(&impl_->d_tmp_out_b, sizeof(int) * static_cast<size_t>(impl_->max_num_features));
    }
    if (impl_->d_tmp_pairs_qj == nullptr) {
      cudaMalloc(&impl_->d_tmp_pairs_qj,
                 sizeof(uint32_t) * static_cast<size_t>(impl_->max_num_features) * 2);
    }
    if (impl_->d_tmp_pair_count == nullptr) {
      cudaMalloc(&impl_->d_tmp_pair_count, sizeof(int));
    }
    if (impl_->stream_a == nullptr) cudaStreamCreate(&impl_->stream_a);
    if (impl_->stream_b == nullptr) cudaStreamCreate(&impl_->stream_b);
    if (impl_->host_pairs_qj.capacity() < static_cast<size_t>(impl_->max_num_features) * 2) {
      impl_->host_pairs_qj.reserve(static_cast<size_t>(impl_->max_num_features) * 2);
    }
    if (impl_->host_forward_best.capacity() < static_cast<size_t>(impl_->max_num_features)) {
      impl_->host_forward_best.reserve(static_cast<size_t>(impl_->max_num_features));
    }
  }
  return true;
}

matching::MatchResult GpuCascadeHashBlockMatcher::match_pair(uint32_t image1_index, uint32_t image2_index) {
  auto results = match_pairs({{image1_index, image2_index}});
  if (!results.empty()) return std::move(results[0]);
  return {};
}

std::vector<matching::MatchResult> GpuCascadeHashBlockMatcher::match_pairs(
    const std::vector<std::pair<uint32_t, uint32_t>>& image_pairs) {
  const int num_pairs = static_cast<int>(image_pairs.size());
  std::vector<matching::MatchResult> results(static_cast<size_t>(num_pairs));
  if (num_pairs == 0) return results;

  finalize();
  if (impl_->stream_a == nullptr) cudaStreamCreate(&impl_->stream_a);
  if (impl_->stream_b == nullptr) cudaStreamCreate(&impl_->stream_b);

  // Build per-pair view arrays.
  // Forward: query=image1, train=image2.
  // Backward: query=image2, train=image1 (views swapped, same arrays reused).
  std::vector<CudaCascadeImageDeviceView> h_fwd_q(static_cast<size_t>(num_pairs));
  std::vector<CudaCascadeImageDeviceView> h_fwd_t(static_cast<size_t>(num_pairs));
  std::vector<int> h_fwd_off(static_cast<size_t>(num_pairs + 1), 0);
  std::vector<int> h_bwd_off(static_cast<size_t>(num_pairs + 1), 0);
  std::vector<bool> is_gpu(static_cast<size_t>(num_pairs), false);

  for (int k = 0; k < num_pairs; ++k) {
    if (image_pairs[static_cast<size_t>(k)].first == image_pairs[static_cast<size_t>(k)].second) {
      h_fwd_off[k + 1] = h_fwd_off[k];
      h_bwd_off[k + 1] = h_bwd_off[k];
      continue;
    }
    const auto it1 = impl_->image_to_local.find(image_pairs[static_cast<size_t>(k)].first);
    const auto it2 = impl_->image_to_local.find(image_pairs[static_cast<size_t>(k)].second);
    if (it1 != impl_->image_to_local.end() && it2 != impl_->image_to_local.end()) {
      const auto& im1 = impl_->images[static_cast<size_t>(it1->second)];
      const auto& im2 = impl_->images[static_cast<size_t>(it2->second)];
      if (im1.host_features->descriptor_type == matching::DescriptorType::kUInt8 &&
          im2.host_features->descriptor_type == matching::DescriptorType::kUInt8 &&
          im1.device_view.desc_u8 != nullptr && im2.device_view.desc_u8 != nullptr) {
        h_fwd_q[static_cast<size_t>(k)] = im1.device_view;
        h_fwd_t[static_cast<size_t>(k)] = im2.device_view;
        is_gpu[static_cast<size_t>(k)] = true;
      }
    }
    h_fwd_off[k + 1] = h_fwd_off[k] + h_fwd_q[static_cast<size_t>(k)].num_features;
    h_bwd_off[k + 1] = h_bwd_off[k] + h_fwd_t[static_cast<size_t>(k)].num_features;
  }

  const int total_fwd = h_fwd_off[num_pairs];
  const int total_bwd = h_bwd_off[num_pairs];
  if (total_fwd == 0 && total_bwd == 0) return results;

  // Upload view arrays and offsets (tiny: ~52*num_pairs bytes).
  CudaCascadeImageDeviceView* d_view_q = nullptr;
  CudaCascadeImageDeviceView* d_view_t = nullptr;
  int* d_fwd_off = nullptr;
  int* d_bwd_off = nullptr;
  int* d_out_fwd = nullptr;
  int* d_out_bwd = nullptr;
  cudaMalloc(&d_view_q, sizeof(CudaCascadeImageDeviceView) * static_cast<size_t>(num_pairs));
  cudaMalloc(&d_view_t, sizeof(CudaCascadeImageDeviceView) * static_cast<size_t>(num_pairs));
  cudaMalloc(&d_fwd_off, sizeof(int) * static_cast<size_t>(num_pairs + 1));
  cudaMalloc(&d_bwd_off, sizeof(int) * static_cast<size_t>(num_pairs + 1));
  if (total_fwd > 0) cudaMalloc(&d_out_fwd, sizeof(int) * static_cast<size_t>(total_fwd));
  if (total_bwd > 0) cudaMalloc(&d_out_bwd, sizeof(int) * static_cast<size_t>(total_bwd));

  cudaMemcpy(d_view_q, h_fwd_q.data(), sizeof(CudaCascadeImageDeviceView) * static_cast<size_t>(num_pairs),
             cudaMemcpyHostToDevice);
  cudaMemcpy(d_view_t, h_fwd_t.data(), sizeof(CudaCascadeImageDeviceView) * static_cast<size_t>(num_pairs),
             cudaMemcpyHostToDevice);
  cudaMemcpy(d_fwd_off, h_fwd_off.data(), sizeof(int) * static_cast<size_t>(num_pairs + 1), cudaMemcpyHostToDevice);
  cudaMemcpy(d_bwd_off, h_bwd_off.data(), sizeof(int) * static_cast<size_t>(num_pairs + 1), cudaMemcpyHostToDevice);

  const auto& opt = impl_->model.options;
  const float ratio_sq = opt.ratio_test * opt.ratio_test;

  // Launch forward (stream_a) and backward (stream_b) concurrently.
  if (total_fwd > 0) {
    launch_match_batch_u8_device(d_view_q, d_view_t, d_fwd_off, num_pairs, total_fwd,
                                 opt.bucket_groups, opt.bucket_bits, opt.candidate_top_max,
                                 ratio_sq, d_out_fwd, impl_->stream_a);
  }
  if (total_bwd > 0) {
    // Backward: swap query and train views; use bwd_off for query offsets.
    launch_match_batch_u8_device(d_view_t, d_view_q, d_bwd_off, num_pairs, total_bwd,
                                 opt.bucket_groups, opt.bucket_bits, opt.candidate_top_max,
                                 ratio_sq, d_out_bwd, impl_->stream_b);
  }

  // Two syncs instead of 2*num_pairs syncs.
  if (total_fwd > 0) cudaStreamSynchronize(impl_->stream_a);
  if (total_bwd > 0) cudaStreamSynchronize(impl_->stream_b);

  // Download results to host.
  std::vector<int> h_out_fwd(static_cast<size_t>(total_fwd), -1);
  std::vector<int> h_out_bwd(static_cast<size_t>(total_bwd), -1);
  if (total_fwd > 0)
    cudaMemcpy(h_out_fwd.data(), d_out_fwd, sizeof(int) * static_cast<size_t>(total_fwd), cudaMemcpyDeviceToHost);
  if (total_bwd > 0)
    cudaMemcpy(h_out_bwd.data(), d_out_bwd, sizeof(int) * static_cast<size_t>(total_bwd), cudaMemcpyDeviceToHost);

  cudaFree(d_view_q);   cudaFree(d_view_t);
  cudaFree(d_fwd_off);  cudaFree(d_bwd_off);
  if (d_out_fwd) cudaFree(d_out_fwd);
  if (d_out_bwd) cudaFree(d_out_bwd);

  // CPU-side mutual check and result assembly (O(total_fwd), negligible cost).
  for (int k = 0; k < num_pairs; ++k) {
    if (!is_gpu[static_cast<size_t>(k)]) continue;
    const auto it1 = impl_->image_to_local.find(image_pairs[static_cast<size_t>(k)].first);
    const auto it2 = impl_->image_to_local.find(image_pairs[static_cast<size_t>(k)].second);
    if (it1 == impl_->image_to_local.end() || it2 == impl_->image_to_local.end()) continue;
    const auto& im1 = impl_->images[static_cast<size_t>(it1->second)];
    const auto& im2 = impl_->images[static_cast<size_t>(it2->second)];
    const int fwd_n = h_fwd_q[static_cast<size_t>(k)].num_features;
    const int bwd_n = h_fwd_t[static_cast<size_t>(k)].num_features;
    const int* fwd = h_out_fwd.data() + h_fwd_off[k];
    const int* bwd = h_out_bwd.data() + h_bwd_off[k];

    auto& r = results[static_cast<size_t>(k)];
    for (int q = 0; q < fwd_n; ++q) {
      const int j = fwd[q];
      if (j < 0 || j >= bwd_n) continue;
      if (!opt.mutual_best || bwd[j] == q) {
        matching::append_match(&r, *im1.host_features, *im2.host_features,
                               static_cast<uint16_t>(q), static_cast<uint16_t>(j), false);
      }
    }
    matching::deduplicate_match_indices(&r);
    r.num_matches = r.indices.size();
  }

  return results;
}

}  // namespace gpu_cascade_hash
}  // namespace algorithm
}  // namespace insight

