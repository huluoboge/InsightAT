#pragma once

#include "algorithm/modules/cpu_cascade_hash/cpu_cascade_hash.h"

#include <cuda_runtime.h>
#include <cstdint>
#include <vector>

namespace insight {
namespace algorithm {
namespace gpu_cascade_hash {

struct CudaCascadeImageDeviceView {
  const uint8_t* desc_u8 = nullptr;
  const uint64_t* hash = nullptr;        // [n * 2]
  const uint16_t* bucket_ids = nullptr;  // [n * groups]
  const int* bucket_offsets = nullptr;   // [groups * (bucket_count+1)]
  const int* bucket_indices = nullptr;   // [groups * n]
  int num_features = 0;
};

std::vector<int> match_one_direction_cuda_u8(
    const matching::FeatureData& query_features,
    const cpu_cascade_hash::ImageFeatures& query_index,
    const matching::FeatureData& train_features,
    const cpu_cascade_hash::ImageFeatures& train_index,
    const cpu_cascade_hash::CascadeHashOptions& options);

std::vector<int> match_one_direction_cuda_u8_device(
    const CudaCascadeImageDeviceView& query_dev,
    const CudaCascadeImageDeviceView& train_dev,
    const cpu_cascade_hash::CascadeHashOptions& options);

void match_one_direction_cuda_u8_device_into(
    const CudaCascadeImageDeviceView& query_dev,
    const CudaCascadeImageDeviceView& train_dev,
    const cpu_cascade_hash::CascadeHashOptions& options,
    int* d_out,
    std::vector<int>* host_out);

void launch_match_one_direction_cuda_u8_device(
    const CudaCascadeImageDeviceView& query_dev,
    const CudaCascadeImageDeviceView& train_dev,
    const cpu_cascade_hash::CascadeHashOptions& options,
    int* d_out,
    cudaStream_t stream);

void launch_mutual_compact_u8_device(
    const int* d_forward_best,
    const int* d_backward_best,
    int query_n,
    int train_n,
    uint32_t* d_pairs_qj,
    int* d_pair_count,
    cudaStream_t stream);

// Batch API: processes all pairs in a single kernel launch.
// d_view_q[k] / d_view_t[k]: device-side view pointers for pair k (already on device).
// d_q_offsets[k]: prefix sum of query feature counts across pairs; size = num_pairs+1.
// d_out[total_q]: output best-match index for every global query slot.
// Forward pass: d_view_q as query, d_view_t as train.
// Backward pass: swap d_view_q and d_view_t.
void launch_match_batch_u8_device(
    const CudaCascadeImageDeviceView* d_view_q,
    const CudaCascadeImageDeviceView* d_view_t,
    const int* d_q_offsets,
    int num_pairs,
    int total_q,
    int groups,
    int bucket_bits,
    int candidate_top_max,
    float ratio_sq,
    int* d_out,
    cudaStream_t stream);

}  // namespace gpu_cascade_hash
}  // namespace algorithm
}  // namespace insight

