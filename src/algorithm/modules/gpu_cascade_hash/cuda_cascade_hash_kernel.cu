#include "algorithm/modules/gpu_cascade_hash/cuda_cascade_hash_kernel.h"

#include <cuda_runtime.h>

#include <stdexcept>

namespace insight {
namespace algorithm {
namespace gpu_cascade_hash {
namespace {

constexpr int kDescriptorDim = 128;
constexpr int kCompressedWords = 2;
// Slightly above candidate_top_max default (10) for safe headroom.
// Keeping this tight is critical for GPU occupancy: each extra slot costs
// 2 registers per thread (cand_idx + cand_ham) and reduces active warps
// per SM, directly hurting memory-latency hiding on Pascal (GTX 1060).
constexpr int kMaxTopCandidates = 12;

__device__ inline int popcnt64(unsigned long long x) {
  return __popcll(x);
}

__device__ inline float l2_u8_128(const uint8_t* a, const uint8_t* b) {
  float sum = 0.0f;
#pragma unroll
  for (int i = 0; i < kDescriptorDim; ++i) {
    const float d = static_cast<float>(a[i]) - static_cast<float>(b[i]);
    sum += d * d;
  }
  return sum;
}

__launch_bounds__(128, 8)
__global__ void match_direction_kernel_u8_generic(
    const uint8_t* query_desc,
    const uint64_t* query_hash,     // [qn * 2]
    const uint16_t* query_bucket,   // [qn * groups]
    int query_n,
    const uint8_t* train_desc,
    const uint64_t* train_hash,     // [tn * 2]
    const int* train_offsets,       // [groups * (bucket_count+1)]
    const int* train_indices,       // [groups * tn]
    int train_n,
    int groups,
    int bucket_bits,
    int candidate_top_max,
    float ratio_sq,
    int* out_best_idx) {
  const int q = blockIdx.x * blockDim.x + threadIdx.x;
  if (q >= query_n) return;

  const int bucket_count = 1 << bucket_bits;
  const uint64_t qh0 = query_hash[static_cast<size_t>(q) * 2];
  const uint64_t qh1 = query_hash[static_cast<size_t>(q) * 2 + 1];

  int cand_idx[kMaxTopCandidates];
  int cand_ham[kMaxTopCandidates];
  int cand_count = 0;
  int worst_pos = -1;
  int worst_ham = -1;
  const int keep_k = candidate_top_max < kMaxTopCandidates ? candidate_top_max : kMaxTopCandidates;

  for (int g = 0; g < groups; ++g) {
    const uint16_t bucket_id = query_bucket[static_cast<size_t>(q) * groups + g];
    const int off_base = g * (bucket_count + 1);
    const int begin = train_offsets[off_base + static_cast<int>(bucket_id)];
    const int end = train_offsets[off_base + static_cast<int>(bucket_id) + 1];
    const int group_base = g * train_n;
    for (int p = begin; p < end; ++p) {
      const int idx = train_indices[group_base + p];
      bool duplicated = false;
      for (int i = 0; i < cand_count; ++i) {
        if (cand_idx[i] == idx) {
          duplicated = true;
          break;
        }
      }
      if (duplicated) continue;

      const uint64_t th0 = train_hash[static_cast<size_t>(idx) * 2];
      const uint64_t th1 = train_hash[static_cast<size_t>(idx) * 2 + 1];
      const int ham = popcnt64(qh0 ^ th0) + popcnt64(qh1 ^ th1);

      if (cand_count < keep_k) {
        cand_idx[cand_count] = idx;
        cand_ham[cand_count] = ham;
        if (ham > worst_ham) {
          worst_ham = ham;
          worst_pos = cand_count;
        }
        ++cand_count;
      } else if (ham < worst_ham) {
        cand_idx[worst_pos] = idx;
        cand_ham[worst_pos] = ham;
        worst_ham = cand_ham[0];
        worst_pos = 0;
        for (int i = 1; i < cand_count; ++i) {
          if (cand_ham[i] > worst_ham) {
            worst_ham = cand_ham[i];
            worst_pos = i;
          }
        }
      }
    }
  }

  if (cand_count < 2) {
    out_best_idx[q] = -1;
    return;
  }

  const uint8_t* qd = query_desc + static_cast<size_t>(q) * kDescriptorDim;
  float best = 1e30f, second = 1e30f;
  int best_idx = -1;
  for (int i = 0; i < cand_count; ++i) {
    const int idx = cand_idx[i];
    const uint8_t* td = train_desc + static_cast<size_t>(idx) * kDescriptorDim;
    const float dist = l2_u8_128(qd, td);
    if (dist < best) {
      second = best;
      best = dist;
      best_idx = idx;
    } else if (dist < second) {
      second = dist;
    }
  }
  out_best_idx[q] = (best_idx >= 0 && second < 1e29f && best < ratio_sq * second) ? best_idx : -1;
}

template <int GROUPS, int BUCKET_BITS, int KEEP_K>
__launch_bounds__(128, 8)
__global__ void match_direction_kernel_u8_templ(
    const uint8_t* query_desc,
    const uint64_t* query_hash,
    const uint16_t* query_bucket,
    int query_n,
    const uint8_t* train_desc,
    const uint64_t* train_hash,
    const int* train_offsets,
    const int* train_indices,
    int train_n,
    float ratio_sq,
    int* out_best_idx) {
  const int q = blockIdx.x * blockDim.x + threadIdx.x;
  if (q >= query_n) return;

  constexpr int bucket_count = 1 << BUCKET_BITS;
  const uint64_t qh0 = query_hash[static_cast<size_t>(q) * 2];
  const uint64_t qh1 = query_hash[static_cast<size_t>(q) * 2 + 1];

  // Use KEEP_K (compile-time constant) instead of kMaxTopCandidates to
  // minimize register footprint and maximize warp occupancy per SM.
  int cand_idx[KEEP_K];
  int cand_ham[KEEP_K];
  int cand_count = 0;
  int worst_pos = -1;
  int worst_ham = -1;

#pragma unroll
  for (int g = 0; g < GROUPS; ++g) {
    const uint16_t bucket_id = query_bucket[static_cast<size_t>(q) * GROUPS + g];
    const int off_base = g * (bucket_count + 1);
    const int begin = train_offsets[off_base + static_cast<int>(bucket_id)];
    const int end = train_offsets[off_base + static_cast<int>(bucket_id) + 1];
    const int group_base = g * train_n;
    for (int p = begin; p < end; ++p) {
      const int idx = train_indices[group_base + p];
      bool duplicated = false;
      for (int i = 0; i < cand_count; ++i) {
        if (cand_idx[i] == idx) {
          duplicated = true;
          break;
        }
      }
      if (duplicated) continue;

      const uint64_t th0 = train_hash[static_cast<size_t>(idx) * 2];
      const uint64_t th1 = train_hash[static_cast<size_t>(idx) * 2 + 1];
      const int ham = popcnt64(qh0 ^ th0) + popcnt64(qh1 ^ th1);

      if (cand_count < KEEP_K) {
        cand_idx[cand_count] = idx;
        cand_ham[cand_count] = ham;
        if (ham > worst_ham) {
          worst_ham = ham;
          worst_pos = cand_count;
        }
        ++cand_count;
      } else if (ham < worst_ham) {
        cand_idx[worst_pos] = idx;
        cand_ham[worst_pos] = ham;
        worst_ham = cand_ham[0];
        worst_pos = 0;
        for (int i = 1; i < cand_count; ++i) {
          if (cand_ham[i] > worst_ham) {
            worst_ham = cand_ham[i];
            worst_pos = i;
          }
        }
      }
    }
  }

  if (cand_count < 2) {
    out_best_idx[q] = -1;
    return;
  }

  const uint8_t* qd = query_desc + static_cast<size_t>(q) * kDescriptorDim;
  float best = 1e30f, second = 1e30f;
  int best_idx = -1;
  for (int i = 0; i < cand_count; ++i) {
    const int idx = cand_idx[i];
    const uint8_t* td = train_desc + static_cast<size_t>(idx) * kDescriptorDim;
    const float dist = l2_u8_128(qd, td);
    if (dist < best) {
      second = best;
      best = dist;
      best_idx = idx;
    } else if (dist < second) {
      second = dist;
    }
  }
  out_best_idx[q] = (best_idx >= 0 && second < 1e29f && best < ratio_sq * second) ? best_idx : -1;
}

__global__ void mutual_compact_kernel_u8(const int* forward_best,
                                         const int* backward_best,
                                         int query_n,
                                         int train_n,
                                         uint32_t* out_pairs_qj,
                                         int* out_count) {
  __shared__ uint2 s_pairs[128];
  __shared__ int s_count;
  __shared__ int s_base;
  if (threadIdx.x == 0) {
    s_count = 0;
    s_base = 0;
  }
  __syncthreads();

  const int q = blockIdx.x * blockDim.x + threadIdx.x;
  if (q < query_n) {
    const int j = forward_best[q];
    if (j >= 0 && j < train_n && backward_best[j] == q) {
      const int local_pos = atomicAdd(&s_count, 1);
      if (local_pos < static_cast<int>(blockDim.x)) {
        s_pairs[local_pos] = make_uint2(static_cast<uint32_t>(q), static_cast<uint32_t>(j));
      }
    }
  }
  __syncthreads();

  if (threadIdx.x == 0 && s_count > 0) {
    s_base = atomicAdd(out_count, s_count);
  }
  __syncthreads();

  if (threadIdx.x < s_count) {
    const uint2 p = s_pairs[threadIdx.x];
    const int pos = s_base + static_cast<int>(threadIdx.x);
    out_pairs_qj[static_cast<size_t>(pos) * 2] = p.x;
    out_pairs_qj[static_cast<size_t>(pos) * 2 + 1] = p.y;
  }
}

// Batch kernel: one thread per global query across all pairs.
// view_q[k]/view_t[k] hold device pointers for pair k.
// q_offsets[k] = prefix sum of query feature counts; thread gq belongs to pair k
// where q_offsets[k] <= gq < q_offsets[k+1].
__launch_bounds__(128, 8)
__global__ void match_direction_batch_kernel_u8(
    const CudaCascadeImageDeviceView* view_q,
    const CudaCascadeImageDeviceView* view_t,
    const int* q_offsets,
    int num_pairs,
    int total_q,
    int groups,
    int bucket_bits,
    int candidate_top_max,
    float ratio_sq,
    int* out_best) {
  const int gq = blockIdx.x * blockDim.x + threadIdx.x;
  if (gq >= total_q) return;

  // Binary search: find pair_id such that q_offsets[pair_id] <= gq < q_offsets[pair_id+1]
  int lo = 0, hi = num_pairs - 1;
  while (lo < hi) {
    const int mid = (lo + hi + 1) >> 1;
    if (q_offsets[mid] <= gq) lo = mid;
    else hi = mid - 1;
  }
  const int pair_id = lo;
  const int q = gq - q_offsets[pair_id];

  const CudaCascadeImageDeviceView qv = view_q[pair_id];
  const CudaCascadeImageDeviceView tv = view_t[pair_id];
  if (q >= qv.num_features || tv.num_features == 0 ||
      qv.hash == nullptr || tv.hash == nullptr) {
    out_best[gq] = -1;
    return;
  }

  const int train_n = tv.num_features;
  const int bucket_count = 1 << bucket_bits;
  const uint64_t qh0 = qv.hash[static_cast<size_t>(q) * 2];
  const uint64_t qh1 = qv.hash[static_cast<size_t>(q) * 2 + 1];

  int cand_idx[kMaxTopCandidates];
  int cand_ham[kMaxTopCandidates];
  int cand_count = 0;
  int worst_pos = -1;
  int worst_ham = -1;
  const int keep_k = candidate_top_max < kMaxTopCandidates ? candidate_top_max : kMaxTopCandidates;

  for (int g = 0; g < groups; ++g) {
    const uint16_t bucket_id = qv.bucket_ids[static_cast<size_t>(q) * groups + g];
    const int off_base = g * (bucket_count + 1);
    const int begin = tv.bucket_offsets[off_base + static_cast<int>(bucket_id)];
    const int end   = tv.bucket_offsets[off_base + static_cast<int>(bucket_id) + 1];
    const int group_base = g * train_n;
    for (int p = begin; p < end; ++p) {
      const int idx = tv.bucket_indices[group_base + p];
      bool dup = false;
      for (int i = 0; i < cand_count; ++i) {
        if (cand_idx[i] == idx) { dup = true; break; }
      }
      if (dup) continue;

      const uint64_t th0 = tv.hash[static_cast<size_t>(idx) * 2];
      const uint64_t th1 = tv.hash[static_cast<size_t>(idx) * 2 + 1];
      const int ham = popcnt64(qh0 ^ th0) + popcnt64(qh1 ^ th1);

      if (cand_count < keep_k) {
        cand_idx[cand_count] = idx;
        cand_ham[cand_count] = ham;
        if (ham > worst_ham) { worst_ham = ham; worst_pos = cand_count; }
        ++cand_count;
      } else if (ham < worst_ham) {
        cand_idx[worst_pos] = idx;
        cand_ham[worst_pos] = ham;
        worst_ham = cand_ham[0]; worst_pos = 0;
        for (int i = 1; i < cand_count; ++i) {
          if (cand_ham[i] > worst_ham) { worst_ham = cand_ham[i]; worst_pos = i; }
        }
      }
    }
  }

  if (cand_count < 2) { out_best[gq] = -1; return; }

  const uint8_t* qd = qv.desc_u8 + static_cast<size_t>(q) * kDescriptorDim;
  float best = 1e30f, second = 1e30f;
  int best_idx = -1;
  for (int i = 0; i < cand_count; ++i) {
    const int idx = cand_idx[i];
    const uint8_t* td = tv.desc_u8 + static_cast<size_t>(idx) * kDescriptorDim;
    const float dist = l2_u8_128(qd, td);
    if (dist < best) { second = best; best = dist; best_idx = idx; }
    else if (dist < second) { second = dist; }
  }
  out_best[gq] = (best_idx >= 0 && second < 1e29f && best < ratio_sq * second) ? best_idx : -1;
}

template <typename T>
T* copy_to_device(const std::vector<T>& host) {
  if (host.empty()) return nullptr;
  T* ptr = nullptr;
  cudaMalloc(&ptr, sizeof(T) * host.size());
  cudaMemcpy(ptr, host.data(), sizeof(T) * host.size(), cudaMemcpyHostToDevice);
  return ptr;
}

}  // namespace

std::vector<int> match_one_direction_cuda_u8(
    const matching::FeatureData& query_features,
    const cpu_cascade_hash::ImageFeatures& query_index,
    const matching::FeatureData& train_features,
    const cpu_cascade_hash::ImageFeatures& train_index,
    const cpu_cascade_hash::CascadeHashOptions& options) {
  const int query_n = static_cast<int>(query_features.num_features);
  const int train_n = static_cast<int>(train_features.num_features);
  if (query_n <= 0 || train_n <= 0) return {};

  std::vector<uint64_t> query_hash;
  query_hash.reserve(query_index.compressed_hashes.size() * 2);
  for (const auto& h : query_index.compressed_hashes) {
    query_hash.push_back(h[0]);
    query_hash.push_back(h[1]);
  }
  std::vector<uint64_t> train_hash;
  train_hash.reserve(train_index.compressed_hashes.size() * 2);
  for (const auto& h : train_index.compressed_hashes) {
    train_hash.push_back(h[0]);
    train_hash.push_back(h[1]);
  }

  uint8_t* d_q_desc = copy_to_device(query_features.descriptors_uint8);
  uint8_t* d_t_desc = copy_to_device(train_features.descriptors_uint8);
  uint64_t* d_q_hash = copy_to_device(query_hash);
  uint64_t* d_t_hash = copy_to_device(train_hash);
  uint16_t* d_q_bucket = copy_to_device(query_index.bucket_ids_flat);
  int* d_t_offsets = copy_to_device(train_index.bucket_offsets);
  int* d_t_indices = copy_to_device(train_index.bucket_indices);
  int* d_out = nullptr;
  cudaMalloc(&d_out, sizeof(int) * query_n);

  const int threads = 128;
  const int blocks = (query_n + threads - 1) / threads;
  const float ratio_sq = options.ratio_test * options.ratio_test;
  match_direction_kernel_u8_generic<<<blocks, threads>>>(
      d_q_desc, d_q_hash, d_q_bucket, query_n, d_t_desc, d_t_hash, d_t_offsets, d_t_indices, train_n,
      options.bucket_groups, options.bucket_bits, options.candidate_top_max, ratio_sq, d_out);
  cudaError_t err = cudaDeviceSynchronize();
  if (err != cudaSuccess) {
    cudaFree(d_q_desc); cudaFree(d_t_desc); cudaFree(d_q_hash); cudaFree(d_t_hash);
    cudaFree(d_q_bucket); cudaFree(d_t_offsets); cudaFree(d_t_indices); cudaFree(d_out);
    throw std::runtime_error(cudaGetErrorString(err));
  }

  std::vector<int> out(static_cast<size_t>(query_n), -1);
  cudaMemcpy(out.data(), d_out, sizeof(int) * query_n, cudaMemcpyDeviceToHost);

  cudaFree(d_q_desc); cudaFree(d_t_desc); cudaFree(d_q_hash); cudaFree(d_t_hash);
  cudaFree(d_q_bucket); cudaFree(d_t_offsets); cudaFree(d_t_indices); cudaFree(d_out);
  return out;
}

std::vector<int> match_one_direction_cuda_u8_device(
    const CudaCascadeImageDeviceView& query_dev,
    const CudaCascadeImageDeviceView& train_dev,
    const cpu_cascade_hash::CascadeHashOptions& options) {
  const int query_n = query_dev.num_features;
  const int train_n = train_dev.num_features;
  if (query_n <= 0 || train_n <= 0) return {};

  int* d_out = nullptr;
  cudaMalloc(&d_out, sizeof(int) * query_n);

  const int threads = 128;
  const int blocks = (query_n + threads - 1) / threads;
  const float ratio_sq = options.ratio_test * options.ratio_test;
  match_direction_kernel_u8_generic<<<blocks, threads>>>(
      query_dev.desc_u8, query_dev.hash, query_dev.bucket_ids, query_n,
      train_dev.desc_u8, train_dev.hash, train_dev.bucket_offsets, train_dev.bucket_indices, train_n,
      options.bucket_groups, options.bucket_bits, options.candidate_top_max, ratio_sq, d_out);
  cudaError_t err = cudaDeviceSynchronize();
  if (err != cudaSuccess) {
    cudaFree(d_out);
    throw std::runtime_error(cudaGetErrorString(err));
  }

  std::vector<int> out(static_cast<size_t>(query_n), -1);
  cudaMemcpy(out.data(), d_out, sizeof(int) * query_n, cudaMemcpyDeviceToHost);
  cudaFree(d_out);
  return out;
}

void match_one_direction_cuda_u8_device_into(
    const CudaCascadeImageDeviceView& query_dev,
    const CudaCascadeImageDeviceView& train_dev,
    const cpu_cascade_hash::CascadeHashOptions& options,
    int* d_out,
    std::vector<int>* host_out) {
  const int query_n = query_dev.num_features;
  const int train_n = train_dev.num_features;
  if (host_out == nullptr) {
    throw std::runtime_error("host_out is null");
  }
  if (query_n <= 0 || train_n <= 0) {
    host_out->clear();
    return;
  }
  if (d_out == nullptr) {
    throw std::runtime_error("d_out is null");
  }

  launch_match_one_direction_cuda_u8_device(query_dev, train_dev, options, d_out, 0);
  cudaError_t err = cudaDeviceSynchronize();
  if (err != cudaSuccess) {
    throw std::runtime_error(cudaGetErrorString(err));
  }
  host_out->assign(static_cast<size_t>(query_n), -1);
  cudaMemcpy(host_out->data(), d_out, sizeof(int) * query_n, cudaMemcpyDeviceToHost);
}

void launch_match_one_direction_cuda_u8_device(
    const CudaCascadeImageDeviceView& query_dev,
    const CudaCascadeImageDeviceView& train_dev,
    const cpu_cascade_hash::CascadeHashOptions& options,
    int* d_out,
    cudaStream_t stream) {
  const int query_n = query_dev.num_features;
  const int train_n = train_dev.num_features;
  if (query_n <= 0 || train_n <= 0 || d_out == nullptr) {
    return;
  }
  const int threads = 128;
  const int blocks = (query_n + threads - 1) / threads;
  const float ratio_sq = options.ratio_test * options.ratio_test;
  const int keep_k = options.candidate_top_max < kMaxTopCandidates ? options.candidate_top_max : kMaxTopCandidates;
  if (options.bucket_groups == 6 && options.bucket_bits == 8 && keep_k == 10) {
    match_direction_kernel_u8_templ<6, 8, 10><<<blocks, threads, 0, stream>>>(
        query_dev.desc_u8, query_dev.hash, query_dev.bucket_ids, query_n,
        train_dev.desc_u8, train_dev.hash, train_dev.bucket_offsets, train_dev.bucket_indices, train_n,
        ratio_sq, d_out);
  } else {
    match_direction_kernel_u8_generic<<<blocks, threads, 0, stream>>>(
        query_dev.desc_u8, query_dev.hash, query_dev.bucket_ids, query_n,
        train_dev.desc_u8, train_dev.hash, train_dev.bucket_offsets, train_dev.bucket_indices, train_n,
        options.bucket_groups, options.bucket_bits, keep_k, ratio_sq, d_out);
  }
}

void launch_mutual_compact_u8_device(
    const int* d_forward_best,
    const int* d_backward_best,
    int query_n,
    int train_n,
    uint32_t* d_pairs_qj,
    int* d_pair_count,
    cudaStream_t stream) {
  if (query_n <= 0 || train_n <= 0 || d_forward_best == nullptr || d_backward_best == nullptr ||
      d_pairs_qj == nullptr || d_pair_count == nullptr) {
    return;
  }
  const int threads = 128;
  const int blocks = (query_n + threads - 1) / threads;
  mutual_compact_kernel_u8<<<blocks, threads, 0, stream>>>(
      d_forward_best, d_backward_best, query_n, train_n, d_pairs_qj, d_pair_count);
}

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
    cudaStream_t stream) {
  if (total_q <= 0 || num_pairs <= 0 || d_out == nullptr) return;
  const int keep_k = candidate_top_max < kMaxTopCandidates ? candidate_top_max : kMaxTopCandidates;
  const int threads = 128;
  const int blocks = (total_q + threads - 1) / threads;
  match_direction_batch_kernel_u8<<<blocks, threads, 0, stream>>>(
      d_view_q, d_view_t, d_q_offsets, num_pairs, total_q,
      groups, bucket_bits, keep_k, ratio_sq, d_out);
}

}  // namespace gpu_cascade_hash
}  // namespace algorithm
}  // namespace insight

