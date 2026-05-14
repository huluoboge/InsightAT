/**
 * isat_gpu_cascade_hashing_match.cpp
 * InsightAT GPU Cascade Hashing Matcher
 *
 * Pipeline (triangular-block strategy):
 *   Stage 0  [main thread]   Build global mean descriptor from sampled images.
 *   Stage 1  [1 IO thread]   Per block-pair job: load features + compute hash index.
 *             ↓ chain
 *   StageCurrent [main thread, CUDA]  Upload → batch kernel → download → free GPU/host mem.
 *             ↓ chain
 *   Stage 3  [N IO threads]  Write .isat_match files (results freed after write).
 *
 * Block-pair strategy:
 *   - All unique images are sorted and divided into image blocks of size B.
 *   - B is computed adaptively from available VRAM, capped by --image-block-size / 2.
 *   - Jobs are the upper-triangular block-pair matrix: (bi, bj) where bi ≤ bj.
 *     · Intra-block (bi == bj): up to B images on GPU, B*(B-1)/2 pairs.
 *     · Inter-block (bi < bj): up to 2B images on GPU, B_i*B_j pairs.
 *   - Every GPU launch processes the maximum number of pairs given the VRAM budget.
 *
 * CUDA thread safety:
 *   All CUDA API calls live inside the GpuStage lambda which runs in the main thread
 *   via StageCurrent. LoadStage and WriteStage workers are pure-CPU threads.
 */

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <glog/logging.h>
#include <iostream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <cuda_runtime.h>

#include "../io/idc_reader.h"
#include "../io/idc_writer.h"
#include "../modules/gpu_cascade_hash/gpu_cascade_hash.h"
#include "../modules/cpu_cascade_hash/cpu_cascade_hash.h"
#include "cli_logging.h"
#include "cmdLine/cmdLine.h"
#include "pair_json_utils.h"
#include "task_queue/task_queue.hpp"

namespace fs = std::filesystem;
using json = nlohmann::json;
using insight::algorithm::gpu_cascade_hash::GpuCascadeHashBlockMatcher;
using insight::algorithm::gpu_cascade_hash::GpuCascadeHashOptions;
using insight::algorithm::cpu_cascade_hash::CascadeHashOptions;
using insight::algorithm::cpu_cascade_hash::CascadeHashSampleModel;
using insight::algorithm::matching::DescriptorType;
using insight::algorithm::matching::FeatureData;
using insight::algorithm::matching::MatchResult;
using insight::io::IDCReader;
using insight::io::IDCWriter;

static constexpr const char* kEventPrefix = "ISAT_EVENT ";

static void print_event(const json& j) {
  std::cout << kEventPrefix << j.dump() << "\n";
  std::cout.flush();
}

static bool write_pairs_json(const std::string& output_path,
                             const std::vector<std::pair<uint32_t, uint32_t>>& pairs) {
  json pairs_arr = json::array();
  for (const auto& [i, j] : pairs)
    pairs_arr.push_back({{"image1_index", i}, {"image2_index", j}});
  std::ofstream f(output_path);
  if (!f.is_open())
    return false;
  f << json{{"pairs", pairs_arr}}.dump(2) << "\n";
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Data structures
// ─────────────────────────────────────────────────────────────────────────────

struct PairTask {
  uint32_t image1_index = 0;
  uint32_t image2_index = 0;
  std::string feature1_file;
  std::string feature2_file;
};

// Image descriptor in a block-pair job: file info (pre-load) + feature data (post-load).
struct LoadedImage {
  uint32_t image_index = 0;
  std::string feature_file;
  FeatureData features;
  insight::algorithm::cpu_cascade_hash::ImageFeatures hash_index;
  bool valid = false;

  // Called by GpuStage after upload to free CPU RAM immediately.
  void release_host_data() {
    features = FeatureData{};
    hash_index = insight::algorithm::cpu_cascade_hash::ImageFeatures{};
  }
};

// One unit of work for the pipeline: a pair of image-blocks to match.
struct BlockPairJob {
  int job_idx = 0;
  int block_a = 0;
  int block_b = 0;
  bool is_intra = false;  // block_a == block_b

  // Filled during job construction (tiny):
  // images_a is always populated; images_b is populated only for inter-block jobs.
  std::vector<LoadedImage> loaded_a;
  std::vector<LoadedImage> loaded_b;

  // Indices into the global pair_tasks vector that belong to this job.
  std::vector<int> pair_indices;

  // Filled by GpuStage; consumed and freed by WriteStage.
  std::vector<MatchResult> results;
  // Per-result scales flattened as [s1, s2] per match, aligned with results.
  std::vector<std::vector<float>> result_scales;

  int load_ms = 0;
  int gpu_ms = 0;
};

static std::vector<float> build_scales_flat(const MatchResult& matches,
                                            const FeatureData& left_features,
                                            const FeatureData& right_features) {
  std::vector<float> scales_flat;
  scales_flat.reserve(matches.num_matches * 2);
  const auto& kpts1 = left_features.keypoints;
  const auto& kpts2 = right_features.keypoints;
  for (size_t m = 0; m < matches.num_matches; ++m) {
    float s1 = 1.0f;
    float s2 = 1.0f;
    if (matches.indices[m].first < kpts1.size()) {
      s1 = kpts1[matches.indices[m].first](2);
    }
    if (matches.indices[m].second < kpts2.size()) {
      s2 = kpts2[matches.indices[m].second](2);
    }
    scales_flat.push_back(s1);
    scales_flat.push_back(s2);
  }
  return scales_flat;
}

// ─────────────────────────────────────────────────────────────────────────────
// I/O helpers
// ─────────────────────────────────────────────────────────────────────────────

static std::vector<PairTask> load_pairs_json(const std::string& json_path,
                                             const std::string& feature_dir) {
  std::ifstream file(json_path);
  if (!file.is_open()) {
    LOG(FATAL) << "Failed to open pairs file: " << json_path;
  }
  json j;
  file >> j;
  std::vector<PairTask> pairs;
  for (const auto& pair : j["pairs"]) {
    PairTask task;
    task.image1_index = insight::tools::get_image_index_from_pair(pair, "image1_index");
    task.image2_index = insight::tools::get_image_index_from_pair(pair, "image2_index");
    if (!feature_dir.empty()) {
      task.feature1_file = feature_dir + "/" + std::to_string(task.image1_index) + ".isat_feat";
      task.feature2_file = feature_dir + "/" + std::to_string(task.image2_index) + ".isat_feat";
    } else {
      task.feature1_file = pair["feature1_file"];
      task.feature2_file = pair["feature2_file"];
    }
    pairs.push_back(std::move(task));
  }
  return pairs;
}

static FeatureData load_features_idc(const std::string& idc_path) {
  IDCReader reader(idc_path);
  if (!reader.is_valid()) return FeatureData();
  const auto keypoints_raw = reader.read_blob<float>("keypoints");
  if (keypoints_raw.empty()) return FeatureData();
  const auto desc_blob = reader.get_blob_descriptor("descriptors");
  const std::string dtype = desc_blob["dtype"];
  const size_t num_features = keypoints_raw.size() / 4;
  DescriptorType descriptor_type =
      (dtype == "float32") ? DescriptorType::kFloat32 : DescriptorType::kUInt8;
  FeatureData features(num_features, descriptor_type);
  for (size_t i = 0; i < num_features; ++i) {
    features.keypoints[i] << keypoints_raw[i * 4], keypoints_raw[i * 4 + 1],
        keypoints_raw[i * 4 + 2], keypoints_raw[i * 4 + 3];
  }
  if (descriptor_type == DescriptorType::kUInt8) {
    features.descriptors_uint8 = reader.read_blob<uint8_t>("descriptors");
  } else {
    features.descriptors_float = reader.read_blob<float>("descriptors");
  }
  return features;
}

static bool write_match_idc(const MatchResult& matches, const PairTask& pair,
                            const std::vector<float>& scales_flat,
                            const std::string& output_dir) {
  if (matches.num_matches == 0) return false;
  const std::string output_file = output_dir + "/" + std::to_string(pair.image1_index) + "_" +
                                  std::to_string(pair.image2_index) + ".isat_match";
  json metadata;
  metadata["schema_version"] = "1.0";
  metadata["task_type"] = "feature_matching";
  metadata["algorithm"]["name"] = "CASCADE_HASH_GPU";
  metadata["algorithm"]["impl"] = "gpu_cascade_hash";
  metadata["algorithm"]["version"] = "1.0";
  metadata["image_pair"]["image1_index"] = pair.image1_index;
  metadata["image_pair"]["image2_index"] = pair.image2_index;
  metadata["metadata"]["num_matches"] = matches.num_matches;

  std::vector<uint16_t> indices_flat;
  indices_flat.reserve(matches.num_matches * 2);
  for (const auto& idx : matches.indices) {
    indices_flat.push_back(idx.first);
    indices_flat.push_back(idx.second);
  }
  std::vector<float> coords_flat;
  coords_flat.reserve(matches.num_matches * 4);
  for (const auto& coord : matches.coords_pixel) {
    coords_flat.push_back(coord(0));
    coords_flat.push_back(coord(1));
    coords_flat.push_back(coord(2));
    coords_flat.push_back(coord(3));
  }
  std::vector<float> distances(matches.num_matches, 0.0f);
  IDCWriter writer(output_file);
  writer.set_metadata(metadata);
  writer.add_blob("indices", indices_flat.data(), indices_flat.size() * sizeof(uint16_t), "uint16",
                  {static_cast<int>(matches.num_matches), 2});
  writer.add_blob("coords_pixel", coords_flat.data(), coords_flat.size() * sizeof(float),
                  "float32", {static_cast<int>(matches.num_matches), 4});
  writer.add_blob("scales", scales_flat.data(), scales_flat.size() * sizeof(float), "float32",
                  {static_cast<int>(matches.num_matches), 2});
  writer.add_blob("distances", distances.data(), distances.size() * sizeof(float), "float32",
                  {static_cast<int>(matches.num_matches)});
  return writer.write();
}

// ─────────────────────────────────────────────────────────────────────────────
// Global mean descriptor computation
// ─────────────────────────────────────────────────────────────────────────────

static bool collect_sample_image_files(const std::vector<PairTask>& pair_tasks,
                                       size_t max_sample_images,
                                       std::vector<std::string>* sample_files) {
  sample_files->clear();
  std::unordered_map<uint32_t, std::string> id_to_file;
  id_to_file.reserve(max_sample_images * 2);
  for (const auto& task : pair_tasks) {
    if (id_to_file.size() < max_sample_images &&
        id_to_file.find(task.image1_index) == id_to_file.end())
      id_to_file.emplace(task.image1_index, task.feature1_file);
    if (id_to_file.size() < max_sample_images &&
        id_to_file.find(task.image2_index) == id_to_file.end())
      id_to_file.emplace(task.image2_index, task.feature2_file);
    if (id_to_file.size() >= max_sample_images) break;
  }
  for (const auto& kv : id_to_file) sample_files->push_back(kv.second);
  return !sample_files->empty();
}

// Returns true if any features were found.
// Also outputs avg_features_per_image (used for VRAM-adaptive block size).
static bool accumulate_sample_mean_async(const std::vector<std::string>& sample_files,
                                         int num_threads, std::vector<double>* mean_sum,
                                         uint64_t* total_sample_features,
                                         double* avg_features_per_image) {
  if (sample_files.empty() || num_threads <= 0) return false;
  constexpr int kDim = 128;
  mean_sum->assign(kDim, 0.0);
  *total_sample_features = 0;
  std::vector<std::array<double, kDim>> thread_sum(static_cast<size_t>(num_threads));
  std::vector<uint64_t> thread_count(static_cast<size_t>(num_threads), 0);
  std::vector<int> thread_valid(static_cast<size_t>(num_threads), 0);
  Stage sample_stage(
      "GpuSampleMean", num_threads, 16,
      [&sample_files, &thread_sum, &thread_count, &thread_valid, num_threads, kDim](int index) {
        const int slot = task_queue_context::current_worker_index();
        if (slot < 0 || slot >= num_threads) return;
        const FeatureData features = load_features_idc(sample_files[static_cast<size_t>(index)]);
        if (features.num_features == 0) return;
        ++thread_valid[static_cast<size_t>(slot)];
        auto& sum = thread_sum[static_cast<size_t>(slot)];
        if (features.descriptor_type == DescriptorType::kUInt8) {
          for (size_t i = 0; i < features.num_features; ++i)
            for (int d = 0; d < kDim; ++d)
              sum[static_cast<size_t>(d)] += features.descriptors_uint8[i * kDim + d];
        } else {
          for (size_t i = 0; i < features.num_features; ++i)
            for (int d = 0; d < kDim; ++d)
              sum[static_cast<size_t>(d)] += features.descriptors_float[i * kDim + d];
        }
        thread_count[static_cast<size_t>(slot)] += features.num_features;
      });
  sample_stage.setTaskCount(static_cast<int>(sample_files.size()));
  for (int i = 0; i < static_cast<int>(sample_files.size()); ++i) sample_stage.push(i);
  sample_stage.wait();
  int valid_images = 0;
  for (int t = 0; t < num_threads; ++t) {
    for (int d = 0; d < kDim; ++d)
      (*mean_sum)[static_cast<size_t>(d)] += thread_sum[static_cast<size_t>(t)][static_cast<size_t>(d)];
    *total_sample_features += thread_count[static_cast<size_t>(t)];
    valid_images += thread_valid[static_cast<size_t>(t)];
  }
  *avg_features_per_image = (valid_images > 0)
      ? static_cast<double>(*total_sample_features) / valid_images
      : 0.0;
  return *total_sample_features > 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// VRAM-adaptive block size
// ─────────────────────────────────────────────────────────────────────────────

// Computes the single-block size B such that an inter-block job (two blocks of size B)
// fits within available VRAM.
//
// VRAM budget for inter-block job:
//   image data:   2*B * avg_feat * 200 bytes/feat (desc + hash + bucket data, conservative)
//   output buf:   2 * B^2 * avg_feat * 4 bytes     (fwd + bwd best-match indices)
//
// Solving:  8*f*B^2 + 400*f*B - usable = 0
//   B = (-400 + sqrt(400^2 + 4*8*usable/f)) / (2*8)
//     = (-400 + sqrt(160000 + 32*usable/f)) / 16
//
// Returns min(user_cap, computed_B), at least 1.
static int compute_adaptive_block_size(int cuda_device, double avg_features_per_image,
                                       int user_max_combined_images) {
  cudaSetDevice(cuda_device);
  size_t free_bytes = 0, total_bytes = 0;
  if (cudaMemGetInfo(&free_bytes, &total_bytes) != cudaSuccess) {
    LOG(WARNING) << "cudaMemGetInfo failed, falling back to user cap / 2";
    return std::max(1, user_max_combined_images / 2);
  }

  const double safety = 0.72;
  const double usable = static_cast<double>(free_bytes) * safety;
  const double f = std::max(1.0, avg_features_per_image);

  // Quadratic formula: B = (-400 + sqrt(160000 + 32*usable/f)) / 16
  const double disc = 160000.0 + 32.0 * usable / f;
  const int computed_B = static_cast<int>((-400.0 + std::sqrt(disc)) / 16.0);

  // Also cap by user limit (user_max_combined_images is the max images held simultaneously,
  // i.e., 2*B for inter-block). So single-block cap = user_max_combined_images / 2.
  const int user_cap_B = std::max(1, user_max_combined_images / 2);
  const int result = std::max(1, std::min(computed_B, user_cap_B));

  LOG(INFO) << "VRAM adaptive block size:"
            << " free=" << (free_bytes / 1024 / 1024) << "MB"
            << " avg_feat=" << static_cast<int>(f)
            << " vram_B=" << computed_B
            << " user_cap_B=" << user_cap_B
            << " final_B=" << result;
  return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// Block-pair job construction (triangular block matrix)
// ─────────────────────────────────────────────────────────────────────────────

static std::vector<BlockPairJob> build_block_pair_jobs(
    const std::vector<PairTask>& pair_tasks, int block_size) {
  // Collect all unique image IDs and map them to feature files.
  std::unordered_map<uint32_t, std::string> image_to_file;
  image_to_file.reserve(pair_tasks.size() * 2);
  for (const auto& t : pair_tasks) {
    image_to_file.emplace(t.image1_index, t.feature1_file);
    image_to_file.emplace(t.image2_index, t.feature2_file);
  }

  // Sort image IDs for deterministic, cache-friendly block assignment.
  std::vector<uint32_t> sorted_ids;
  sorted_ids.reserve(image_to_file.size());
  for (const auto& kv : image_to_file) sorted_ids.push_back(kv.first);
  std::sort(sorted_ids.begin(), sorted_ids.end());

  const int n_images = static_cast<int>(sorted_ids.size());
  const int n_blocks = (n_images + block_size - 1) / block_size;

  // image_id → block index
  std::unordered_map<uint32_t, int> image_to_block;
  image_to_block.reserve(static_cast<size_t>(n_images));
  for (int i = 0; i < n_images; ++i) {
    image_to_block[sorted_ids[static_cast<size_t>(i)]] = i / block_size;
  }

  // block index → sorted image IDs in that block
  std::vector<std::vector<uint32_t>> block_images(static_cast<size_t>(n_blocks));
  for (int i = 0; i < n_images; ++i) {
    block_images[static_cast<size_t>(i / block_size)].push_back(sorted_ids[static_cast<size_t>(i)]);
  }

  // Create one BlockPairJob per (bi, bj) with bi ≤ bj.
  // Total jobs = n_blocks * (n_blocks + 1) / 2, typically very small.
  std::vector<BlockPairJob> jobs;
  jobs.reserve(static_cast<size_t>(n_blocks * (n_blocks + 1) / 2));

  // (bi*n_blocks + bj) → job index for fast pair assignment.
  std::unordered_map<int, int> pair_key_to_job;
  pair_key_to_job.reserve(static_cast<size_t>(n_blocks * (n_blocks + 1) / 2));

  for (int bi = 0; bi < n_blocks; ++bi) {
    for (int bj = bi; bj < n_blocks; ++bj) {
      const int key = bi * n_blocks + bj;
      pair_key_to_job[key] = static_cast<int>(jobs.size());

      BlockPairJob job;
      job.job_idx = static_cast<int>(jobs.size());
      job.block_a = bi;
      job.block_b = bj;
      job.is_intra = (bi == bj);

      // Pre-populate LoadedImage lists with image indices and file paths.
      const auto& imgs_a = block_images[static_cast<size_t>(bi)];
      job.loaded_a.reserve(imgs_a.size());
      for (uint32_t id : imgs_a) {
        LoadedImage li;
        li.image_index = id;
        li.feature_file = image_to_file.at(id);
        job.loaded_a.push_back(std::move(li));
      }
      if (!job.is_intra) {
        const auto& imgs_b = block_images[static_cast<size_t>(bj)];
        job.loaded_b.reserve(imgs_b.size());
        for (uint32_t id : imgs_b) {
          LoadedImage li;
          li.image_index = id;
          li.feature_file = image_to_file.at(id);
          job.loaded_b.push_back(std::move(li));
        }
      }
      jobs.push_back(std::move(job));
    }
  }

  // Assign each pair task to its corresponding block-pair job.
  for (int pi = 0; pi < static_cast<int>(pair_tasks.size()); ++pi) {
    const auto& t = pair_tasks[static_cast<size_t>(pi)];
    const auto it1 = image_to_block.find(t.image1_index);
    const auto it2 = image_to_block.find(t.image2_index);
    if (it1 == image_to_block.end() || it2 == image_to_block.end()) continue;
    const int lo = std::min(it1->second, it2->second);
    const int hi = std::max(it1->second, it2->second);
    const int key = lo * n_blocks + hi;
    const auto jt = pair_key_to_job.find(key);
    if (jt != pair_key_to_job.end()) {
      jobs[static_cast<size_t>(jt->second)].pair_indices.push_back(pi);
    }
  }

  // Remove jobs that have no matching pairs.
  jobs.erase(std::remove_if(jobs.begin(), jobs.end(),
                            [](const BlockPairJob& j) { return j.pair_indices.empty(); }),
             jobs.end());

  // Re-assign sequential indices.
  for (int i = 0; i < static_cast<int>(jobs.size()); ++i) {
    jobs[static_cast<size_t>(i)].job_idx = i;
  }

  LOG(INFO) << "Block-pair jobs: " << jobs.size()
            << " (images=" << n_images << ", blocks=" << n_blocks
            << ", block_size=" << block_size << ")";
  for (const auto& job : jobs) {
    const int na = static_cast<int>(job.loaded_a.size());
    const int nb = static_cast<int>(job.loaded_b.size());
    VLOG(1) << "  Job " << job.job_idx << ": block(" << job.block_a << "," << job.block_b << ")"
            << (job.is_intra ? " intra" : " inter")
            << " images=" << (na + nb)
            << " pairs=" << job.pair_indices.size();
  }
  return jobs;
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;
  CmdLine cmd("InsightAT GPU Cascade Hashing Matcher");

  std::string pairs_json;
  std::string output_dir;
  std::string output_pairs_json;
  std::string feature_dir;
  int num_threads = -1;
  int sample_images = 256;
  int image_block_size = 1000;  // max images held simultaneously in GPU (= 2*B for inter-block)
  int min_output_matches = 16;
  int cuda_device = 0;
  std::string log_level;

  cmd.add(make_option('i', pairs_json, "input").doc("Input pairs list (JSON format)"));
  cmd.add(make_option('o', output_dir, "output").doc("Output directory for .isat_match files"));
  cmd.add(make_option(0, output_pairs_json, "output-pairs-json")
              .doc("Optional JSON path listing only pairs whose .isat_match files were written."));
  cmd.add(make_option('f', feature_dir, "feature-dir").doc("Feature directory (.isat_feat files)"));
  cmd.add(make_option('j', num_threads, "threads").doc("I/O worker threads (-1 = auto)"));
  cmd.add(make_option(0, sample_images, "sample-images")
              .doc("Images for global mean descriptor (default: 256)"));
  cmd.add(make_option(0, image_block_size, "image-block-size")
              .doc("Max images held in GPU simultaneously; single block = half this value. "
                   "VRAM-adaptive sizing is applied automatically (default: 1000)"));
  cmd.add(make_option(0, min_output_matches, "min-output-matches")
              .doc("Skip writing pair if matches below this threshold (default: 16)"));
  cmd.add(make_option(0, cuda_device, "cuda-device").doc("CUDA device id (default: 0)"));
  cmd.add(make_option(0, log_level, "log-level").doc("Log level: error|warn|info|debug"));
  cmd.add(make_switch('v', "verbose").doc("Verbose logging (INFO level)"));
  cmd.add(make_switch('q', "quiet").doc("Quiet mode (ERROR level only)"));
  cmd.add(make_switch('h', "help").doc("Show help message"));

  try {
    cmd.process(argc, argv);
  } catch (const std::string& s) {
    std::cerr << "Error: " << s << "\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }
  if (cmd.checkHelp(argv[0])) return 0;
  if (pairs_json.empty() || output_dir.empty()) {
    std::cerr << "Error: -i and -o are required\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }
  if (num_threads == -1) {
    const unsigned int hw = std::thread::hardware_concurrency();
    num_threads = static_cast<int>(hw > 0 ? hw : 4);
  }
  if (num_threads <= 0 || image_block_size <= 0) {
    std::cerr << "Error: --threads and --image-block-size must be > 0\n";
    return 1;
  }
  insight::tools::apply_log_level(cmd.used('v'), cmd.used('q'), log_level);

  // ── Load pair list ────────────────────────────────────────────────────────
  std::vector<PairTask> pair_tasks = load_pairs_json(pairs_json, feature_dir);
  const int total_pairs = static_cast<int>(pair_tasks.size());
  if (total_pairs == 0) {
    LOG(ERROR) << "No pairs to process";
    return 1;
  }
  fs::create_directories(output_dir);
  LOG(INFO) << "Total pairs: " << total_pairs;

  // ── Compute global mean descriptor ────────────────────────────────────────
  std::vector<std::string> sample_files;
  if (!collect_sample_image_files(pair_tasks, static_cast<size_t>(sample_images), &sample_files)) {
    LOG(ERROR) << "Failed to collect sample image files";
    return 1;
  }
  std::vector<double> mean_sum(128, 0.0);
  uint64_t total_sample_features = 0;
  double avg_features_per_image = 0.0;
  if (!accumulate_sample_mean_async(sample_files, num_threads, &mean_sum, &total_sample_features,
                                    &avg_features_per_image)) {
    LOG(ERROR) << "No valid sample features for mean descriptor";
    return 1;
  }
  std::vector<float> mean_descriptor(128, 0.0f);
  for (int d = 0; d < 128; ++d)
    mean_descriptor[d] = static_cast<float>(mean_sum[d] / static_cast<double>(total_sample_features));
  LOG(INFO) << "Mean descriptor built from " << sample_files.size() << " sample images"
            << " (avg_features=" << static_cast<int>(avg_features_per_image) << ")";

  // Build CPU hash model (shared across all jobs, read-only after this point).
  CascadeHashOptions hash_options;
  const CascadeHashSampleModel hash_model =
      insight::algorithm::cpu_cascade_hash::build_sample_model_from_mean_descriptor(
          mean_descriptor, hash_options);

  // ── VRAM-adaptive block size ──────────────────────────────────────────────
  const int block_size =
      compute_adaptive_block_size(cuda_device, avg_features_per_image, image_block_size);

  // ── Build block-pair jobs (triangular matrix traversal) ───────────────────
  std::vector<BlockPairJob> block_jobs = build_block_pair_jobs(pair_tasks, block_size);
  const int total_jobs = static_cast<int>(block_jobs.size());
  if (total_jobs == 0) {
    LOG(ERROR) << "No block-pair jobs generated";
    return 1;
  }

  // ── Progress tracking ─────────────────────────────────────────────────────
  std::atomic<int> jobs_done{0};
  const auto t_pipeline_start = std::chrono::steady_clock::now();

  // ── Stage 1: LoadStage (1 worker, uses inner Stages for parallelism) ─────
  // Bounded queue capacity 3: LoadStage may be at most 3 jobs ahead of GpuStage.
  Stage load_stage(
      "GpuLoadBlockPair", 1, 3,
      [&block_jobs, &hash_model, num_threads](int job_idx) {
        auto& job = block_jobs[static_cast<size_t>(job_idx)];
        const auto t0 = std::chrono::high_resolution_clock::now();

        const int n_a = static_cast<int>(job.loaded_a.size());
        const int n_b = static_cast<int>(job.loaded_b.size());
        const int total_img = n_a + n_b;

        // Parallel feature file loading for all images in this job.
        Stage inner_load(
            "InnerLoad", num_threads, 64, [&job, n_a](int flat) {
              LoadedImage& li = (flat < n_a) ? job.loaded_a[static_cast<size_t>(flat)]
                                             : job.loaded_b[static_cast<size_t>(flat - n_a)];
              li.features = load_features_idc(li.feature_file);
              li.valid = (li.features.num_features > 0);
            });
        inner_load.setTaskCount(total_img);
        for (int i = 0; i < total_img; ++i) inner_load.push(i);
        inner_load.wait();

        // Parallel hash index computation.
        Stage inner_hash(
            "InnerHash", num_threads, 64, [&job, &hash_model, n_a](int flat) {
              LoadedImage& li = (flat < n_a) ? job.loaded_a[static_cast<size_t>(flat)]
                                             : job.loaded_b[static_cast<size_t>(flat - n_a)];
              if (!li.valid) return;
              li.hash_index = insight::algorithm::cpu_cascade_hash::compute_image_features(
                  li.features, hash_model);
            });
        inner_hash.setTaskCount(total_img);
        for (int i = 0; i < total_img; ++i) inner_hash.push(i);
        inner_hash.wait();

        job.load_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::high_resolution_clock::now() - t0).count());
      });

  // ── StageCurrent: GpuStage (main thread – all CUDA calls happen here) ────
  // Bounded queue capacity 2: GpuStage accepts at most 2 pre-loaded jobs.
  StageCurrent gpu_stage(
      "GpuMatchStage", 1, 2,
      [&block_jobs, &pair_tasks, &mean_descriptor, cuda_device, &jobs_done,
       total_jobs, &t_pipeline_start](int job_idx) {
        auto& job = block_jobs[static_cast<size_t>(job_idx)];
        const auto t0 = std::chrono::high_resolution_clock::now();

        // Create a fresh matcher for this job.
        GpuCascadeHashOptions gpu_options;
        gpu_options.mean_descriptor = mean_descriptor;
        gpu_options.cuda_device_id = cuda_device;
        GpuCascadeHashBlockMatcher matcher(gpu_options);

        // Upload both block's features to GPU.
        for (auto& li : job.loaded_a)
          if (li.valid)
            matcher.add_image_with_index(li.image_index, &li.features, &li.hash_index);
        for (auto& li : job.loaded_b)
          if (li.valid)
            matcher.add_image_with_index(li.image_index, &li.features, &li.hash_index);
        matcher.finalize();

        // Build pair list for this job and run batch GPU matching.
        std::vector<std::pair<uint32_t, uint32_t>> gpu_pairs;
        gpu_pairs.reserve(job.pair_indices.size());
        for (int pi : job.pair_indices)
          gpu_pairs.push_back({pair_tasks[static_cast<size_t>(pi)].image1_index,
                               pair_tasks[static_cast<size_t>(pi)].image2_index});
        job.results = matcher.match_pairs(gpu_pairs);
        job.result_scales.clear();
        job.result_scales.reserve(job.results.size());
        std::unordered_map<uint32_t, const FeatureData*> image_features;
        image_features.reserve(job.loaded_a.size() + job.loaded_b.size());
        for (const auto& li : job.loaded_a) {
          if (li.valid) {
            image_features.emplace(li.image_index, &li.features);
          }
        }
        for (const auto& li : job.loaded_b) {
          if (li.valid) {
            image_features.emplace(li.image_index, &li.features);
          }
        }
        for (size_t k = 0; k < job.pair_indices.size() && k < job.results.size(); ++k) {
          const PairTask& pair = pair_tasks[static_cast<size_t>(job.pair_indices[k])];
          const auto it1 = image_features.find(pair.image1_index);
          const auto it2 = image_features.find(pair.image2_index);
          if (it1 == image_features.end() || it2 == image_features.end()) {
            job.result_scales.emplace_back(job.results[k].num_matches * 2, 1.0f);
            continue;
          }
          job.result_scales.push_back(build_scales_flat(job.results[k], *it1->second, *it2->second));
        }

        // matcher destructor frees all GPU (device) memory here.

        // Immediately release host feature data to reclaim CPU RAM.
        for (auto& li : job.loaded_a) li.release_host_data();
        for (auto& li : job.loaded_b) li.release_host_data();
        job.loaded_a.clear();
        job.loaded_a.shrink_to_fit();
        job.loaded_b.clear();
        job.loaded_b.shrink_to_fit();

        job.gpu_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::high_resolution_clock::now() - t0).count());

        // Progress to stderr (one line, overwrites itself if TTY).
        const int done = ++jobs_done;
        const double elapsed_s = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t_pipeline_start).count();
        const double eta_s =
            (done < total_jobs) ? elapsed_s / done * (total_jobs - done) : 0.0;
        const int pct = done * 100 / total_jobs;

        char prog_buf[128];
        const int na = static_cast<int>(
            block_jobs[static_cast<size_t>(job_idx)].pair_indices.size());
        std::snprintf(prog_buf, sizeof(prog_buf),
                      "\r[GPU cascade] %d/%d (%d%%) pairs=%d load=%dms gpu=%dms ETA=%.0fs  ",
                      done, total_jobs, pct, na, job.load_ms, job.gpu_ms, eta_s);
        std::cerr << prog_buf;
        if (done == total_jobs) std::cerr << "\n";
        std::cerr.flush();
      });

  // ── Stage 3: WriteStage (multi-threaded IO) ───────────────────────────────
  // Bounded queue capacity 6: allows GPU to stay 6 jobs ahead of disk write.
  std::mutex written_pairs_mu;
  std::vector<std::pair<uint32_t, uint32_t>> written_pairs;
  written_pairs.reserve(static_cast<size_t>(total_pairs));
  std::atomic<int> total_matches_written{0};
  Stage write_stage(
      "GpuWriteStage", num_threads, 6,
      [&block_jobs, &pair_tasks, &output_dir, min_output_matches, &written_pairs_mu,
       &written_pairs, &total_matches_written](int job_idx) {
        auto& job = block_jobs[static_cast<size_t>(job_idx)];
        for (size_t k = 0; k < job.pair_indices.size(); ++k) {
          if (k >= job.results.size()) break;
          const MatchResult& res = job.results[k];
          if (static_cast<int>(res.num_matches) < min_output_matches) continue;
          const std::vector<float>& scales =
              (k < job.result_scales.size()) ? job.result_scales[k] : std::vector<float>{};
          if (scales.empty()) {
            std::vector<float> default_scales(res.num_matches * 2, 1.0f);
            if (write_match_idc(res, pair_tasks[static_cast<size_t>(job.pair_indices[k])],
                                default_scales, output_dir)) {
              const auto& pair = pair_tasks[static_cast<size_t>(job.pair_indices[k])];
              total_matches_written.fetch_add(static_cast<int>(res.num_matches),
                                              std::memory_order_relaxed);
              std::lock_guard<std::mutex> lock(written_pairs_mu);
              written_pairs.emplace_back(pair.image1_index, pair.image2_index);
            }
          } else {
            if (write_match_idc(res, pair_tasks[static_cast<size_t>(job.pair_indices[k])],
                                scales, output_dir)) {
              const auto& pair = pair_tasks[static_cast<size_t>(job.pair_indices[k])];
              total_matches_written.fetch_add(static_cast<int>(res.num_matches),
                                              std::memory_order_relaxed);
              std::lock_guard<std::mutex> lock(written_pairs_mu);
              written_pairs.emplace_back(pair.image1_index, pair.image2_index);
            }
          }
        }
        // Free match results immediately after writing.
        job.results.clear();
        job.results.shrink_to_fit();
        job.result_scales.clear();
        job.result_scales.shrink_to_fit();
        job.pair_indices.clear();
        job.pair_indices.shrink_to_fit();
      });

  // ── Wire pipeline and run ─────────────────────────────────────────────────
  chain(load_stage, gpu_stage);
  chain(gpu_stage, write_stage);

  load_stage.setTaskCount(total_jobs);
  gpu_stage.setTaskCount(total_jobs);
  write_stage.setTaskCount(total_jobs);

  // Push tasks from a background thread (so main thread can run GpuStage).
  std::thread push_thread([&]() {
    for (int i = 0; i < total_jobs; ++i) {
      load_stage.push(i);
    }
  });

  // Run GPU stage in main thread (CUDA context stays on main thread).
  gpu_stage.run();

  push_thread.join();
  load_stage.wait();
  write_stage.wait();

  const double total_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - t_pipeline_start).count();

  if (!output_pairs_json.empty()) {
    if (!write_pairs_json(output_pairs_json, written_pairs)) {
      LOG(ERROR) << "Failed to write output pairs JSON: " << output_pairs_json;
      return 1;
    }
  }

  int total_matches = 0;
  const int pairs_with_matches = static_cast<int>(written_pairs.size());
  total_matches = total_matches_written.load(std::memory_order_relaxed);
  LOG(INFO) << "GPU cascade complete: jobs=" << total_jobs
            << " total_pairs=" << total_pairs
            << " elapsed=" << total_s << "s";
  print_event({{"type", "match.complete"},
               {"ok", true},
               {"data",
                {{"total_pairs", total_pairs},
                 {"pairs_with_matches", pairs_with_matches},
                 {"total_matches", total_matches},
                 {"failed_pairs", total_pairs - pairs_with_matches},
                 {"total_time_s", total_s},
                 {"image_block_size", image_block_size},
                 {"min_output_matches", min_output_matches},
                 {"output_dir", output_dir},
                 {"output_pairs_json", output_pairs_json}}}});
  return 0;
}
