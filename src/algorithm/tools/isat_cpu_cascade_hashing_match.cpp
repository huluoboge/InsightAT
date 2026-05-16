/**
 * isat_cpu_cascade_hashing_match.cpp
 * InsightAT CPU Cascade Hashing Matcher
 *
 * Reads pair JSON + .isat_feat files, builds one global cascade-hash sample model,
 * then runs streamed CPU matching and writes .isat_match files.
 *
 * Pipeline (Stage/chain):
 *   Stage 0  [single-thread]      Build global sample model from sampled images
 *   Stage 1  [multi-thread I/O]   Load features + compute image hash features
 *   Stage 2  [multi-thread CPU]   Cascade-hash matching
 *   Stage 3  [multi-thread I/O]   Write .isat_match
 *
 * Usage:
 *   isat_cpu_cascade_hashing_match -i pairs.json -f feat_dir/ -o match_dir/
 */

#include <chrono>
#include <array>
#include <thread>
#include <filesystem>
#include <fstream>
#include <future>
#include <glog/logging.h>
#include <iostream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "../io/idc_reader.h"
#include "../io/idc_writer.h"
#include "../modules/cpu_cascade_hash/cpu_cascade_hash.h"
#include "cli_logging.h"
#include "cmdLine/cmdLine.h"
#include "pair_json_utils.h"
#include "task_queue/task_queue.hpp"

namespace fs = std::filesystem;
using json = nlohmann::json;
using insight::algorithm::cpu_cascade_hash::CascadeHashOptions;
using insight::algorithm::cpu_cascade_hash::CascadeHashSampleModel;
using insight::algorithm::cpu_cascade_hash::ImageFeatures;
using insight::algorithm::cpu_cascade_hash::build_sample_model_from_mean_descriptor;
using insight::algorithm::cpu_cascade_hash::compute_image_features;
using insight::algorithm::cpu_cascade_hash::match_cascade_hash;
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

struct PairTask {
  uint32_t image1_index = 0;
  uint32_t image2_index = 0;
  std::string feature1_file;
  std::string feature2_file;
  float priority = 1.0f;
  int index = 0;
  int image1_cache_idx = -1;
  int image2_cache_idx = -1;
  MatchResult matches;
  std::vector<float> match_scales;
};

struct ImageCacheEntry {
  uint32_t image_index = 0;
  std::string feature_file;
  FeatureData features;
  ImageFeatures image_features;
  bool valid = false;
};

struct BlockRange {
  int begin = 0;
  int end = 0;  // exclusive
};

struct BlockRuntimeData {
  int block_begin = 0;
  int block_end = 0;
  int preload_ms = 0;
  int unique_images = 0;
  int valid_images = 0;
  std::vector<ImageCacheEntry> image_cache;
};

static std::vector<PairTask> load_pairs_json(const std::string& json_path,
                                             const std::string& feature_dir) {
  std::ifstream file(json_path);
  if (!file.is_open()) {
    LOG(FATAL) << "Failed to open pairs file: " << json_path;
  }

  json j;
  file >> j;

  std::vector<PairTask> pairs;
  int index = 0;
  for (const auto& pair : j["pairs"]) {
    PairTask task;
    task.image1_index = insight::tools::get_image_index_from_pair(pair, "image1_index");
    task.image2_index = insight::tools::get_image_index_from_pair(pair, "image2_index");
    task.priority = pair.value("priority", 1.0f);
    task.index = index++;

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
  if (!reader.is_valid()) {
    LOG(ERROR) << "Invalid IDC file: " << idc_path;
    return FeatureData();
  }

  const auto keypoints_raw = reader.read_blob<float>("keypoints");
  if (keypoints_raw.empty()) {
    LOG(ERROR) << "Failed to read keypoints from " << idc_path;
    return FeatureData();
  }

  const auto desc_blob = reader.get_blob_descriptor("descriptors");
  const std::string dtype = desc_blob["dtype"];
  const size_t num_features = keypoints_raw.size() / 4;

  DescriptorType descriptor_type = DescriptorType::kUInt8;
  if (dtype == "uint8") {
    descriptor_type = DescriptorType::kUInt8;
  } else if (dtype == "float32") {
    descriptor_type = DescriptorType::kFloat32;
  } else {
    LOG(ERROR) << "Unsupported descriptor dtype: " << dtype << " in " << idc_path;
    return FeatureData();
  }

  FeatureData features(num_features, descriptor_type);
  for (size_t i = 0; i < num_features; ++i) {
    features.keypoints[i] << keypoints_raw[i * 4 + 0], keypoints_raw[i * 4 + 1],
        keypoints_raw[i * 4 + 2], keypoints_raw[i * 4 + 3];
  }

  if (descriptor_type == DescriptorType::kUInt8) {
    features.descriptors_uint8 = reader.read_blob<uint8_t>("descriptors");
    if (features.descriptors_uint8.empty()) {
      LOG(ERROR) << "Failed to read uint8 descriptors from " << idc_path;
      return FeatureData();
    }
  } else {
    features.descriptors_float = reader.read_blob<float>("descriptors");
    if (features.descriptors_float.empty()) {
      LOG(ERROR) << "Failed to read float descriptors from " << idc_path;
      return FeatureData();
    }
  }
  return features;
}

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

static bool write_match_idc(const MatchResult& matches, const PairTask& pair,
                            const std::string& output_dir) {
  if (matches.num_matches == 0) {
    return false;
  }

  const std::string output_file = output_dir + "/" + std::to_string(pair.image1_index) + "_" +
                                  std::to_string(pair.image2_index) + ".isat_match";

  json metadata;
  metadata["schema_version"] = "1.0";
  metadata["task_type"] = "feature_matching";
  metadata["algorithm"]["name"] = "CASCADE_HASH_CPU";
  metadata["algorithm"]["impl"] = "cpu_cascade_hash";
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
  if (matches.distances.size() == matches.num_matches) {
    distances = matches.distances;
  }

  IDCWriter writer(output_file);
  writer.set_metadata(metadata);
  writer.add_blob("indices", indices_flat.data(), indices_flat.size() * sizeof(uint16_t), "uint16",
                  {static_cast<int>(matches.num_matches), 2});
  writer.add_blob("coords_pixel", coords_flat.data(), coords_flat.size() * sizeof(float), "float32",
                  {static_cast<int>(matches.num_matches), 4});
  writer.add_blob("scales", pair.match_scales.data(), pair.match_scales.size() * sizeof(float),
                  "float32", {static_cast<int>(matches.num_matches), 2});
  writer.add_blob("distances", distances.data(), distances.size() * sizeof(float), "float32",
                  {static_cast<int>(matches.num_matches)});
  if (!writer.write()) {
    LOG(ERROR) << "Failed to write match file: " << output_file;
    return false;
  }
  return true;
}

static bool collect_sample_image_files(const std::vector<PairTask>& pair_tasks,
                                       size_t max_sample_images,
                                       std::vector<std::string>* sample_files) {
  sample_files->clear();
  sample_files->reserve(max_sample_images);
  std::unordered_map<uint32_t, std::string> id_to_file;
  id_to_file.reserve(max_sample_images * 2);
  for (const auto& task : pair_tasks) {
    if (id_to_file.size() < max_sample_images && id_to_file.find(task.image1_index) == id_to_file.end()) {
      id_to_file.emplace(task.image1_index, task.feature1_file);
    }
    if (id_to_file.size() < max_sample_images && id_to_file.find(task.image2_index) == id_to_file.end()) {
      id_to_file.emplace(task.image2_index, task.feature2_file);
    }
    if (id_to_file.size() >= max_sample_images) {
      break;
    }
  }

  for (const auto& kv : id_to_file) {
    sample_files->push_back(kv.second);
  }
  return !sample_files->empty();
}

static std::vector<ImageCacheEntry> collect_unique_images(const std::vector<PairTask>& pair_tasks,
                                                          std::unordered_map<uint32_t, int>* image_to_cache_idx) {
  std::vector<ImageCacheEntry> images;
  image_to_cache_idx->clear();
  image_to_cache_idx->reserve(pair_tasks.size() * 2);
  for (const auto& task : pair_tasks) {
    if (image_to_cache_idx->find(task.image1_index) == image_to_cache_idx->end()) {
      const int idx = static_cast<int>(images.size());
      image_to_cache_idx->emplace(task.image1_index, idx);
      images.push_back({task.image1_index, task.feature1_file, FeatureData(), ImageFeatures(), false});
    }
    if (image_to_cache_idx->find(task.image2_index) == image_to_cache_idx->end()) {
      const int idx = static_cast<int>(images.size());
      image_to_cache_idx->emplace(task.image2_index, idx);
      images.push_back({task.image2_index, task.feature2_file, FeatureData(), ImageFeatures(), false});
    }
  }
  return images;
}

static std::vector<BlockRange> build_blocks_by_unique_images(const std::vector<PairTask>& pair_tasks,
                                                             int image_block_size) {
  std::vector<BlockRange> blocks;
  if (pair_tasks.empty() || image_block_size <= 0) {
    return blocks;
  }
  int begin = 0;
  std::unordered_map<uint32_t, uint8_t> image_seen;
  image_seen.reserve(static_cast<size_t>(image_block_size) * 2);
  for (int i = 0; i < static_cast<int>(pair_tasks.size()); ++i) {
    const auto& task = pair_tasks[static_cast<size_t>(i)];
    int new_images = 0;
    if (image_seen.find(task.image1_index) == image_seen.end()) {
      ++new_images;
    }
    if (task.image2_index != task.image1_index &&
        image_seen.find(task.image2_index) == image_seen.end()) {
      ++new_images;
    }

    if (!image_seen.empty() &&
        static_cast<int>(image_seen.size()) + new_images > image_block_size) {
      blocks.push_back({begin, i});
      begin = i;
      image_seen.clear();
    }
    image_seen[task.image1_index] = 1;
    image_seen[task.image2_index] = 1;
  }
  if (begin < static_cast<int>(pair_tasks.size())) {
    blocks.push_back({begin, static_cast<int>(pair_tasks.size())});
  }
  return blocks;
}

static bool accumulate_descriptor_mean(const FeatureData& features, std::vector<double>* sum,
                                       uint64_t* count) {
  constexpr int kDescriptorDim = 128;
  if (features.num_features == 0) {
    return false;
  }
  if (sum->size() != static_cast<size_t>(kDescriptorDim)) {
    sum->assign(kDescriptorDim, 0.0);
  }

  if (features.descriptor_type == DescriptorType::kUInt8) {
    if (features.descriptors_uint8.size() < features.num_features * kDescriptorDim) {
      return false;
    }
    for (size_t i = 0; i < features.num_features; ++i) {
      for (int d = 0; d < kDescriptorDim; ++d) {
        (*sum)[d] += static_cast<double>(features.descriptors_uint8[i * kDescriptorDim + d]);
      }
    }
  } else {
    if (features.descriptors_float.size() < features.num_features * kDescriptorDim) {
      return false;
    }
    for (size_t i = 0; i < features.num_features; ++i) {
      for (int d = 0; d < kDescriptorDim; ++d) {
        (*sum)[d] += static_cast<double>(features.descriptors_float[i * kDescriptorDim + d]);
      }
    }
  }
  *count += static_cast<uint64_t>(features.num_features);
  return true;
}

static bool accumulate_sample_mean_async(const std::vector<std::string>& sample_files, int num_threads,
                                         std::vector<double>* mean_sum, uint64_t* total_sample_features,
                                         int* valid_sample_images) {
  if (sample_files.empty() || num_threads <= 0) {
    return false;
  }

  constexpr int kDescriptorDim = 128;
  mean_sum->assign(kDescriptorDim, 0.0);
  *total_sample_features = 0;
  *valid_sample_images = 0;

  struct ThreadAccumulator {
    std::array<double, kDescriptorDim> sum{};
    uint64_t count = 0;
    int valid_images = 0;
  };
  std::vector<ThreadAccumulator> thread_accumulators(static_cast<size_t>(num_threads));

  const int queue_size = 16;
  Stage sample_stage("SampleMeanAccumulation", num_threads, queue_size,
                     [&sample_files, &thread_accumulators, num_threads, kDescriptorDim](int index) {
                       const int thread_slot = task_queue_context::current_worker_index();
                       if (thread_slot < 0 || thread_slot >= num_threads) {
                         LOG(FATAL) << "Invalid worker index from task_queue: " << thread_slot;
                       }
                       const FeatureData features = load_features_idc(sample_files[static_cast<size_t>(index)]);
                       std::vector<double> local_sum(kDescriptorDim, 0.0);
                       uint64_t local_count = 0;
                       if (!accumulate_descriptor_mean(features, &local_sum, &local_count)) {
                         return;
                       }

                       auto& acc = thread_accumulators[static_cast<size_t>(thread_slot)];
                       for (int d = 0; d < kDescriptorDim; ++d) {
                         acc.sum[static_cast<size_t>(d)] += local_sum[static_cast<size_t>(d)];
                       }
                       acc.count += local_count;
                       acc.valid_images += 1;
                     });
  sample_stage.setTaskCount(static_cast<int>(sample_files.size()));
  for (int i = 0; i < static_cast<int>(sample_files.size()); ++i) {
    sample_stage.push(i);
  }
  sample_stage.wait();

  for (const auto& acc : thread_accumulators) {
    for (int d = 0; d < kDescriptorDim; ++d) {
      (*mean_sum)[static_cast<size_t>(d)] += acc.sum[static_cast<size_t>(d)];
    }
    *total_sample_features += acc.count;
    *valid_sample_images += acc.valid_images;
  }
  return *total_sample_features > 0;
}

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  CmdLine cmd("InsightAT CPU Cascade Hashing Matcher");

  std::string pairs_json;
  std::string output_dir;
  std::string output_pairs_json;
  std::string feature_dir;
  int num_threads = -1;
  int sample_images = 256;
  int image_block_size = 1000;
  int hash_bits = 128;
  int bucket_groups = 6;
  int bucket_bits = 8;
  int candidate_top_min = 6;
  int candidate_top_max = 10;
  int min_match_list_len = 16;
  int min_output_matches = 16;
  float ratio_test = 0.8f;
  uint32_t random_seed = 1337;
  std::string preset = "modern";
  std::string log_level;

  cmd.add(make_option('i', pairs_json, "input").doc("Input pairs list (JSON format)"));
  cmd.add(make_option('o', output_dir, "output").doc("Output directory for .isat_match files"));
  cmd.add(make_option(0, output_pairs_json, "output-pairs-json")
              .doc("Optional JSON path listing only pairs whose .isat_match files were written."));
  cmd.add(make_option('f', feature_dir, "feature-dir")
              .doc("Matching feature directory (.isat_feat files from isat_extract -o)"));
  cmd.add(make_option('j', num_threads, "threads")
              .doc("Number of CPU threads (-1 = auto detect, default: -1)"));
  cmd.add(make_option(0, sample_images, "sample-images")
              .doc("Max unique images used to build global sample model (default: 256)"));
  cmd.add(make_option(0, image_block_size, "image-block-size")
              .doc("Max unique images per in-memory block (default: 1000)."));
  cmd.add(make_option(0, preset, "preset")
              .doc("Preset: legacy or modern (default: modern)"));
  cmd.add(make_option(0, hash_bits, "hash-bits").doc("Hash bit length (default: 128)"));
  cmd.add(make_option(0, bucket_groups, "bucket-groups").doc("Bucket groups L (default: 6)"));
  cmd.add(make_option(0, bucket_bits, "bucket-bits").doc("Bits per bucket group m (default: 8)"));
  cmd.add(make_option(0, candidate_top_min, "candidate-top-min")
              .doc("Minimum candidate count before stopping hamming expansion (default: 6)"));
  cmd.add(make_option(0, candidate_top_max, "candidate-top-max")
              .doc("Maximum candidates for L2 rerank (default: 10)"));
  cmd.add(make_option(0, min_match_list_len, "min-match-list-len")
              .doc("Minimum one-direction match list length (default: 16)"));
  cmd.add(make_option(0, min_output_matches, "min-output-matches")
              .doc("Drop pair if final matches < this threshold (default: 16)"));
  cmd.add(make_option('r', ratio_test, "ratio").doc("Ratio test threshold (default: 0.8)"));
  cmd.add(make_option(0, random_seed, "random-seed").doc("Random seed (default: 1337)"));
  cmd.add(make_option(0, log_level, "log-level").doc("Log level: error|warn|info|debug"));
  cmd.add(make_switch('v', "verbose").doc("Verbose logging (INFO level)"));
  cmd.add(make_switch('q', "quiet").doc("Quiet mode (ERROR level only)"));
  cmd.add(make_switch('h', "help").doc("Show this help message"));

  try {
    cmd.process(argc, argv);
  } catch (const std::string& s) {
    std::cerr << "Error: " << s << "\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }
  if (cmd.checkHelp(argv[0])) {
    return 0;
  }
  if (pairs_json.empty() || output_dir.empty()) {
    std::cerr << "Error: -i/--input and -o/--output are required\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }
  if (num_threads == -1) {
    const unsigned int hw = std::thread::hardware_concurrency();
    num_threads = static_cast<int>(hw > 0 ? hw : 4);
  } else if (num_threads <= 0) {
    LOG(ERROR) << "threads must be > 0, or -1 for auto";
    return 1;
  }
  LOG(INFO) << "Using " << num_threads << " worker threads";
  if (sample_images <= 0) {
    LOG(ERROR) << "sample-images must be > 0";
    return 1;
  }
  if (image_block_size <= 0) {
    LOG(ERROR) << "image-block-size must be > 0";
    return 1;
  }
  if (min_output_matches < 0) {
    LOG(ERROR) << "min-output-matches must be >= 0";
    return 1;
  }

  insight::tools::apply_log_level(cmd.used('v'), cmd.used('q'), log_level);

  std::vector<PairTask> pair_tasks = load_pairs_json(pairs_json, feature_dir);
  const int total_pairs = static_cast<int>(pair_tasks.size());
  if (total_pairs == 0) {
    print_event({{"type", "match.complete"}, {"ok", false}, {"error", "no pairs to process"}});
    return 1;
  }

  fs::create_directories(output_dir);

  CascadeHashOptions options;
  options.hash_bits = hash_bits;
  options.bucket_groups = bucket_groups;
  options.bucket_bits = bucket_bits;
  options.candidate_top_min = candidate_top_min;
  options.candidate_top_max = candidate_top_max;
  options.min_match_list_len = min_match_list_len;
  options.ratio_test = ratio_test;
  options.mutual_best = true;
  options.use_bucket_secondary_hash = true;
  options.random_seed = random_seed;

  if (preset == "legacy") {
    options.use_legacy_rng = true;
    options.use_legacy_numeric = true;
  } else if (preset == "modern") {
    options.use_legacy_rng = false;
    options.use_legacy_numeric = false;
  } else {
    LOG(ERROR) << "Invalid --preset: " << preset << " (expected legacy|modern)";
    return 1;
  }

  auto model_start = std::chrono::high_resolution_clock::now();
  std::vector<std::string> sample_files;
  if (!collect_sample_image_files(pair_tasks, static_cast<size_t>(sample_images), &sample_files)) {
    LOG(ERROR) << "Failed to collect sample image files";
    return 1;
  }
  // Stream sample files and accumulate descriptor mean with async I/O workers.
  std::vector<double> mean_sum(128, 0.0);
  uint64_t total_sample_features = 0;
  int valid_sample_images = 0;
  if (!accumulate_sample_mean_async(sample_files, num_threads, &mean_sum, &total_sample_features,
                                    &valid_sample_images)) {
    LOG(ERROR) << "No valid sample features for model building";
    return 1;
  }
  std::vector<float> mean_descriptor(128, 0.0f);
  for (int d = 0; d < 128; ++d) {
    mean_descriptor[d] = static_cast<float>(mean_sum[d] / static_cast<double>(total_sample_features));
  }

  const CascadeHashSampleModel model = build_sample_model_from_mean_descriptor(mean_descriptor, options);
  auto model_end = std::chrono::high_resolution_clock::now();
  const int model_ms = std::chrono::duration_cast<std::chrono::milliseconds>(model_end - model_start).count();
  LOG(INFO) << "Built sample model from " << valid_sample_images << " images and "
            << total_sample_features << " descriptors in " << model_ms << " ms";

  const int queue_size = 16;
  auto match_start = std::chrono::high_resolution_clock::now();
  int preload_ms_total = 0;
  int unique_images_total = 0;
  int block_count = 0;
  const std::vector<BlockRange> blocks = build_blocks_by_unique_images(pair_tasks, image_block_size);
  const int total_blocks = static_cast<int>(blocks.size());
  if (total_blocks == 0) {
    LOG(ERROR) << "No blocks generated from pair tasks";
    return 1;
  }

  auto preload_block = [&](int block_idx) -> BlockRuntimeData {
    const BlockRange range = blocks[static_cast<size_t>(block_idx)];
    const int block_begin = range.begin;
    const int block_end = range.end;
    const int block_pairs = block_end - block_begin;
    BlockRuntimeData runtime;
    runtime.block_begin = block_begin;
    runtime.block_end = block_end;

    std::unordered_map<uint32_t, int> image_to_cache_idx;
    image_to_cache_idx.reserve(static_cast<size_t>(block_pairs) * 2);
    runtime.image_cache.reserve(static_cast<size_t>(block_pairs) * 2);
    for (int i = block_begin; i < block_end; ++i) {
      auto& task = pair_tasks[static_cast<size_t>(i)];
      auto it1 = image_to_cache_idx.find(task.image1_index);
      if (it1 == image_to_cache_idx.end()) {
        const int idx = static_cast<int>(runtime.image_cache.size());
        image_to_cache_idx.emplace(task.image1_index, idx);
        runtime.image_cache.push_back(
            {task.image1_index, task.feature1_file, FeatureData(), ImageFeatures(), false});
        task.image1_cache_idx = idx;
      } else {
        task.image1_cache_idx = it1->second;
      }

      auto it2 = image_to_cache_idx.find(task.image2_index);
      if (it2 == image_to_cache_idx.end()) {
        const int idx = static_cast<int>(runtime.image_cache.size());
        image_to_cache_idx.emplace(task.image2_index, idx);
        runtime.image_cache.push_back(
            {task.image2_index, task.feature2_file, FeatureData(), ImageFeatures(), false});
        task.image2_cache_idx = idx;
      } else {
        task.image2_cache_idx = it2->second;
      }
    }

    auto preload_start = std::chrono::high_resolution_clock::now();
    Stage preload_stage("PreloadBlockImages", num_threads, queue_size,
                        [&runtime, &model](int index) {
                          auto& entry = runtime.image_cache[static_cast<size_t>(index)];
                          entry.features = load_features_idc(entry.feature_file);
                          if (entry.features.num_features == 0) {
                            entry.valid = false;
                            return;
                          }
                          entry.image_features = compute_image_features(entry.features, model);
                          entry.valid = true;
                        });
    preload_stage.setTaskCount(static_cast<int>(runtime.image_cache.size()));
    for (int i = 0; i < static_cast<int>(runtime.image_cache.size()); ++i) {
      preload_stage.push(i);
    }
    preload_stage.wait();
    auto preload_end = std::chrono::high_resolution_clock::now();
    runtime.preload_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(preload_end - preload_start).count();
    runtime.unique_images = static_cast<int>(runtime.image_cache.size());
    runtime.valid_images = 0;
    for (const auto& entry : runtime.image_cache) {
      if (entry.valid) {
        ++runtime.valid_images;
      }
    }
    return runtime;
  };

  std::future<BlockRuntimeData> preload_future =
      std::async(std::launch::async, preload_block, 0);

  for (int block_idx = 0; block_idx < total_blocks; ++block_idx) {
    ++block_count;
    BlockRuntimeData runtime = preload_future.get();
    const int block_begin = runtime.block_begin;
    const int block_end = runtime.block_end;
    const int block_pairs = block_end - block_begin;

    LOG(INFO) << "Block " << block_count << "/" << total_blocks
              << ": pair range [" << block_begin << ", " << block_end
              << "), pairs=" << block_pairs
              << ", unique_images=" << runtime.unique_images;
    LOG(INFO) << "Block " << block_count << "/" << total_blocks
              << ": preload finished in " << runtime.preload_ms
              << " ms, valid_images=" << runtime.valid_images;

    preload_ms_total += runtime.preload_ms;
    unique_images_total += runtime.unique_images;

    if (block_idx + 1 < total_blocks) {
      preload_future = std::async(std::launch::async, preload_block, block_idx + 1);
    }

    auto block_match_start = std::chrono::high_resolution_clock::now();
    Stage match_stage("CpuCascadeHashMatchBlock", num_threads, queue_size,
                      [&pair_tasks, &runtime, &model, block_begin](int local_index) {
                        const int global_index = block_begin + local_index;
                        auto& task = pair_tasks[static_cast<size_t>(global_index)];
                        const auto& left = runtime.image_cache[static_cast<size_t>(task.image1_cache_idx)];
                        const auto& right = runtime.image_cache[static_cast<size_t>(task.image2_cache_idx)];
                        if (!left.valid || !right.valid) {
                          return;
                        }
                        task.matches = match_cascade_hash(left.features, left.image_features, right.features,
                                                          right.image_features, model);
                        task.match_scales = build_scales_flat(task.matches, left.features, right.features);
                      });

    match_stage.setTaskCount(block_pairs);
    for (int i = 0; i < block_pairs; ++i) {
      match_stage.push(i);
    }
    match_stage.wait();
    auto block_match_end = std::chrono::high_resolution_clock::now();
    const int block_match_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(block_match_end - block_match_start).count();
    int block_pairs_with_matches = 0;
    int block_total_matches = 0;
    for (int i = block_begin; i < block_end; ++i) {
      const auto& task = pair_tasks[static_cast<size_t>(i)];
      if (task.matches.num_matches > 0) {
        ++block_pairs_with_matches;
        block_total_matches += static_cast<int>(task.matches.num_matches);
      }
    }
    LOG(INFO) << "Block " << block_count << "/" << total_blocks
              << ": matching finished in " << block_match_ms
              << " ms, pairs_with_matches=" << block_pairs_with_matches
              << ", total_matches=" << block_total_matches;
  }

  auto match_end = std::chrono::high_resolution_clock::now();
  const int match_time_s =
      std::chrono::duration_cast<std::chrono::seconds>(match_end - match_start).count();

  auto write_start = std::chrono::high_resolution_clock::now();
  std::mutex written_pairs_mu;
  std::vector<std::pair<uint32_t, uint32_t>> written_pairs;
  written_pairs.reserve(static_cast<size_t>(total_pairs));
  Stage write_stage("WriteAllResultsAtEnd", num_threads, queue_size,
                    [&pair_tasks, &output_dir, min_output_matches, &written_pairs_mu,
                     &written_pairs](int index) {
                      auto& task = pair_tasks[static_cast<size_t>(index)];
                      if (static_cast<int>(task.matches.num_matches) < min_output_matches) {
                        task.matches.clear();
                        task.match_scales.clear();
                        return;
                      }
                      if (write_match_idc(task.matches, task, output_dir)) {
                        std::lock_guard<std::mutex> lock(written_pairs_mu);
                        written_pairs.emplace_back(task.image1_index, task.image2_index);
                      }
                    });
  write_stage.setTaskCount(total_pairs);
  for (int i = 0; i < total_pairs; ++i) {
    write_stage.push(i);
  }
  write_stage.wait();
  auto write_end = std::chrono::high_resolution_clock::now();
  const int write_time_s =
      std::chrono::duration_cast<std::chrono::seconds>(write_end - write_start).count();
  const int total_time_s = match_time_s + write_time_s;

  int total_matches = 0;
  int pairs_with_matches = 0;
  for (const auto& task : pair_tasks) {
    if (task.matches.num_matches > 0) {
      ++pairs_with_matches;
      total_matches += static_cast<int>(task.matches.num_matches);
    }
  }
  const int failed_pairs = total_pairs - pairs_with_matches;

  if (!output_pairs_json.empty()) {
    if (!write_pairs_json(output_pairs_json, written_pairs)) {
      LOG(ERROR) << "Failed to write output pairs JSON: " << output_pairs_json;
      return 1;
    }
  }

  print_event({{"type", "match.complete"},
               {"ok", true},
               {"data",
                {{"total_pairs", total_pairs},
                 {"pairs_with_matches", pairs_with_matches},
                 {"total_matches", total_matches},
                 {"failed_pairs", failed_pairs},
                 {"total_time_s", total_time_s},
                 {"match_time_s", match_time_s},
                 {"write_time_s", write_time_s},
                 {"model_build_ms", model_ms},
                 {"preload_images_ms", preload_ms_total},
                 {"unique_images_total", unique_images_total},
                 {"image_block_size", image_block_size},
                 {"min_output_matches", min_output_matches},
                 {"blocks", block_count},
                 {"preset", preset},
                 {"output_dir", output_dir},
                 {"output_pairs_json", output_pairs_json}}}});
  return 0;
}
