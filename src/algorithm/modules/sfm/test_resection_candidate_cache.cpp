/**
 * @file  test_resection_candidate_cache.cpp
 * @brief Focused unit tests for choose_resection_candidates cache behavior.
 */

#include "resection_batch.h"

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

using insight::camera::Intrinsics;
using insight::sfm::ResectionCandidate;
using insight::sfm::ResectionScoreCache;
using insight::sfm::TrackStore;
using insight::sfm::choose_resection_candidates;

namespace {

int fail(const std::string& msg) {
  std::cerr << "  FAIL: " << msg << "\n";
  return 1;
}

Intrinsics make_test_intrinsics() {
  Intrinsics K;
  K.fx = 1000.0;
  K.fy = 1000.0;
  K.cx = 500.0;
  K.cy = 400.0;
  K.width = 1000;
  K.height = 800;
  return K;
}

void add_triangulated_obs(TrackStore* store, int image_index, uint32_t feature_id, float u, float v,
                          std::vector<int>* obs_ids_out) {
  const int tid = store->add_track(0.0f, 0.0f, 0.0f);
  const int obs_id = store->add_observation(tid, static_cast<uint32_t>(image_index), feature_id,
                                            u, v, 1.0f);
  store->set_track_xyz(tid, static_cast<float>(feature_id), 1.0f, 5.0f);
  if (obs_ids_out)
    obs_ids_out->push_back(obs_id);
}

bool same_candidates(const std::vector<ResectionCandidate>& a,
                     const std::vector<ResectionCandidate>& b) {
  if (a.size() != b.size())
    return false;
  for (size_t i = 0; i < a.size(); ++i) {
    if (a[i].image_index != b[i].image_index || a[i].num_3d2d != b[i].num_3d2d ||
        a[i].coverage != b[i].coverage) {
      return false;
    }
  }
  return true;
}

int test_snapshot_hit_and_partial_refresh() {
  std::cout << "[test1] snapshot hit + dirty-image partial refresh\n";

  TrackStore store;
  store.set_num_images(3);
  std::vector<int> image1_obs_ids;

  add_triangulated_obs(&store, 1, 10u, 100.0f, 100.0f, &image1_obs_ids);
  add_triangulated_obs(&store, 1, 11u, 900.0f, 100.0f, &image1_obs_ids);
  add_triangulated_obs(&store, 1, 12u, 100.0f, 700.0f, &image1_obs_ids);
  add_triangulated_obs(&store, 1, 13u, 900.0f, 700.0f, &image1_obs_ids);
  add_triangulated_obs(&store, 1, 14u, 500.0f, 400.0f, &image1_obs_ids);
  add_triangulated_obs(&store, 1, 15u, 250.0f, 300.0f, &image1_obs_ids);

  add_triangulated_obs(&store, 2, 20u, 120.0f, 120.0f, nullptr);
  add_triangulated_obs(&store, 2, 21u, 880.0f, 120.0f, nullptr);
  add_triangulated_obs(&store, 2, 22u, 120.0f, 680.0f, nullptr);
  add_triangulated_obs(&store, 2, 23u, 880.0f, 680.0f, nullptr);

  store.clear_dirty_sets();

  std::vector<bool> registered = {true, false, false};
  std::vector<Intrinsics> cameras = {make_test_intrinsics()};
  std::vector<int> image_to_camera_index = {0, 0, 0};
  ResectionScoreCache cache;

  const auto first = choose_resection_candidates(store, registered, cameras, image_to_camera_index,
                                                 4, 10, 0.0f, 4, &cache);
  if (first.size() != 2)
    return fail("first choose_resection_candidates call must return two candidates");
  if (first[0].image_index != 1 || first[0].num_3d2d != 6)
    return fail("image 1 should be the best initial candidate with 6 correspondences");

  const auto second = choose_resection_candidates(store, registered, cameras, image_to_camera_index,
                                                  4, 10, 0.0f, 4, &cache);
  if (!same_candidates(first, second))
    return fail("snapshot cache hit must return the same candidate snapshot");

  const int image1_n_tri_before = cache.entries[1].n_tri;
  const int image2_n_tri_before = cache.entries[2].n_tri;
  store.mark_observation_deleted(image1_obs_ids.front());

  const auto third = choose_resection_candidates(store, registered, cameras, image_to_camera_index,
                                                 4, 10, 0.0f, 4, &cache);
  if (third.size() != 2)
    return fail("dirty-image refresh must still produce two candidates");
  if (cache.entries[1].n_tri != image1_n_tri_before - 1)
    return fail("dirty-image refresh must update only the modified image count");
  if (cache.entries[2].n_tri != image2_n_tri_before)
    return fail("dirty-image refresh must preserve untouched image cache entries");

  std::vector<int> dirty_images;
  if (store.consume_dirty_images(&dirty_images) != 0)
    return fail("choose_resection_candidates must consume dirty image ids during partial refresh");

  std::cout << "  PASS\n";
  return 0;
}

int test_parameter_only_rerank() {
  std::cout << "[test2] parameter-only rerank reuses per-image cache\n";

  TrackStore store;
  store.set_num_images(3);
  add_triangulated_obs(&store, 1, 30u, 100.0f, 100.0f, nullptr);
  add_triangulated_obs(&store, 1, 31u, 900.0f, 100.0f, nullptr);
  add_triangulated_obs(&store, 1, 32u, 100.0f, 700.0f, nullptr);
  add_triangulated_obs(&store, 1, 33u, 900.0f, 700.0f, nullptr);
  add_triangulated_obs(&store, 1, 34u, 500.0f, 400.0f, nullptr);

  add_triangulated_obs(&store, 2, 40u, 150.0f, 150.0f, nullptr);
  add_triangulated_obs(&store, 2, 41u, 850.0f, 150.0f, nullptr);
  add_triangulated_obs(&store, 2, 42u, 150.0f, 650.0f, nullptr);
  add_triangulated_obs(&store, 2, 43u, 850.0f, 650.0f, nullptr);

  store.clear_dirty_sets();

  std::vector<bool> registered = {true, false, false};
  std::vector<Intrinsics> cameras = {make_test_intrinsics()};
  std::vector<int> image_to_camera_index = {0, 0, 0};
  ResectionScoreCache cache;

  const auto full = choose_resection_candidates(store, registered, cameras, image_to_camera_index,
                                                4, 10, 0.0f, 4, &cache);
  if (full.size() != 2)
    return fail("parameter rerank setup must produce two candidates");
  const int image1_n_tri = cache.entries[1].n_tri;
  const int image2_n_tri = cache.entries[2].n_tri;
  const float image1_cov = cache.entries[1].score;
  const float image2_cov = cache.entries[2].score;

  const auto limited = choose_resection_candidates(store, registered, cameras, image_to_camera_index,
                                                   4, 1, 0.0f, 4, &cache);
  if (limited.size() != 1)
    return fail("max_candidates change must rerank down to one candidate");
  if (limited[0].image_index != full[0].image_index)
    return fail("parameter-only rerank must preserve best candidate ordering");
  if (cache.entries[1].n_tri != image1_n_tri || cache.entries[2].n_tri != image2_n_tri ||
      cache.entries[1].score != image1_cov || cache.entries[2].score != image2_cov) {
    return fail("parameter-only rerank must reuse cached per-image entries");
  }

  std::cout << "  PASS\n";
  return 0;
}

} // namespace

int main() {
  int failures = 0;
  failures += test_snapshot_hit_and_partial_refresh();
  failures += test_parameter_only_rerank();
  if (failures == 0)
    std::cout << "\nAll tests PASSED.\n";
  return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}