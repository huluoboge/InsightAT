/**
 * @file  test_track_store_state_cache.cpp
 * @brief Unit tests for TrackStore incremental state helpers (epoch/dirty/retri queue).
 */

#include "track_store.h"

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

using insight::sfm::TrackStore;

namespace {

int fail(const std::string& msg) {
  std::cerr << "  FAIL: " << msg << "\n";
  return 1;
}

int test_epoch_and_dirty_sets() {
  std::cout << "[test1] epoch + dirty-set semantics\n";

  TrackStore s;
  s.set_num_images(3);

  if (s.obs_epoch() != 0 || s.xyz_epoch() != 0 || s.registration_epoch() != 0)
    return fail("initial epochs must be zero");

  const int t0 = s.add_track(1.0f, 2.0f, 3.0f);
  if (t0 != 0)
    return fail("first track id must be 0");
  if (s.registration_epoch() != 1)
    return fail("registration_epoch must bump on add_track");

  const int o0 = s.add_observation(t0, 0u, 10u, 100.0f, 120.0f, 1.0f);
  if (o0 != 0)
    return fail("first observation id must be 0");
  if (s.obs_epoch() != 1 || s.registration_epoch() != 2)
    return fail("obs/registration epochs must bump on add_observation");

  s.set_track_xyz(t0, 5.0f, 6.0f, 7.0f);
  if (s.xyz_epoch() != 1)
    return fail("xyz_epoch must bump on first set_track_xyz");

  // Same XYZ should not bump epoch.
  s.set_track_xyz(t0, 5.0f, 6.0f, 7.0f);
  if (s.xyz_epoch() != 1)
    return fail("xyz_epoch must not bump when xyz unchanged");

  std::vector<int> dirty_tracks;
  std::vector<int> dirty_images;
  const int n_dt = s.consume_dirty_tracks(&dirty_tracks);
  const int n_di = s.consume_dirty_images(&dirty_images);
  if (n_dt != 1 || n_di != 1)
    return fail("dirty sets should dedupe to one track and one image");
  if (dirty_tracks[0] != 0 || dirty_images[0] != 0)
    return fail("dirty ids mismatch");

  // Re-consume after clear should return empty.
  dirty_tracks.clear();
  dirty_images.clear();
  if (s.consume_dirty_tracks(&dirty_tracks) != 0 || s.consume_dirty_images(&dirty_images) != 0)
    return fail("dirty sets must be empty after consume");

  // Deleting the same observation twice should only bump once.
  s.mark_observation_deleted(o0);
  const uint64_t obs_e_after_del = s.obs_epoch();
  const uint64_t reg_e_after_del = s.registration_epoch();
  s.mark_observation_deleted(o0);
  if (s.obs_epoch() != obs_e_after_del || s.registration_epoch() != reg_e_after_del)
    return fail("duplicate delete must be idempotent for epochs");

  s.mark_observation_restored(o0);
  if (s.obs_epoch() != obs_e_after_del + 1)
    return fail("restore must bump obs_epoch");

  s.clear_track_xyz(t0);
  if (s.xyz_epoch() != 2)
    return fail("clear_track_xyz must bump xyz_epoch when had XYZ");

  std::cout << "  PASS\n";
  return 0;
}

int test_retri_pending_dedupe() {
  std::cout << "[test2] retriangulation pending dedupe\n";

  TrackStore s;
  s.set_num_images(2);
  const int t0 = s.add_track(0.f, 0.f, 1.f);

  s.set_track_retriangulation_flag(t0, true);
  s.set_track_retriangulation_flag(t0, true);

  std::vector<int> pending;
  s.drain_retriangulation_pending(&pending);
  if (pending.size() != 1 || pending[0] != t0)
    return fail("pending queue must dedupe repeated set(true)");

  // Toggle false then true should enqueue again once.
  s.set_track_retriangulation_flag(t0, false);
  s.set_track_retriangulation_flag(t0, true);
  s.set_track_retriangulation_flag(t0, true);
  pending.clear();
  s.drain_retriangulation_pending(&pending);
  if (pending.size() != 1 || pending[0] != t0)
    return fail("pending queue re-enqueue semantics broken");

  std::cout << "  PASS\n";
  return 0;
}

int test_observation_views_and_scalar_accessors() {
  std::cout << "[test3] observation views + scalar accessors\n";

  TrackStore s;
  s.set_num_images(2);
  const int t0 = s.add_track(1.f, 2.f, 3.f);
  const int t1 = s.add_track(4.f, 5.f, 6.f);
  const int o0 = s.add_observation(t0, 1u, 11u, 120.5f, 240.25f, 1.5f);
  const int o1 = s.add_observation(t1, 1u, 12u, 121.5f, 241.25f, 2.5f);

  const auto& image_view = s.image_all_obs_ids_view(1);
  if (image_view.size() != 2)
    return fail("image_all_obs_ids_view must expose structural observation ids");
  if (image_view[0] != o0 || image_view[1] != o1)
    return fail("image_all_obs_ids_view order mismatch");

  const auto& track_view = s.track_all_obs_ids_view(t0);
  if (track_view.size() != 1 || track_view[0] != o0)
    return fail("track_all_obs_ids_view mismatch");

  if (s.obs_image_index(o0) != 1u || s.obs_feature_id(o0) != 11u)
    return fail("scalar observation integer accessors mismatch");
  if (s.obs_u(o0) != 120.5f || s.obs_v(o0) != 240.25f || s.obs_scale(o0) != 1.5f)
    return fail("scalar observation float accessors mismatch");

  s.mark_observation_deleted(o0);
  const auto& image_view_after_delete = s.image_all_obs_ids_view(1);
  if (image_view_after_delete.size() != 2)
    return fail("structural image view must keep deleted observations");
  if (s.is_obs_valid(o0))
    return fail("deleted observation must be invalid");

  std::cout << "  PASS\n";
  return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// test4: global_pose_epoch + is_track_tri_stale
// ─────────────────────────────────────────────────────────────────────────────
static int test_pose_epoch_and_tri_stale() {
  std::cout << "[test4] pose epoch + is_track_tri_stale\n";

  TrackStore s;
  s.set_num_images(2);
  const int t0 = s.add_track(1.f, 2.f, 3.f);
  /*const int o0 =*/ s.add_observation(t0, 0, 0, 10.f, 20.f);
  /*const int o1 =*/ s.add_observation(t0, 1, 1, 15.f, 25.f);

  // Freshly added track has never been triangulated → stale.
  if (!s.is_track_tri_stale(t0))
    return fail("new track must be stale before any set_track_xyz");

  // After set_track_xyz, the track is recorded at the current pose epoch.
  const uint64_t ep0 = s.global_pose_epoch();
  s.set_track_xyz(t0, 1.f, 2.f, 3.f);
  if (s.is_track_tri_stale(t0))
    return fail("track must not be stale after set_track_xyz at same epoch");

  // Pure XYZ update (no flag change) must also refresh the epoch stamp.
  s.set_track_xyz(t0, 4.f, 5.f, 6.f);
  if (s.is_track_tri_stale(t0))
    return fail("track must not be stale after pure xyz update");

  // Bump pose epoch → track becomes stale.
  s.bump_global_pose_epoch();
  if (s.global_pose_epoch() != ep0 + 1)
    return fail("bump_global_pose_epoch did not increment");
  if (!s.is_track_tri_stale(t0))
    return fail("track must be stale after bump_global_pose_epoch");

  // Re-triangulate → fresh again.
  s.set_track_xyz(t0, 7.f, 8.f, 9.f);
  if (s.is_track_tri_stale(t0))
    return fail("track must not be stale after re-triangulation");

  // clear_track_xyz → stale again.
  s.clear_track_xyz(t0);
  if (!s.is_track_tri_stale(t0))
    return fail("track must be stale after clear_track_xyz");

  std::cout << "  PASS\n";
  return 0;
}

} // namespace

int main() {
  int failures = 0;
  failures += test_epoch_and_dirty_sets();
  failures += test_retri_pending_dedupe();
  failures += test_observation_views_and_scalar_accessors();
  failures += test_pose_epoch_and_tri_stale();
  if (failures == 0)
    std::cout << "\nAll tests PASSED.\n";
  return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
