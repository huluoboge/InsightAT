/**
 * @file  test_sfm_diagnosis.cpp
 * @brief Post-SfM diagnostic tool: analyze why images fail to register.
 *
 * Workflow
 * ────────
 *   1. Load .isat_tracks IDC  → TrackStore (no XYZ) + embedded ViewGraph
 *   2. Load images_all.json   → ProjectData (cameras, image_to_camera_index)
 *   3. Load poses.json        → registered[] + poses_R[] + poses_C[]
 *   4. Run run_full_scan_triangulation to re-triangulate ALL pending tracks
 *      using the saved poses (no RANSAC truncation on registered observations).
 *   5. For each unregistered image:
 *      - n_3d2d_current: # triangulated tracks seen in that image
 *      - n_tracks_total: all track observations in that image
 *      - registered_neighbors: images sharing a ViewGraph edge that ARE registered,
 *        with their F_inliers, E_ok, twoview_ok
 *   6. Run choose_resection_candidates with the post-triangulation store to see
 *      which unregistered images would now be resectable.
 *   7. Write JSON report to -o  (stdout if omitted).
 *   8. Optionally save triangulated TrackStore IDC with --save-tracks.
 *
 * Diagnostic hypothesis tested
 * ─────────────────────────────
 *   If n_3d2d_after_tri ≥ min_3d2d but the pipeline stopped before reaching
 *   this image, the problem is triangulation lag (re-triangulation was not
 *   aggressive enough during the SfM run).
 *   If n_3d2d_after_tri < min_3d2d AND ViewGraph neighbors are sparse/weak,
 *   the problem is upstream matching (retrieval or matching missed this image).
 *
 * Usage
 * ─────
 *   test_sfm_diagnosis \
 *       -t /path/to/tracks.isat_tracks \
 *       -p /path/to/images_all.json    \
 *       -s /path/to/poses.json         \
 *       [-o report.json]               \
 *       [--save-tracks out.isat_tracks] \
 *       [--min-3d2d 20]                \
 *       [--min-tri-angle 0.5]          \
 *       [-v | -q]
 */

#include "../../io/track_store_idc.h"
#include "../../tools/project_loader.h"
#include "incremental_triangulation.h"
#include "resection_batch.h"
#include "track_store.h"
#include "view_graph.h"
#include "view_graph_loader.h"

#include <Eigen/Dense>
#include <glog/logging.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace fs = std::filesystem;
using json = nlohmann::json;
using namespace insight::sfm;
using insight::tools::ProjectData;
using insight::tools::load_project_data;
using CameraIntrinsics = insight::camera::Intrinsics;
using Eigen::Matrix3d;
using Eigen::Vector3d;

// ─────────────────────────────────────────────────────────────────────────────
// Pose loading
// ─────────────────────────────────────────────────────────────────────────────

struct LoadedPoses {
    int n_images = 0;
    std::vector<bool> registered;
    std::vector<Matrix3d> poses_R;
    std::vector<Vector3d> poses_C;
    std::vector<CameraIntrinsics> cameras;
    std::vector<int> image_to_camera_index;
    bool has_camera_bundle = false;
};

static bool parse_intrinsics_json(const json& j, CameraIntrinsics* K) {
    if (!K)
        return false;
    if (!j.is_object())
        return false;
    K->fx = j.value("fx", 0.0);
    K->fy = j.value("fy", 0.0);
    K->cx = j.value("cx", 0.0);
    K->cy = j.value("cy", 0.0);
    K->width = j.value("width", 0);
    K->height = j.value("height", 0);
    K->k1 = j.value("k1", 0.0);
    K->k2 = j.value("k2", 0.0);
    K->k3 = j.value("k3", 0.0);
    K->p1 = j.value("p1", 0.0);
    K->p2 = j.value("p2", 0.0);
    return K->fx > 0.0 && K->fy > 0.0;
}

/// Load poses.json written by isat_incremental_sfm.
/// Supports both legacy array format [{image_index,R,C}, ...] and v2 object format
/// {format, poses:[...], cameras:[...], image_to_camera_index:[...]}.
/// n_images is taken from the track store (already loaded).
static bool load_poses_json(const std::string& path, int n_images, LoadedPoses* out) {
    std::ifstream f(path);
    if (!f.is_open()) {
        LOG(ERROR) << "Cannot open poses file: " << path;
        return false;
    }
    json j;
    try {
        f >> j;
    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to parse poses JSON: " << e.what();
        return false;
    }
    const json* poses_arr = nullptr;
    if (j.is_array()) {
        poses_arr = &j;
    } else if (j.is_object() && j.contains("poses") && j["poses"].is_array()) {
        poses_arr = &j["poses"];
        if (j.contains("cameras") && j["cameras"].is_array()) {
            out->cameras.clear();
            out->cameras.reserve(j["cameras"].size());
            for (const auto& cj : j["cameras"]) {
                CameraIntrinsics K;
                if (!parse_intrinsics_json(cj, &K)) {
                    LOG(WARNING) << "poses.json: invalid camera intrinsics entry";
                    out->cameras.clear();
                    break;
                }
                out->cameras.push_back(K);
            }
        }
        if (j.contains("image_to_camera_index") && j["image_to_camera_index"].is_array()) {
            try {
                out->image_to_camera_index = j["image_to_camera_index"].get<std::vector<int>>();
            } catch (const std::exception& e) {
                LOG(WARNING) << "poses.json: failed to parse image_to_camera_index: " << e.what();
                out->image_to_camera_index.clear();
            }
        }
        out->has_camera_bundle = !out->cameras.empty() &&
                                 static_cast<int>(out->image_to_camera_index.size()) == n_images;
    } else {
        LOG(ERROR) << "poses.json: expected array or object with poses[] at top level";
        return false;
    }

    out->n_images = n_images;
    out->registered.assign(static_cast<size_t>(n_images), false);
    out->poses_R.resize(static_cast<size_t>(n_images), Matrix3d::Identity());
    out->poses_C.resize(static_cast<size_t>(n_images), Vector3d::Zero());

    int loaded = 0;
    for (const auto& entry : *poses_arr) {
        const int idx = entry.at("image_index").get<int>();
        if (idx < 0 || idx >= n_images) {
            LOG(WARNING) << "poses.json: image_index=" << idx
                         << " out of range [0," << n_images << ")";
            continue;
        }
        const auto rv = entry.at("R").get<std::vector<double>>();
        const auto cv = entry.at("C").get<std::vector<double>>();
        if (rv.size() != 9 || cv.size() != 3) {
            LOG(WARNING) << "poses.json: image " << idx << " invalid R/C size";
            continue;
        }
        Matrix3d R;
        R << rv[0], rv[1], rv[2], rv[3], rv[4], rv[5], rv[6], rv[7], rv[8];
        Vector3d C(cv[0], cv[1], cv[2]);
        out->poses_R[static_cast<size_t>(idx)] = R;
        out->poses_C[static_cast<size_t>(idx)] = C;
        out->registered[static_cast<size_t>(idx)] = true;
        ++loaded;
    }
    LOG(INFO) << "Loaded " << loaded << " registered poses from " << path;
    if (out->has_camera_bundle) {
        LOG(INFO) << "Loaded final camera bundle from poses.json: cameras="
                  << out->cameras.size();
    }
    return loaded > 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Per-image covisibility summary from ViewGraph
// ─────────────────────────────────────────────────────────────────────────────

struct NeighborSummary {
    int neighbor_image_index = -1;
    bool neighbor_registered = false;
    int f_inliers = 0;
    bool e_ok = false;
    bool twoview_ok = false;
    bool stable = false;
};

/// Build a map: image_index → list of ViewGraph neighbors.
static std::unordered_map<int, std::vector<NeighborSummary>>
build_neighbor_map(const ViewGraph& vg, const std::vector<bool>& registered) {
    std::unordered_map<int, std::vector<NeighborSummary>> map;
    for (size_t pi = 0; pi < vg.num_pairs(); ++pi) {
        const PairGeoInfo& p = vg.pair_at(pi);
        const int i1 = static_cast<int>(p.image1_index);
        const int i2 = static_cast<int>(p.image2_index);
        const bool r1 = (i1 < static_cast<int>(registered.size())) && registered[i1];
        const bool r2 = (i2 < static_cast<int>(registered.size())) && registered[i2];

        NeighborSummary n1, n2;
        n1.neighbor_image_index = i1;
        n1.neighbor_registered  = r1;
        n1.f_inliers            = p.F_inliers;
        n1.e_ok                 = p.E_ok;
        n1.twoview_ok           = p.twoview_ok;
        n1.stable               = p.stable;

        n2.neighbor_image_index = i2;
        n2.neighbor_registered  = r2;
        n2.f_inliers            = p.F_inliers;
        n2.e_ok                 = p.E_ok;
        n2.twoview_ok           = p.twoview_ok;
        n2.stable               = p.stable;

        map[i2].push_back(n1);  // i2 sees i1 as neighbor
        map[i1].push_back(n2);  // i1 sees i2 as neighbor
    }
    return map;
}

// ─────────────────────────────────────────────────────────────────────────────
// 3D-2D count for one image
// ─────────────────────────────────────────────────────────────────────────────

static int count_3d2d(const TrackStore& store, int image_index) {
    std::vector<int> track_ids;
    std::vector<Observation> obs_scratch;
    store.get_image_track_observations(image_index, &track_ids, &obs_scratch);
    int count = 0;
    for (int tid : track_ids)
        if (store.track_has_triangulated_xyz(tid))
            ++count;
    return count;
}

static int count_total_tracks(const TrackStore& store, int image_index) {
    std::vector<int> track_ids;
    std::vector<Observation> obs_scratch;
    return store.get_image_track_observations(image_index, &track_ids, &obs_scratch);
}

// ─────────────────────────────────────────────────────────────────────────────
// Bundler format export
// ─────────────────────────────────────────────────────────────────────────────

/// Load image file paths from the images_all.json used by load_project_data.
/// Returns vector of paths in image-index order (empty string if missing).
static std::vector<std::string> load_image_paths(const std::string& project_json_path, int n_images) {
    std::vector<std::string> paths(static_cast<size_t>(n_images));
    std::ifstream f(project_json_path);
    if (!f.is_open()) return paths;
    nlohmann::json j;
    try { f >> j; } catch (...) { return paths; }
    if (!j.contains("images") || !j["images"].is_array()) return paths;
    const auto& arr = j["images"];
    const size_t n = std::min(arr.size(), static_cast<size_t>(n_images));
    for (size_t i = 0; i < n; ++i)
        paths[i] = arr[i].value("path", "");
    return paths;
}

/**
 * Write a Bundler v0.3 bundle.out file and a companion list.txt.
 *
 * Camera convention:
 *   - All n_images cameras written in order (unregistered get f=0, R=I, t=0).
 *   - cam_idx in the point view-list == image_index, so no re-mapping needed.
 *   - t = -R * C  (world-to-camera translation)
 *   - Image coords: x = u - cx,  y = -(v - cy)  (center-origin, y-up)
 *
 * list.txt (written alongside bundle.out) contains image paths in order.
 */
static bool save_bundler_format(
    const std::string& bundle_path,
    const TrackStore& store,
    int n_images,
    const std::vector<bool>& registered,
    const std::vector<Matrix3d>& poses_R,
    const std::vector<Vector3d>& poses_C,
    const std::vector<CameraIntrinsics>& cameras,
    const std::vector<int>& image_to_camera_index,
    const std::vector<std::string>& image_paths)
{
    // ── Collect triangulated points ───────────────────────────────────────────
    struct BundlerObs { int cam_idx; double bx, by; };
    struct BundlerPoint {
        double X, Y, Z;
        std::vector<BundlerObs> obs;
    };

    std::vector<BundlerPoint> points;
    const int n_tracks = store.num_tracks();
    points.reserve(static_cast<size_t>(n_tracks / 2));

    for (int tid = 0; tid < n_tracks; ++tid) {
        if (!store.is_track_valid(tid) || !store.track_has_triangulated_xyz(tid))
            continue;
        float tx, ty, tz;
        store.get_track_xyz(tid, &tx, &ty, &tz);

        BundlerPoint pt;
        pt.X = static_cast<double>(tx);
        pt.Y = static_cast<double>(ty);
        pt.Z = static_cast<double>(tz);

        std::vector<int> obs_ids;
        store.get_track_obs_ids(tid, &obs_ids);
        for (int oid : obs_ids) {
            if (!store.is_obs_valid(oid))
                continue;
            Observation o;
            store.get_obs(oid, &o);
            const int im = static_cast<int>(o.image_index);
            if (im < 0 || im >= n_images || !registered[static_cast<size_t>(im)])
                continue;
            const CameraIntrinsics& K =
                cameras[static_cast<size_t>(image_to_camera_index[static_cast<size_t>(im)])];
            const double bx = static_cast<double>(o.u) - K.cx;
            const double by = -(static_cast<double>(o.v) - K.cy);  // y-up
            pt.obs.push_back({im, bx, by});
        }
        if (pt.obs.size() >= 2)
            points.push_back(std::move(pt));
    }

    // ── Write bundle.out ──────────────────────────────────────────────────────
    std::ofstream f(bundle_path);
    if (!f.is_open()) {
        LOG(ERROR) << "save_bundler_format: cannot open " << bundle_path;
        return false;
    }

    f << "# Bundle file v0.3\n";
    f << n_images << " " << points.size() << "\n";

    f << std::fixed << std::setprecision(10);
    for (int i = 0; i < n_images; ++i) {
        if (!registered[static_cast<size_t>(i)]) {
            // Unregistered: write a zeroed camera so index parity is preserved.
            f << "0 0 0\n1 0 0\n0 1 0\n0 0 1\n0 0 0\n";
            continue;
        }
        const CameraIntrinsics& K =
            cameras[static_cast<size_t>(image_to_camera_index[static_cast<size_t>(i)])];
        const double focal = (K.fx + K.fy) * 0.5;
        const Matrix3d& R = poses_R[static_cast<size_t>(i)];
        const Vector3d  t = -R * poses_C[static_cast<size_t>(i)];
        f << focal << " " << K.k1 << " " << K.k2 << "\n";
        f << R(0,0) << " " << R(0,1) << " " << R(0,2) << "\n";
        f << R(1,0) << " " << R(1,1) << " " << R(1,2) << "\n";
        f << R(2,0) << " " << R(2,1) << " " << R(2,2) << "\n";
        f << t(0)   << " " << t(1)   << " " << t(2)   << "\n";
    }

    f << std::fixed << std::setprecision(6);
    for (const auto& pt : points) {
        f << pt.X << " " << pt.Y << " " << pt.Z << "\n";
        f << "255 255 255\n";  // color unknown
        f << pt.obs.size();
        for (const auto& o : pt.obs)
            f << " " << o.cam_idx << " 0 " << o.bx << " " << o.by;
        f << "\n";
    }

    LOG(INFO) << "Saved Bundler file: " << bundle_path
              << "  cameras=" << n_images
              << "  points=" << points.size();

    // ── Write companion list.txt ──────────────────────────────────────────────
    if (!image_paths.empty()) {
        const fs::path list_path =
            fs::path(bundle_path).parent_path() / "list.txt";
        std::ofstream lf(list_path);
        if (lf.is_open()) {
            for (const auto& p : image_paths)
                lf << p << "\n";
            LOG(INFO) << "Saved Bundler list.txt: " << list_path.string();
        } else {
            LOG(WARNING) << "Cannot write list.txt alongside bundle.out";
        }
    }

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    google::SetStderrLogging(google::INFO);

    // ── Arg parsing (simple hand-rolled) ─────────────────────────────────────
    std::string tracks_path, project_path, poses_path, output_path, save_tracks_path, save_bundler_path;
    int min_3d2d = 20;
    double min_tri_angle_deg = 0.5;
    bool verbose = false;
    bool quiet = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << a << "\n";
                exit(2);
            }
            return argv[++i];
        };
        if (a == "-t" || a == "--tracks")        tracks_path        = next();
        else if (a == "-p" || a == "--project")  project_path       = next();
        else if (a == "-s" || a == "--poses")    poses_path         = next();
        else if (a == "-o" || a == "--output")   output_path        = next();
        else if (a == "--save-tracks")           save_tracks_path   = next();
        else if (a == "--save-bundler")          save_bundler_path  = next();
        else if (a == "--min-3d2d")              min_3d2d           = std::stoi(next());
        else if (a == "--min-tri-angle")         min_tri_angle_deg  = std::stod(next());
        else if (a == "-v" || a == "--verbose")  verbose = true;
        else if (a == "-q" || a == "--quiet")    quiet   = true;
        else if (a == "-h" || a == "--help") {
            std::cout <<
                "test_sfm_diagnosis: post-SfM diagnostic for unregistered images\n\n"
                "  -t / --tracks    <path>   .isat_tracks IDC (required)\n"
                "  -p / --project   <path>   images_all.json (required)\n"
                "  -s / --poses     <path>   poses.json from SfM run (required)\n"
                "  -o / --output    <path>   JSON report output (default: stdout)\n"
                "  --save-tracks    <path>   save triangulated TrackStore IDC\n"
                "  --save-bundler   <path>   export Bundler v0.3 bundle.out + list.txt\n"
                "  --min-3d2d       <N>      min 3D-2D threshold (default: 20)\n"
                "  --min-tri-angle  <deg>    min triangulation angle (default: 0.5)\n"
                "  -v               verbose log\n"
                "  -q               quiet (error only)\n";
            return 0;
        } else {
            std::cerr << "Unknown argument: " << a << "\n";
            return 2;
        }
    }

    if (tracks_path.empty() || project_path.empty() || poses_path.empty()) {
        std::cerr << "Error: -t, -p, -s are required\n";
        return 2;
    }

    if (quiet)        google::SetStderrLogging(google::ERROR);
    else if (verbose) google::SetStderrLogging(google::INFO);

    // ── 1. Load TrackStore + ViewGraph ────────────────────────────────────────
    TrackStore store;
    std::vector<uint32_t> image_indices_from_idc;
    ViewGraph vg;
    LOG(INFO) << "Loading tracks: " << tracks_path;
    if (!load_track_store_from_idc(tracks_path, &store, &image_indices_from_idc, &vg)) {
        LOG(ERROR) << "Failed to load tracks";
        return 1;
    }
    const int n_images = store.num_images();
    const size_t n_tracks = store.num_tracks();
    LOG(INFO) << "TrackStore: " << n_images << " images, " << n_tracks
              << " tracks, ViewGraph pairs: " << vg.num_pairs();

    // ── 2. Load ProjectData (cameras + image_to_camera_index) ────────────────
    ProjectData project;
    LOG(INFO) << "Loading project: " << project_path;
    if (!load_project_data(project_path, &project)) {
        LOG(ERROR) << "Failed to load project from " << project_path;
        return 1;
    }
    if (project.num_images() != n_images) {
        LOG(ERROR) << "Image count mismatch: project=" << project.num_images()
                   << " tracks=" << n_images;
        return 1;
    }

    // ── 3. Load Poses ─────────────────────────────────────────────────────────
    LoadedPoses poses;
    LOG(INFO) << "Loading poses: " << poses_path;
    if (!load_poses_json(poses_path, n_images, &poses)) {
        LOG(ERROR) << "Failed to load poses from " << poses_path;
        return 1;
    }
    if (poses.has_camera_bundle) {
        LOG(INFO)<<"poses.json contains camera bundle; overriding project cameras with poses.json bundle";
        if (poses.image_to_camera_index != project.image_to_camera_index) {
            LOG(WARNING) << "poses.json image_to_camera_index differs from project; using poses.json bundle";
        }
        project.cameras = poses.cameras;
        project.image_to_camera_index = poses.image_to_camera_index;
    }
    const int n_registered   = static_cast<int>(
        std::count(poses.registered.begin(), poses.registered.end(), true));
    const int n_unregistered = n_images - n_registered;
    LOG(INFO) << "Registered: " << n_registered << "/" << n_images
              << "  Unregistered: " << n_unregistered;

    // ── 4. Re-triangulate ALL tracks using saved poses ────────────────────────
    LOG(INFO) << "Running full-scan triangulation (min_angle=" << min_tri_angle_deg << " deg) ...";
    const int newly_tri = run_full_scan_triangulation(
        &store, poses.poses_R, poses.poses_C, poses.registered,
        project.cameras, project.image_to_camera_index,
        min_tri_angle_deg);
    const int total_tri = static_cast<int>(
        [&]() {
            int c = 0;
            for (size_t tid = 0; tid < n_tracks; ++tid)
                if (store.track_has_triangulated_xyz(static_cast<int>(tid)))
                    ++c;
            return c;
        }());
    LOG(INFO) << "Triangulation done: newly_tri=" << newly_tri
              << "  total_tri=" << total_tri;

    // ── 5. Analyze unregistered images ────────────────────────────────────────
    auto neighbor_map = build_neighbor_map(vg, poses.registered);

    struct ImageAnalysis {
        int image_index;
        int n_tracks_total;
        int n_3d2d;
        int n_registered_neighbors;
        int max_f_inliers_with_registered;   // ViewGraph quality signal
        int total_f_inliers_with_registered;
        // Top-5 registered neighbors by f_inliers
        std::vector<NeighborSummary> top_neighbors;
    };

    std::vector<ImageAnalysis> unregistered_analysis;
    unregistered_analysis.reserve(static_cast<size_t>(n_unregistered));

    for (int i = 0; i < n_images; ++i) {
        if (poses.registered[i])
            continue;

        ImageAnalysis a;
        a.image_index   = i;
        a.n_tracks_total = count_total_tracks(store, i);
        a.n_3d2d        = count_3d2d(store, i);

        auto& neighbors = neighbor_map[i];
        int n_reg_nb = 0, max_f = 0, total_f = 0;
        for (const auto& nb : neighbors) {
            if (nb.neighbor_registered) {
                ++n_reg_nb;
                total_f += nb.f_inliers;
                if (nb.f_inliers > max_f)
                    max_f = nb.f_inliers;
            }
        }
        a.n_registered_neighbors        = n_reg_nb;
        a.max_f_inliers_with_registered = max_f;
        a.total_f_inliers_with_registered = total_f;

        // Top-5 registered neighbors by f_inliers
        std::vector<NeighborSummary> reg_nbs;
        for (const auto& nb : neighbors)
            if (nb.neighbor_registered)
                reg_nbs.push_back(nb);
        std::sort(reg_nbs.begin(), reg_nbs.end(),
                  [](const NeighborSummary& a, const NeighborSummary& b) {
                      return a.f_inliers > b.f_inliers;
                  });
        a.top_neighbors.assign(reg_nbs.begin(),
                               reg_nbs.begin() + std::min(reg_nbs.size(), size_t(5)));

        unregistered_analysis.push_back(std::move(a));
    }

    // Sort by n_3d2d descending (best candidates first)
    std::sort(unregistered_analysis.begin(), unregistered_analysis.end(),
              [](const ImageAnalysis& a, const ImageAnalysis& b) {
                  return a.n_3d2d > b.n_3d2d;
              });

    // ── 6. Run choose_resection_candidates to see which would pass ────────────
    ResectionScoreCache score_cache;
    auto candidates = choose_resection_candidates(
        store, poses.registered, project.cameras, project.image_to_camera_index,
        min_3d2d, /*max_candidates=*/n_images,
        /*min_coverage_good=*/0.0f,
        /*visibility_pyramid_levels=*/4,
        &score_cache);

    std::unordered_set<int> resectable_set;
    for (const auto& c : candidates)
        resectable_set.insert(c.image_index);
    LOG(INFO) << "Images that would be resectable after triangulation: "
              << candidates.size() << " (min_3d2d=" << min_3d2d << ")";

    // Also count "raw resectable" (just the n_3d2d >= min_3d2d filter, no internal cutoff).
    int raw_resectable = 0;
    for (const auto& a : unregistered_analysis)
        if (a.n_3d2d >= min_3d2d)
            ++raw_resectable;
    const int cutoff_filtered = raw_resectable - static_cast<int>(candidates.size());

    // ── 7. Print console summary ──────────────────────────────────────────────
    std::cout << "\n";
    std::cout << "═══════════════════════════════════════════════════════════\n";
    std::cout << "  SfM Diagnosis: " << n_unregistered << " unregistered / "
              << n_images << " total\n";
    std::cout << "  After full-scan triangulation: total_tri=" << total_tri
              << " (+" << newly_tri << " new)\n";
    std::cout << "  Raw resectable (≥" << min_3d2d << " 3D-2D, no pipeline cutoff): "
              << raw_resectable << "\n";
    std::cout << "  Would-be-resectable (choose_resection_candidates): "
              << candidates.size() << "\n";
    if (cutoff_filtered > 0)
        std::cout << "  *** Internal pipeline cutoff (num_3d2d<100 truncation) dropped "
                  << cutoff_filtered << " images ***\n";
    std::cout << "═══════════════════════════════════════════════════════════\n\n";

    // Categories:
    // A: n_3d2d >= min_3d2d AND in resectable → triangulation was insufficient (fixable)
    // B: n_3d2d < min_3d2d but n_registered_neighbors > 0 → weak matches
    // C: n_registered_neighbors == 0 → isolated (no ViewGraph edge to any registered)
    int cat_a = 0, cat_b = 0, cat_c = 0;
    for (const auto& a : unregistered_analysis) {
        if (a.n_3d2d >= min_3d2d)
            ++cat_a;
        else if (a.n_registered_neighbors > 0)
            ++cat_b;
        else
            ++cat_c;
    }
    std::cout << "  Category A (enough 3D-2D, fixable by better triangulation): "
              << cat_a << "\n";
    std::cout << "  Category B (has registered neighbors but <" << min_3d2d
              << " 3D-2D, weak matches):  " << cat_b << "\n";
    std::cout << "  Category C (no ViewGraph edge to any registered image):      "
              << cat_c << "\n\n";

    std::cout << " idx |  3D-2D | tracks | reg_nb | max_F | resectable | Category\n";
    std::cout << "-----|--------|--------|--------|-------|------------|----------\n";
    for (const auto& a : unregistered_analysis) {
        char cat = (a.n_3d2d >= min_3d2d) ? 'A'
                 : (a.n_registered_neighbors > 0) ? 'B' : 'C';
        bool res = resectable_set.count(a.image_index) > 0;
        std::printf(" %3d | %6d | %6d | %6d | %5d |     %s      | %c\n",
                    a.image_index, a.n_3d2d, a.n_tracks_total,
                    a.n_registered_neighbors, a.max_f_inliers_with_registered,
                    res ? "YES" : " NO", cat);
    }
    std::cout << "\n";

    // ── 8. Build and write JSON report ────────────────────────────────────────
    json report;
    report["summary"] = {
        {"n_images",           n_images},
        {"n_registered",       n_registered},
        {"n_unregistered",     n_unregistered},
        {"n_tracks",           n_tracks},
        {"n_triangulated",     total_tri},
        {"n_newly_triangulated", newly_tri},
        {"n_raw_resectable",   raw_resectable},
        {"n_would_be_resectable", static_cast<int>(candidates.size())},
        {"n_cutoff_by_pipeline_internal", cutoff_filtered},
        {"min_3d2d_threshold", min_3d2d},
        {"min_tri_angle_deg",  min_tri_angle_deg},
        {"category_a_fixable_by_retri",    cat_a},
        {"category_b_weak_matches",        cat_b},
        {"category_c_no_registered_edge",  cat_c}
    };

    json unregistered_arr = json::array();
    for (const auto& a : unregistered_analysis) {
        char cat = (a.n_3d2d >= min_3d2d) ? 'A'
                 : (a.n_registered_neighbors > 0) ? 'B' : 'C';
        json entry = {
            {"image_index",                   a.image_index},
            {"n_3d2d",                        a.n_3d2d},
            {"n_tracks_total",                a.n_tracks_total},
            {"n_registered_neighbors",        a.n_registered_neighbors},
            {"max_f_inliers_with_registered", a.max_f_inliers_with_registered},
            {"total_f_inliers_with_registered", a.total_f_inliers_with_registered},
            {"would_be_resectable",           resectable_set.count(a.image_index) > 0},
            {"category",                      std::string(1, cat)}
        };

        json top_nb_arr = json::array();
        for (const auto& nb : a.top_neighbors) {
            top_nb_arr.push_back({
                {"image_index",  nb.neighbor_image_index},
                {"f_inliers",    nb.f_inliers},
                {"e_ok",         nb.e_ok},
                {"twoview_ok",   nb.twoview_ok},
                {"stable",       nb.stable}
            });
        }
        entry["top_registered_neighbors"] = top_nb_arr;
        unregistered_arr.push_back(std::move(entry));
    }
    report["unregistered_images"] = std::move(unregistered_arr);

    // Resection candidate list (ranked)
    json cand_arr = json::array();
    for (const auto& c : candidates) {
        cand_arr.push_back({
            {"image_index",  c.image_index},
            {"n_3d2d",       c.num_3d2d},
            {"coverage",     c.coverage}
        });
    }
    report["resection_candidates"] = std::move(cand_arr);

    const std::string report_str = report.dump(2);
    if (output_path.empty()) {
        std::cout << report_str << "\n";
    } else {
        std::ofstream of(output_path);
        if (!of.is_open()) {
            LOG(ERROR) << "Cannot write report to " << output_path;
        } else {
            of << report_str;
            LOG(INFO) << "Wrote report to " << output_path;
        }
    }

    // ── 9. Optionally save triangulated TrackStore ────────────────────────────
    if (!save_tracks_path.empty()) {
        std::vector<uint32_t> idx_out(static_cast<size_t>(n_images));
        for (int i = 0; i < n_images; ++i)
            idx_out[i] = static_cast<uint32_t>(i);
        if (save_track_store_to_idc(store, idx_out, save_tracks_path, &vg)) {
            LOG(INFO) << "Saved triangulated tracks to " << save_tracks_path;
        } else {
            LOG(ERROR) << "Failed to save triangulated tracks to " << save_tracks_path;
        }
    }

    // ── 10. Optionally export Bundler format ──────────────────────────────────
    if (!save_bundler_path.empty()) {
        const auto image_paths = load_image_paths(project_path, n_images);
        if (!save_bundler_format(
                save_bundler_path,
                store, n_images,
                poses.registered, poses.poses_R, poses.poses_C,
                project.cameras, project.image_to_camera_index,
                image_paths)) {
            LOG(ERROR) << "Failed to export Bundler format to " << save_bundler_path;
        }
    }

    return 0;
}
