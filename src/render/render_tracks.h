#pragma once

#include "render_global.h"
#include "render_object.h"
#include "render_types.h"
#include <QString>
#include <array>

namespace insight {

namespace render {

class RENDER_EXPORT RenderTracks : public RenderObject {
public:
  RenderTracks();
  virtual ~RenderTracks();
  struct Photo {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Photo() {
      id = 0;
      focal = 0;
      w = 0;
      h = 0;
    }
    int id;
    float focal;
    float w;
    float h;

    struct Pose {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      Pose() {
        memset(data, 0, sizeof(double) * 6);
        centerValid = false;
        rotationValid = false;
        openglMat.setIdentity();
        color.setOnes();
      }
      double data[6]; // C and OmegaPhiKappa
      bool centerValid;
      bool rotationValid;
      Mat4 openglMat;
      Vec3 color;
    };

    Pose initPose;
    Pose refinedPose;
    QString name;
  };
  /*
   */
  struct Observe {
    Observe() {
      photoId = 0;
      featX = 0;
      featY = 0;
    }
    int photoId;
    float featX, featY;
  };
  /*
   */
  struct Track {
    int trackId = 0;
    double x = 0;
    double y = 0;
    double z = 0;
    Vec3 color;
    std::vector<Observe> obs;
  };

  struct Grid {
    Grid() : minx(0), miny(0), minz(0), maxx(1), maxy(1), maxz(1) {}
    int xcount = 1;
    int ycount = 1;
    int zcount = 1;
    double minx, miny, minz;
    double maxx, maxy, maxz;
    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> zs;
    void generate(double low, double high, int count, std::vector<double>& datas) {
      if (count == 1) {
        datas.push_back(low);
        datas.push_back(high);
      } else if (count == 2) {
        double mid = (low + high) * 0.5;
        datas.emplace_back(low);
        datas.emplace_back(mid);
        datas.emplace_back(high);
      } else {
        int n = count - 1;
        double space = (high - low) / n;
        double half = space * 0.5;
        datas.emplace_back(low);
        double start = low + half;
        for (int i = 0; i < n; ++i) {
          double d = start + i * space;
          datas.emplace_back(d);
        }
        datas.emplace_back(high);
      }
    }
    void generate_datas() {
      //两边都是0.5，中间都是1
      xs.clear();
      ys.clear();
      zs.clear();
      generate(minx, maxx, xcount, xs);
      generate(miny, maxy, ycount, ys);
      generate(minz, maxz, zcount, zs);
    }
  };
  struct RenderOptions {
    RenderOptions() {}
    float photoScale = 1.f;
    float poseSize = 3;
    float vetexSize = 1;
    uint8_t gridcolor[3] = {10, 150, 100};
  };

  virtual void draw(RenderContext* rc);

  typedef std::vector<Track> Tracks;
  typedef std::vector<Photo> Photos;

  void set_render_options(const RenderOptions& opt) { render_options_ = opt; }

  void set_tracks(const Tracks& t) { tracks_ = t; rebuild_cam_to_tracks(); }
  void set_photos(const Photos& p) { photos_ = p; }
  void set_gcps(const Tracks& gcp) { gcps_ = gcp; }
  void set_center(double x, double y, double z);

  const Photos& photos() const { return photos_; }
  const Tracks& tracks() const { return tracks_; }
  const RenderOptions& render_options() const { return render_options_; }

  // ── Selection / highlight ────────────────────────────────────────────────
  void set_selected_camera(int cam_idx);
  void set_selected_track(int track_idx);
  void clear_selection() { set_selected_camera(-1); set_selected_track(-1); }

  int selected_camera() const { return selected_camera_; }
  int selected_track()  const { return selected_track_; }

  int cam_to_tracks_size() const { return static_cast<int>(cam_to_tracks_.size()); }
  int cam_track_count(int cam_idx) const {
    if (cam_idx < 0 || cam_idx >= static_cast<int>(cam_to_tracks_.size())) return 0;
    return static_cast<int>(cam_to_tracks_[cam_idx].size());
  }

  // ── Pick: unproject screen pixel to world-space ray ─────────────────────
  /// Uses GL state captured during the last draw() call.
  /// Returns false if draw() has never been called.
  bool unproject_ray(int px, int py, Vec3* ray_origin, Vec3* ray_dir) const;

  /// Screen-space pick: find the camera or track closest to screen pixel (px,py).
  /// Returns index >= 0 on hit, -1 on miss. is_camera=true means camera hit.
  /// threshold_px: max screen-space distance in pixels.
  int pick_screen(int px, int py, bool* is_camera, double threshold_px = 12.0) const;

  struct GlState {
    double mv[16]   = {};
    double proj[16] = {};
    GLint  vp[4]    = {};
    bool   valid    = false;
  };
  const GlState& gl_state() const { return gl_state_; }

  /// 相机中心与点云在当前节点坐标系下的轴对齐包围盒；无几何则返回 false。
  bool world_axis_aligned_bounds(Vec3* bmin, Vec3* bmax) const;

  void clear() {
    tracks_.clear();
    photos_.clear();
    cam_to_tracks_.clear();
    selected_camera_ = -1;
    selected_track_  = -1;
  }
  void photo_larger() { render_options_.photoScale *= 1.1f; }
  void photo_smaller() { render_options_.photoScale *= 0.9f; }

  void vertex_large() { render_options_.vetexSize *= 1.1f; };
  void vertex_smaller() { render_options_.vetexSize *= 0.9f; };

  void set_photo_visible(bool vis) { show_photo_ = vis; }
  void set_vertex_visible(bool vis) { show_vertex_ = vis; }

  bool is_photo_visible() const { return show_photo_; }
  bool is_vertex_visible() const { return show_vertex_; }

  void set_grid_visible(bool vis) { show_grid_ = vis; }

  void set_grid(const Grid& grid) { grid_ = grid; }
  Grid& grid() { return grid_; }

protected:
  void render_photo(const Photo& p, bool highlighted = false);
  void draw_highlight_overlay();
  void rebuild_cam_to_tracks(); ///< Build cam_to_tracks_ index from tracks_.

protected:
  bool show_photo_;
  bool show_vertex_;
  bool show_grid_ = false;
  bool show_axis_ = true;

  Tracks tracks_;
  Tracks gcps_;
  Photos photos_;
  Grid grid_;
  RenderOptions render_options_;

  std::array<double, 3> center_;

  // ── Selection state ──────────────────────────────────────────────────────
  int selected_camera_ = -1;
  int selected_track_  = -1;

  /// cam_to_tracks_[cam_idx] = list of track indices that observe that camera.
  std::vector<std::vector<int>> cam_to_tracks_;

  // ── GL state captured during last draw() — used for picking ─────────────
  mutable double last_mv_[16]   = {};
  mutable double last_proj_[16] = {};
  mutable GLint  last_vp_[4]    = {};
  mutable bool   gl_state_valid_ = false;
  mutable GlState gl_state_;
};
} // namespace render

} // namespace insight
