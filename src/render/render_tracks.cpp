
#include "render_tracks.h"
#include "render_types.h"

#include "render_context.h"
#include <algorithm>
#include <array>

namespace insight {

namespace render {

namespace {

/// 在相机局部坐标系下绘制视锥与像平面矩形：底边（图像「下方」）用高亮色并加粗，便于辨认相机朝向。
void draw_camera_image_frame(const RenderTracks::Photo& p, float photoscale,
                             const RenderTracks::RenderOptions& ro, const double pose_rgb[3]) {
  const float scale = 1.f / photoscale;
  glScalef(scale, scale, scale);
  glScalef(ro.photoScale, ro.photoScale, ro.photoScale);

  glBegin(GL_LINE_STRIP);
  glVertex3d(0, 0, 0);
  glVertex3d(0, 0, -p.focal);
  glEnd();

  const double z = -static_cast<double>(p.focal);
  const double hw = p.w * 0.5;
  const double hh = p.h * 0.5;

  glColor3dv(pose_rgb);
  // 左、上、右三边（不含底边），底边单独加粗着色
  glBegin(GL_LINE_STRIP);
  glVertex3d(-hw, -hh, z);
  glVertex3d(-hw, hh, z);
  glVertex3d(hw, hh, z);
  glVertex3d(hw, -hh, z);
  glEnd();

  GLfloat lw = 1.f;
  glGetFloatv(GL_LINE_WIDTH, &lw);
  glLineWidth(std::max(4.f, lw * 2.5f));
  glColor3d(0.98, 0.72, 0.08);
  glBegin(GL_LINES);
  glVertex3d(-hw, -hh, z);
  glVertex3d(hw, -hh, z);
  glEnd();
  glLineWidth(lw);
  glColor3dv(pose_rgb);
}

} // namespace

RenderTracks::RenderTracks() : show_photo_(true), show_vertex_(true) {}

RenderTracks::~RenderTracks() {}

void RenderTracks::rebuild_cam_to_tracks() {
  cam_to_tracks_.clear();
  if (photos_.empty())
    return;
  cam_to_tracks_.resize(photos_.size());
  for (int ti = 0; ti < static_cast<int>(tracks_.size()); ++ti) {
    for (const auto& ob : tracks_[ti].obs) {
      if (ob.photoId >= 0 && ob.photoId < static_cast<int>(cam_to_tracks_.size()))
        cam_to_tracks_[ob.photoId].push_back(ti);
    }
  }
}

void RenderTracks::set_selected_camera(int cam_idx) {
  selected_camera_ = cam_idx;
  selected_track_  = -1;
}

void RenderTracks::set_selected_track(int track_idx) {
  selected_track_  = track_idx;
  selected_camera_ = -1;
}

void RenderTracks::render_photo(const Photo& p, bool highlighted) {
  float photoscale = 6000;
  if (p.initPose.centerValid) {
    photoscale = 50.f;
    Vec3 color = highlighted ? Vec3(1.0, 0.3, 0.0) : p.initPose.color;
    glColor3dv(color.data());
    glBegin(GL_POINTS);
    glVertex3d(p.initPose.data[0], p.initPose.data[1], p.initPose.data[2]);
    glEnd();
#ifndef SHOW_GPS_ROTATION
#define SHOW_GPS_ROTATION
#endif
#ifdef SHOW_GPS_ROTATION
    if (p.initPose.rotationValid) {
      glMatrixMode(GL_MODELVIEW);
      glPushMatrix();
      glTranslated(p.initPose.data[0], p.initPose.data[1], p.initPose.data[2]);
      glMultMatrixd(p.initPose.openglMat.transpose().data());
      draw_camera_image_frame(p, photoscale, render_options_, color.data());
      glMatrixMode(GL_MODELVIEW);
      glPopMatrix();
    }
#endif
  }

  if (p.refinedPose.centerValid) {
    Vec3 color = highlighted ? Vec3(1.0, 0.3, 0.0) : p.refinedPose.color;
    glColor3dv(color.data());
    glBegin(GL_POINTS);
    glVertex3d(p.refinedPose.data[0], p.refinedPose.data[1], p.refinedPose.data[2]);
    glEnd();

    if (p.refinedPose.rotationValid) {
      glMatrixMode(GL_MODELVIEW);
      glPushMatrix();
      glTranslated(p.refinedPose.data[0], p.refinedPose.data[1], p.refinedPose.data[2]);
      glMultMatrixd(p.refinedPose.openglMat.transpose().data());
      draw_camera_image_frame(p, photoscale, render_options_, color.data());
      glMatrixMode(GL_MODELVIEW);
      glPopMatrix();
    }
  }
}

void RenderTracks::draw(RenderContext* rc) {
  // ── Snapshot GL state at draw time — used by pick ───────────────────────
  glGetDoublev(GL_MODELVIEW_MATRIX,  last_mv_);
  glGetDoublev(GL_PROJECTION_MATRIX, last_proj_);
  glGetIntegerv(GL_VIEWPORT,         last_vp_);
  gl_state_valid_ = true;
  memcpy(gl_state_.mv,   last_mv_,   16 * sizeof(double));
  memcpy(gl_state_.proj, last_proj_, 16 * sizeof(double));
  memcpy(gl_state_.vp,   last_vp_,   4  * sizeof(GLint));
  gl_state_.valid = true;

  // ── Helper: get world-space center of a photo pose ───────────────────────
  auto photo_center = [](const Photo& ph, Vec3* out) -> bool {
    if (ph.refinedPose.centerValid) {
      *out = Vec3(ph.refinedPose.data[0], ph.refinedPose.data[1], ph.refinedPose.data[2]);
      return true;
    }
    if (ph.initPose.centerValid) {
      *out = Vec3(ph.initPose.data[0], ph.initPose.data[1], ph.initPose.data[2]);
      return true;
    }
    return false;
  };

  // ── Build highlight sets ─────────────────────────────────────────────────
  // highlighted_tracks: track indices to draw bright
  // highlighted_cams:   camera indices to draw bright
  std::vector<bool> hl_track(tracks_.size(), false);
  std::vector<bool> hl_cam(photos_.size(), false);

  if (selected_camera_ >= 0 && selected_camera_ < static_cast<int>(photos_.size())) {
    hl_cam[selected_camera_] = true;
    // highlight all tracks observed by this camera
    if (selected_camera_ < static_cast<int>(cam_to_tracks_.size())) {
      for (int ti : cam_to_tracks_[selected_camera_])
        if (ti >= 0 && ti < static_cast<int>(hl_track.size()))
          hl_track[ti] = true;
    }
  }
  if (selected_track_ >= 0 && selected_track_ < static_cast<int>(tracks_.size())) {
    hl_track[selected_track_] = true;
    for (const auto& ob : tracks_[selected_track_].obs)
      if (ob.photoId >= 0 && ob.photoId < static_cast<int>(hl_cam.size()))
        hl_cam[ob.photoId] = true;
  }

  // ── Draw cameras ─────────────────────────────────────────────────────────
  glPointSize(render_options_.poseSize);
  if (show_photo_) {
    for (size_t i = 0; i < photos_.size(); ++i)
      render_photo(photos_[i], hl_cam[i]);
  }

  // ── Draw 3D points ────────────────────────────────────────────────────────
  if (show_vertex_) {
    // Normal points
    glPointSize(render_options_.vetexSize);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < tracks_.size(); ++i) {
      if (hl_track[i]) continue; // drawn separately below
      glColor3dv(tracks_[i].color.data());
      glVertex3d(tracks_[i].x, tracks_[i].y, tracks_[i].z);
    }
    glEnd();

    // Highlighted points (bright yellow, larger)
    bool any_hl = false;
    for (bool b : hl_track) if (b) { any_hl = true; break; }
    if (any_hl) {
      glPointSize(render_options_.vetexSize * 4.f);
      glBegin(GL_POINTS);
      glColor3d(1.0, 0.9, 0.0);
      for (size_t i = 0; i < tracks_.size(); ++i) {
        if (!hl_track[i]) continue;
        glVertex3d(tracks_[i].x, tracks_[i].y, tracks_[i].z);
      }
      glEnd();
    }

    // GCPs
    glPointSize(render_options_.vetexSize * 5);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < gcps_.size(); ++i) {
      glColor3dv(gcps_[i].color.data());
      glVertex3d(gcps_[i].x, gcps_[i].y, gcps_[i].z);
    }
    glEnd();
  }

  // ── Draw observation lines (camera ↔ 3D point) ───────────────────────────
  // Case 1: selected camera → draw lines to all its tracks
  if (selected_camera_ >= 0 && selected_camera_ < static_cast<int>(photos_.size())) {
    Vec3 cam_c;
    if (photo_center(photos_[selected_camera_], &cam_c) &&
        selected_camera_ < static_cast<int>(cam_to_tracks_.size())) {
      GLfloat lw = 1.f;
      glGetFloatv(GL_LINE_WIDTH, &lw);
      glLineWidth(1.f);
      glColor3d(1.0, 0.85, 0.0);
      glBegin(GL_LINES);
      for (int ti : cam_to_tracks_[selected_camera_]) {
        if (ti < 0 || ti >= static_cast<int>(tracks_.size())) continue;
        const auto& tk = tracks_[ti];
        glVertex3d(cam_c.x(), cam_c.y(), cam_c.z());
        glVertex3d(tk.x, tk.y, tk.z);
      }
      glEnd();
      glLineWidth(lw);
    }
  }

  // Case 2: selected track → draw lines to all observing cameras
  if (selected_track_ >= 0 && selected_track_ < static_cast<int>(tracks_.size())) {
    const auto& tk = tracks_[selected_track_];
    GLfloat lw = 1.f;
    glGetFloatv(GL_LINE_WIDTH, &lw);
    glLineWidth(1.f);
    glColor3d(1.0, 0.85, 0.0);
    glBegin(GL_LINES);
    for (const auto& ob : tk.obs) {
      if (ob.photoId < 0 || ob.photoId >= static_cast<int>(photos_.size())) continue;
      Vec3 cam_c;
      if (!photo_center(photos_[ob.photoId], &cam_c)) continue;
      glVertex3d(tk.x, tk.y, tk.z);
      glVertex3d(cam_c.x(), cam_c.y(), cam_c.z());
    }
    glEnd();
    glLineWidth(lw);
  }

  double xsize = 1, ysize = 1, zsize = 1;
  if (show_grid_) {
    glColor3ubv(render_options_.gridcolor);
    for (int j = 0; j < grid_.xs.size(); ++j) {
      glBegin(GL_LINE_LOOP);
      glVertex3d(grid_.xs[j], grid_.miny, grid_.minz);
      glVertex3d(grid_.xs[j], grid_.maxy, grid_.minz);
      glVertex3d(grid_.xs[j], grid_.maxy, grid_.maxz);
      glVertex3d(grid_.xs[j], grid_.miny, grid_.maxz);
      glEnd();
    }
    for (int j = 0; j < grid_.ys.size(); ++j) {
      glBegin(GL_LINE_LOOP);
      glVertex3d(grid_.minx, grid_.ys[j], grid_.minz);
      glVertex3d(grid_.maxx, grid_.ys[j], grid_.minz);
      glVertex3d(grid_.maxx, grid_.ys[j], grid_.maxz);
      glVertex3d(grid_.minx, grid_.ys[j], grid_.maxz);
      glEnd();
    }
    for (int j = 0; j < grid_.zs.size(); ++j) {
      glBegin(GL_LINE_LOOP);
      glVertex3d(grid_.minx, grid_.miny, grid_.zs[j]);
      glVertex3d(grid_.maxx, grid_.miny, grid_.zs[j]);
      glVertex3d(grid_.maxx, grid_.maxy, grid_.zs[j]);
      glVertex3d(grid_.minx, grid_.maxy, grid_.zs[j]);
      glEnd();
    }
    xsize = (grid_.maxx - grid_.minx) * 0.5;
    ysize = (grid_.maxy - grid_.miny) * 0.5;
    zsize = (grid_.maxz - grid_.minz) * 0.5;
  }

  if (show_axis_) {
    glColor3ub(255, 0, 0);
    glBegin(GL_LINES);
    glVertex3d(0, 0, 0);
    glVertex3d(xsize, 0, 0);
    glEnd();
    glColor3ub(0, 255, 0);
    glBegin(GL_LINES);
    glVertex3d(0, 0, 0);
    glVertex3d(0, ysize, 0);
    glEnd();
    glColor3ub(0, 0, 255);
    glBegin(GL_LINES);
    glVertex3d(0, 0, 0);
    glVertex3d(0, 0, zsize);
    glEnd();
  }
}

bool RenderTracks::world_axis_aligned_bounds(Vec3* bmin, Vec3* bmax) const {
  bool any = false;
  auto merge = [&](const Vec3& v) {
    if (!any) {
      *bmin = v;
      *bmax = v;
      any = true;
    } else {
      *bmin = bmin->cwiseMin(v);
      *bmax = bmax->cwiseMax(v);
    }
  };

  for (const auto& p : photos_) {
    if (p.initPose.centerValid) {
      merge(Vec3(p.initPose.data[0], p.initPose.data[1], p.initPose.data[2]));
    }
    if (p.refinedPose.centerValid) {
      merge(Vec3(p.refinedPose.data[0], p.refinedPose.data[1], p.refinedPose.data[2]));
    }
  }
  for (const auto& t : tracks_) {
    merge(Vec3(t.x, t.y, t.z));
  }
  for (const auto& g : gcps_) {
    merge(Vec3(g.x, g.y, g.z));
  }
  return any;
}

void RenderTracks::set_center(double x, double y, double z) {
  center_.at(0) = x;
  center_.at(1) = y;
  center_.at(2) = z;
}

bool RenderTracks::unproject_ray(int px, int py, Vec3* ray_origin, Vec3* ray_dir) const {
  if (!gl_state_valid_)
    return false;

  // GL viewport origin is bottom-left; Qt/screen origin is top-left — flip y.
  const int gl_y = last_vp_[3] - 1 - py;

  GLdouble wx0, wy0, wz0, wx1, wy1, wz1;
  if (gluUnProject(px, gl_y, 0.0, last_mv_, last_proj_, last_vp_, &wx0, &wy0, &wz0) != GL_TRUE)
    return false;
  if (gluUnProject(px, gl_y, 1.0, last_mv_, last_proj_, last_vp_, &wx1, &wy1, &wz1) != GL_TRUE)
    return false;

  *ray_origin = Vec3(wx0, wy0, wz0);
  *ray_dir    = (Vec3(wx1, wy1, wz1) - *ray_origin).normalized();
  return true;
}

int RenderTracks::pick_screen(int px, int py, bool* is_camera, double threshold_px) const {
  if (!gl_state_valid_)
    return -1;

  // Qt y → GL y
  const double gl_py = last_vp_[3] - 1 - py;
  const double thresh2 = threshold_px * threshold_px;

  auto project_pt = [&](double wx, double wy, double wz, double* sx, double* sy) -> bool {
    GLdouble gx, gy, gz;
    if (gluProject(wx, wy, wz, last_mv_, last_proj_, last_vp_, &gx, &gy, &gz) != GL_TRUE)
      return false;
    *sx = gx;
    *sy = gy;  // GL bottom-left
    return gz > 0.0 && gz < 1.0;  // in front of camera and within depth range
  };

  // ── Cameras ──────────────────────────────────────────────────────────────
  int   best_cam = -1;
  double best_cam_d2 = thresh2;
  for (int i = 0; i < static_cast<int>(photos_.size()); ++i) {
    const auto& ph = photos_[i];
    double wx, wy, wz;
    if (ph.refinedPose.centerValid) {
      wx = ph.refinedPose.data[0]; wy = ph.refinedPose.data[1]; wz = ph.refinedPose.data[2];
    } else if (ph.initPose.centerValid) {
      wx = ph.initPose.data[0]; wy = ph.initPose.data[1]; wz = ph.initPose.data[2];
    } else continue;

    double sx, sy;
    if (!project_pt(wx, wy, wz, &sx, &sy)) continue;
    double dx = sx - px, dy = sy - gl_py;
    double d2 = dx*dx + dy*dy;
    if (d2 < best_cam_d2) { best_cam_d2 = d2; best_cam = i; }
  }

  // ── 3D points ─────────────────────────────────────────────────────────────
  int   best_pt = -1;
  double best_pt_d2 = thresh2;
  for (int i = 0; i < static_cast<int>(tracks_.size()); ++i) {
    const auto& tk = tracks_[i];
    double sx, sy;
    if (!project_pt(tk.x, tk.y, tk.z, &sx, &sy)) continue;
    double dx = sx - px, dy = sy - gl_py;
    double d2 = dx*dx + dy*dy;
    if (d2 < best_pt_d2) { best_pt_d2 = d2; best_pt = i; }
  }

  // Return whichever is closer
  if (best_cam >= 0 && best_cam_d2 <= best_pt_d2) {
    *is_camera = true;
    return best_cam;
  }
  if (best_pt >= 0) {
    *is_camera = false;
    return best_pt;
  }
  return -1;
}

} // namespace render

} // namespace insight
