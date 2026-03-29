
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

void RenderTracks::render_photo(const Photo& p) {
  float photoscale = 6000;
  if (p.initPose.centerValid) {
    photoscale = 50.f;
    glColor3dv(p.initPose.color.data());
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
      {
        draw_camera_image_frame(p, photoscale, render_options_, p.initPose.color.data());
      }
      glMatrixMode(GL_MODELVIEW);
      glPopMatrix();
    }
#endif
  }

  if (p.refinedPose.centerValid) {
    glColor3dv(p.refinedPose.color.data());
    glBegin(GL_POINTS);
    glVertex3d(p.refinedPose.data[0], p.refinedPose.data[1], p.refinedPose.data[2]);
    glEnd();

    if (p.refinedPose.rotationValid) {
      glMatrixMode(GL_MODELVIEW);
      glPushMatrix();
      glTranslated(p.refinedPose.data[0], p.refinedPose.data[1], p.refinedPose.data[2]);
      glMultMatrixd(p.refinedPose.openglMat.transpose().data());
      {
        draw_camera_image_frame(p, photoscale, render_options_, p.refinedPose.color.data());
      }
      glMatrixMode(GL_MODELVIEW);
      glPopMatrix();
    }
  }
}

void RenderTracks::draw(RenderContext* rc) {
  // draw photos
  glPointSize(render_options_.poseSize);
  // draw imported pose
  if (show_photo_) {
    for (size_t i = 0; i < photos_.size(); ++i) {
      render_photo(photos_[i]);
    }
  }

  if (show_vertex_) {

    // draw 3d points
    glPointSize(render_options_.vetexSize);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < tracks_.size(); ++i) {
      glColor3dv(tracks_[i].color.data());
      glVertex3d(tracks_[i].x, tracks_[i].y, tracks_[i].z);
    }
    glEnd();

    glPointSize(render_options_.vetexSize * 5);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < gcps_.size(); ++i) {
      glColor3dv(gcps_[i].color.data());
      glVertex3d(gcps_[i].x, gcps_[i].y, gcps_[i].z);
    }
    glEnd();
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

} // namespace render

} // namespace insight
