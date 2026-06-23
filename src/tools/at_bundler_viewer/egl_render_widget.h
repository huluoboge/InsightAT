#pragma once

// Include GLEW / render headers BEFORE QOpenGLWidget.
#include "render/render_camera.h"
#include "render/render_context.h"
#include "render/render_node.h"
#include "render/render_pan_tool.h"
#include "render/render_pivot.h"
#include "render/render_rotation_tool.h"
#include "render/render_zoom_tool.h"
#include "util/numeric.h"

#include <QOpenGLWidget>
#include <QPoint>

class QOpenGLShaderProgram;

namespace insight {

namespace render {
class RenderTracks;
} // namespace render

/// QOpenGLWidget-based 3D point-cloud viewer with EDL (Eye-Dome Lighting).
/// Renders the scene to an FBO then applies a screen-space depth-based
/// edge-darkening shader for enhanced boundary perception.
class EglRenderWidget : public QOpenGLWidget {
  Q_OBJECT
public:
  explicit EglRenderWidget(QWidget* parent = nullptr);
  ~EglRenderWidget() override;

  render::RenderNode* root() { return m_root; }
  render::RenderNode* data_root() { return data_root_; }

  void set_pivot_visible(bool vis);
  void set_pick_mode(bool enabled) { pick_mode_ = enabled; }
  void set_edl_enabled(bool on);

  /// Reset scene and fit camera to data bounds.
  void fit_scene_to_view();

  /// Unproject screen pixel → world ray (delegates to RenderTracks).
  bool unproject_ray(int px, int py, Vec3* ray_origin, Vec3* ray_dir) const;

signals:
  void clicked(int px, int py);

protected:
  void initializeGL() override;
  void paintGL() override;
  void resizeGL(int w, int h) override;

  void mousePressEvent(QMouseEvent* e) override;
  void mouseMoveEvent(QMouseEvent* e) override;
  void mouseReleaseEvent(QMouseEvent* e) override;
  void wheelEvent(QWheelEvent* e) override;

private:
  void init_scene();
  void init_edl();
  void destroy_fbo();
  void create_fbo(int w, int h);

  // ── Scene graph (same as RenderWidget) ───────────────────────────────────
  render::RenderCamera*       m_camera        = nullptr;
  render::RenderContext*      render_context_ = nullptr;
  render::RenderRotationTool* m_rotationTool  = nullptr;
  render::RenderPanTool*      m_panTool       = nullptr;
  render::RenderZoomTool*     m_zoomTool      = nullptr;
  render::RenderPivot*        m_pivot         = nullptr;
  render::RenderNode*         m_root          = nullptr;
  render::RenderNode*         data_root_      = nullptr;

  // ── EDL ──────────────────────────────────────────────────────────────────
  bool edl_enabled_ = true;
  QOpenGLShaderProgram* edl_prog_ = nullptr;
  GLuint edl_fbo_    = 0;
  GLuint edl_color_  = 0;
  GLuint edl_depth_  = 0;
  int    edl_fbo_w_  = 0;
  int    edl_fbo_h_  = 0;

  // ── Pick support ─────────────────────────────────────────────────────────
  bool  pick_mode_ = false;
  QPoint m_press_pos;
};

} // namespace insight
