#include "feature_overlay_layer.h"

#include "render/render_camera.h"
#include "render/render_context.h"

#include <QGLWidget>

namespace insight {

FeatureOverlayLayer::FeatureOverlayLayer(QObject* parent)
    : render::RenderLayer(parent) {
  m_visible = true;
}

void FeatureOverlayLayer::set_feature_point(double u, double v,
                                            double image_w, double image_h) {
  marker_.u     = u;
  marker_.v     = v;
  marker_.valid = true;
  img_w_        = image_w;
  img_h_        = image_h;
}

void FeatureOverlayLayer::clear_feature_points() {
  marker_.valid = false;
}

void FeatureOverlayLayer::draw(render::RenderContext* rc) {
  render::RenderLayer::draw(rc);

  if (!marker_.valid)
    return;

  auto* cam = dynamic_cast<render::RenderCamera2d*>(rc->camera);
  if (!cam)
    return;

  const double px = marker_.u;
  const double py = marker_.v;

  // Scale marker size by zoom so it stays a constant visual size.
  const double base_len = 10.0;
  const double scale    = cam->scale_ratio();
  const double len      = base_len * scale;

  // ── Green crosshair (+) ────────────────────────────────────────────────────
  glColor3d(0.0, 1.0, 0.0);
  glLineWidth(2.0f);
  glBegin(GL_LINES);
  glVertex2d(px - len, py);
  glVertex2d(px + len, py);
  glVertex2d(px, py - len);
  glVertex2d(px, py + len);
  glEnd();
  glLineWidth(1.0f);

  // ── Small central dot ──────────────────────────────────────────────────────
  glPointSize(4.0f);
  glBegin(GL_POINTS);
  glVertex2d(px, py);
  glEnd();
  glPointSize(1.0f);

  // ── Coordinate label ───────────────────────────────────────────────────────
  double text_x = px + len + 4.0 * scale;
  double text_y = py - len - 2.0 * scale;
  QString label = QString("(%1, %2)")
                      .arg(marker_.u, 0, 'f', 1)
                      .arg(marker_.v, 0, 'f', 1);
  if (rc->widget) {
    auto* glw = dynamic_cast<QGLWidget*>(rc->widget);
    if (glw) glw->renderText(text_x, text_y, 0, label);
  }
}

} // namespace insight
