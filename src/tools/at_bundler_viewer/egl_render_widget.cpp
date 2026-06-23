#include "egl_render_widget.h"

#include "render/render_tracks.h"

#include <glog/logging.h>

#include <QMouseEvent>
#include <QOpenGLShaderProgram>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>

namespace insight {

// ── EDL GLSL shaders (GLSL 1.20, compatible with legacy GL context) ────────

static const char* kEdlVertexShader = R"(
void main() {
    gl_TexCoord[0] = gl_MultiTexCoord0;
    gl_Position = ftransform();
}
)";

static const char* kEdlFragmentShader = R"(
uniform sampler2D uColor;
uniform sampler2D uDepth;
uniform vec2      uPixelSize;
uniform float     uStrength;   // typically 1.0–4.0
uniform float     uRadius;     // neighbour radius in pixels (2–6)

void main() {
    vec2 tc   = gl_TexCoord[0].xy;
    float d0  = texture2D(uDepth, tc).r;

    // Background pixel — pass through unshaded.
    if (d0 >= 1.0) {
        gl_FragColor = texture2D(uColor, tc);
        return;
    }

    float shade = 0.0;
    const int N = 8;
    for (int i = 0; i < N; ++i) {
        float angle  = float(i) * 0.7853981634;  // pi/4
        vec2  offset = vec2(cos(angle), sin(angle)) * uRadius * uPixelSize;
        float dn     = texture2D(uDepth, tc + offset).r;
        if (dn < d0)
            shade += (d0 - dn) * uStrength * 100.0;
    }
    shade /= float(N);
    float bright = exp(-shade);

    vec4 color = texture2D(uColor, tc);
    gl_FragColor = vec4(color.rgb * bright, 1.0);
}
)";

// ── Construction / destruction ─────────────────────────────────────────────

EglRenderWidget::EglRenderWidget(QWidget* parent)
    : QOpenGLWidget(parent) {
  setFocusPolicy(Qt::StrongFocus);
  setMouseTracking(true);
  init_scene();
}

EglRenderWidget::~EglRenderWidget() {
  makeCurrent();
  destroy_fbo();
  delete edl_prog_;
  delete m_camera;
  delete render_context_;
  delete m_rotationTool;
}

void EglRenderWidget::init_scene() {
  m_camera = new render::RenderCamera;
  render_context_ = new render::RenderContext;
  render_context_->camera = m_camera;
  render_context_->widget = this;

  m_pivot = new render::RenderPivot;

  m_root = new render::RenderNode;
  data_root_ = new render::RenderNode(m_root);

  m_rotationTool = new render::RenderRotationTool;
  m_rotationTool->set_render_context(render_context_);
  m_rotationTool->set_cursor(QCursor(Qt::DragMoveCursor));
  m_rotationTool->set_target(m_root);
  m_rotationTool->set_pivot(m_pivot);

  m_panTool = new render::RenderPanTool;
  m_panTool->set_target(data_root_);
  m_panTool->set_render_context(render_context_);

  m_zoomTool = new render::RenderZoomTool;
  m_zoomTool->set_target(m_root);
  m_zoomTool->set_render_context(render_context_);
}

void EglRenderWidget::set_pivot_visible(bool vis) {
  m_pivot->setVisible(vis);
}

bool EglRenderWidget::unproject_ray(int px, int py, Vec3* ray_origin,
                                    Vec3* ray_dir) const {
  for (render::RenderObject* o : data_root_->render_objects()) {
    auto* tracks = dynamic_cast<render::RenderTracks*>(o);
    if (tracks)
      return tracks->unproject_ray(px, py, ray_origin, ray_dir);
  }
  return false;
}

// ── EDL toggle ─────────────────────────────────────────────────────────────

void EglRenderWidget::set_edl_enabled(bool on) {
  edl_enabled_ = on;
  update();
}

// ── GL lifecycle ───────────────────────────────────────────────────────────

void EglRenderWidget::initializeGL() {
#if !defined(RENDER_NO_GLEW)
  GLenum code = glewInit();
  if (GLEW_OK != static_cast<int>(code)) {
    LOG(ERROR) << "Init glew failed: " << glewGetErrorString(code);
    return;
  }
#endif
  glClearColor(0, 0, 0, 1.0);
  glClearDepth(1.0);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_SMOOTH);
  glHint(GL_POINT_SMOOTH, GL_NICEST);
  glHint(GL_LINE_SMOOTH, GL_NICEST);
  glEnable(GL_POINT_SMOOTH);
  glEnable(GL_LINE_SMOOTH);

  m_camera->look_at(Vec3(0, 0, 1000), Vec3(0, 0, 0), Vec3(0, 1, 0));

  init_edl();
}

void EglRenderWidget::init_edl() {
  edl_prog_ = new QOpenGLShaderProgram(this);
  if (!edl_prog_->addShaderFromSourceCode(QOpenGLShader::Vertex, kEdlVertexShader)) {
    LOG(ERROR) << "EDL vertex shader: " << edl_prog_->log().toStdString();
  }
  if (!edl_prog_->addShaderFromSourceCode(QOpenGLShader::Fragment, kEdlFragmentShader)) {
    LOG(ERROR) << "EDL fragment shader: " << edl_prog_->log().toStdString();
  }
  if (!edl_prog_->link()) {
    LOG(ERROR) << "EDL shader link: " << edl_prog_->log().toStdString();
  }
}

void EglRenderWidget::destroy_fbo() {
  if (edl_fbo_ != 0) {
    glDeleteFramebuffers(1, &edl_fbo_);
    edl_fbo_ = 0;
  }
  if (edl_color_ != 0) {
    glDeleteTextures(1, &edl_color_);
    edl_color_ = 0;
  }
  if (edl_depth_ != 0) {
    glDeleteTextures(1, &edl_depth_);
    edl_depth_ = 0;
  }
}

void EglRenderWidget::create_fbo(int w, int h) {
  destroy_fbo();

  // Colour texture
  glGenTextures(1, &edl_color_);
  glBindTexture(GL_TEXTURE_2D, edl_color_);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA,
               GL_UNSIGNED_BYTE, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  // Depth texture
  glGenTextures(1, &edl_depth_);
  glBindTexture(GL_TEXTURE_2D, edl_depth_);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, w, h, 0,
               GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  // FBO
  glGenFramebuffers(1, &edl_fbo_);
  glBindFramebuffer(GL_FRAMEBUFFER, edl_fbo_);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                         GL_TEXTURE_2D, edl_color_, 0);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                         GL_TEXTURE_2D, edl_depth_, 0);

  GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
  if (status != GL_FRAMEBUFFER_COMPLETE)
    LOG(ERROR) << "EDL FBO incomplete: 0x" << std::hex << status;

  glBindFramebuffer(GL_FRAMEBUFFER, defaultFramebufferObject());
  edl_fbo_w_ = w;
  edl_fbo_h_ = h;
}

// ── Two-pass rendering ─────────────────────────────────────────────────────

void EglRenderWidget::paintGL() {
  const int w = std::max(1, width());
  const int h = std::max(1, height());

  // Recreate FBO if size changed.
  if (edl_enabled_ && (edl_fbo_w_ != w || edl_fbo_h_ != h))
    create_fbo(w, h);

  if (edl_enabled_ && edl_fbo_ != 0) {
    // ── Pass 1: render scene → FBO ──────────────────────────────────────
    glBindFramebuffer(GL_FRAMEBUFFER, edl_fbo_);
    glViewport(0, 0, w, h);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    m_camera->update_gl_matrix();
    render_context_->clear();
    render_context_->modelview = m_camera->view_matrix();
    if (m_pivot->isVisible())
      m_pivot->draw(render_context_);
    m_root->render(render_context_);

    // Ensure all drawing commands finish before we sample the textures.
    glFlush();

    // ── Pass 2: EDL post-process → default FB ───────────────────────────
    glBindFramebuffer(GL_FRAMEBUFFER, defaultFramebufferObject());
    glViewport(0, 0, w, h);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_DEPTH_TEST);

    edl_prog_->bind();
    edl_prog_->setUniformValue("uPixelSize",
                               QVector2D(1.0f / w, 1.0f / h));
    edl_prog_->setUniformValue("uStrength", 2.5f);
    edl_prog_->setUniformValue("uRadius", 4.0f);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, edl_color_);
    edl_prog_->setUniformValue("uColor", 0);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, edl_depth_);
    edl_prog_->setUniformValue("uDepth", 1);

    // Full-screen quad in ortho.
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, 1, 0, 1, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glBegin(GL_QUADS);
    glTexCoord2f(0, 0);
    glVertex2f(0, 0);
    glTexCoord2f(1, 0);
    glVertex2f(1, 0);
    glTexCoord2f(1, 1);
    glVertex2f(1, 1);
    glTexCoord2f(0, 1);
    glVertex2f(0, 1);
    glEnd();
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

    edl_prog_->release();
    glBindTexture(GL_TEXTURE_2D, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, 0);
  } else {
    // ── Plain rendering (EDL disabled or FBO not ready) ─────────────────
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    m_camera->update_gl_matrix();
    render_context_->clear();
    render_context_->modelview = m_camera->view_matrix();
    if (m_pivot->isVisible())
      m_pivot->draw(render_context_);
    m_root->render(render_context_);
  }
}

void EglRenderWidget::resizeGL(int w, int h) {
  m_camera->frustum(-double(w) / h, double(w) / h, -1, 1, 5, 100000);
  glViewport(0, 0, w, h);
  render_context_->w = w;
  render_context_->h = h;
}

// ── Mouse events (mirrors RenderWidget, plus pick support) ─────────────────

void EglRenderWidget::mousePressEvent(QMouseEvent* e) {
  m_press_pos = e->pos();
  m_rotationTool->mousePressEvent(e);
  m_panTool->mousePressEvent(e);
  update();
}

void EglRenderWidget::mouseMoveEvent(QMouseEvent* e) {
  m_rotationTool->mouseMoveEvent(e);
  m_panTool->mouseMoveEvent(e);
  update();
}

void EglRenderWidget::mouseReleaseEvent(QMouseEvent* e) {
  if (pick_mode_ && e->button() == Qt::LeftButton &&
      (e->pos() - m_press_pos).manhattanLength() < 8) {
    emit clicked(e->pos().x(), e->pos().y());
  }
  m_rotationTool->mouseReleaseEvent(e);
  m_panTool->mouseReleaseEvent(e);
  update();
}

void EglRenderWidget::wheelEvent(QWheelEvent* e) {
  m_zoomTool->wheelEvent(e);
  update();
}

// ── Fit scene to view ──────────────────────────────────────────────────────

void EglRenderWidget::fit_scene_to_view() {
  makeCurrent();
  const int w = std::max(1, width());
  const int h = std::max(1, height());
  resizeGL(w, h);

  m_root->identity_all();

  render::RenderTracks* tracks = nullptr;
  for (render::RenderObject* o : data_root_->render_objects()) {
    tracks = dynamic_cast<render::RenderTracks*>(o);
    if (tracks)
      break;
  }

  double left = 0, right = 0, bottom = 0, top = 0, near_plane = 0, far_plane = 0;
  m_camera->get_frustum(left, right, bottom, top, near_plane, far_plane);
  const double tan_half_y = ((top - bottom) * 0.5) / near_plane;
  const double tan_half_x = ((right - left) * 0.5) / near_plane;

  auto default_camera = [&]() {
    m_camera->look_at(Vec3(0, 0, 1000), Vec3(0, 0, 0), Vec3(0, 1, 0));
  };

  if (!tracks) {
    default_camera();
    update();
    return;
  }

  const Vec3 origin(0, 0, 0);
  constexpr double kPointDistanceQuantile = 0.88;
  double R = 1.0;
  if (!tracks->fit_framing_radius_about_origin(origin, kPointDistanceQuantile, &R)) {
    default_camera();
    update();
    return;
  }

  constexpr double kMargin = 1.25;
  const double dist = std::max(kMargin * R / tan_half_x, kMargin * R / tan_half_y);
  const double dist_clamped = std::max(dist, 10.0);
  m_camera->look_at(Vec3(0, 0, dist_clamped), Vec3(0, 0, 0), Vec3(0, 1, 0));
  update();
}

} // namespace insight
