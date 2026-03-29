#pragma once
#include "render_global.h"
#include "render_types.h"

#include <QGLWidget>

namespace insight {

namespace render {
class RenderCamera;
class RenderContext {
public:
  int w = 0;
  int h = 0;
  RenderCamera* camera = nullptr; //
  QGLWidget* widget = nullptr;    //

  // opengl modelview
  Mat4 modelview;

  void push() { modelview_stack_.push_back(modelview); }
  void pop() {
    modelview = modelview_stack_.back();
    modelview_stack_.pop_back();
  }
  void clear() { modelview_stack_.clear(); }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
  QList<Mat4> modelview_stack_;
};
} // namespace render

} // namespace insight
