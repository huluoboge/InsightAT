#pragma once

#include "render_global.h"
#include "render_node.h"
#include "render_object.h"
#include "render_types.h"

/************************************************************************/
/*
ùù?ùù?ùù
*/
/************************************************************************/

namespace insight {

namespace render {
class RENDER_EXPORT RenderPivot : public RenderObject {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RenderPivot();

  void set_no_scale(bool no_scale) { no_scale_ = no_scale; }
  bool is_no_scale() const { return no_scale_; }

  virtual void draw(RenderContext* rc);

  Mat4& ref_model_matrix() { return model_matrix_; }

  void update_matrix(const Mat4& mat);

private:
  // draw a unit circle in a given plane (0=YZ, 1 = XZ, 2=XY)
  void gl_draw_unit_circle(int dim, unsigned steps = 64);

  bool no_scale_;
  Mat4 model_matrix_;
};

} // namespace render

} // namespace insight
