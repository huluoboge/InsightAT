#include "render_node.h"
#include "opengl_mat.h"
#include "render_context.h"
#include "render_object.h"
#include <algorithm>
#include <cassert>
#include <list>

namespace insight {

namespace render {
RenderNode::RenderNode(RenderNode* parent /*= NULL*/) {
  parent_node_ = parent;
  model_matrix_ = Mat4::Identity();
  if (parent != NULL) {
    parent->add_child_node(this);
  }
}

void RenderNode::rotate(double angle, double x, double y, double z) {
  model_matrix_ *= rotation_matrix_deg(angle, x, y, z);
}

void RenderNode::rotate_x(double angle) { model_matrix_ *= rotation_matrix_deg(angle, 1, 0, 0); }

void RenderNode::rotate_y(double angle) { model_matrix_ *= rotation_matrix_deg(angle, 0, 1, 0); }

void RenderNode::rotate_z(double angle) { model_matrix_ *= rotation_matrix_deg(angle, 0, 0, 1); }

void RenderNode::rotate_x_by_pos(double angle, double y, double z) {
  Vec3 pos1 = local_to_world(0.0, y, z);
  model_matrix_ *= rotation_matrix_deg(angle, 1, 0, 0);
  Vec3 pos2 = local_to_world(0.0, y, z);
  translate(pos1 - pos2);
}

void RenderNode::rotate_y_by_pos(double angle, double x, double z) {
  Vec3 pos1 = local_to_world(x, 0.0, z);
  model_matrix_ *= rotation_matrix_deg(angle, 0, 1, 0);
  Vec3 pos2 = local_to_world(x, 0.0, z);
  translate(pos1 - pos2);
}

void RenderNode::rotate_z_by_pos(double angle, double x, double y) {
  Vec3 pos1 = local_to_world(x, y, 0.0);
  model_matrix_ *= rotation_matrix_deg(angle, 0, 0, 1);
  Vec3 pos2 = local_to_world(x, y, 0.0);
  translate(pos1 - pos2);
}

void RenderNode::scale(double ratio) {
  Vec3 pos = position();
  translate(-pos);
  model_matrix_ *= scaling_matrix(ratio, ratio, ratio);
  translate(pos);
  // m_scale *= ratio;
}

void RenderNode::scale(double sx, double sy, double sz) {
  Vec3 pos = position();
  translate(-pos);
  model_matrix_ *= scaling_matrix(sx, sy, sz);
  translate(pos);
}

Vec3 RenderNode::get_scale() const {
  double sx = x().norm();
  double sy = y().norm();
  double sz = z().norm();
  return Vec3(sx, sy, sz);
}

void RenderNode::set_scale(double sx, double sy, double sz) {
  Vec3 s = get_scale();
  sx = sx / s.x();
  sy = sy / s.y();
  sz = sz / s.z();
  scale(sx, sy, sz);
}

void RenderNode::scale_by_pos(double ratio, double x, double y, double z) {
  Vec3 pos = local_to_world(x, y, z);
  model_matrix_ *= scaling_pos_matrix(pos.x(), pos.y(), pos.z(), ratio, ratio, ratio);
}

Mat4 RenderNode::local_to_world_mat() const {
  std::vector<RenderNode*> parents = parent_nodes();
  Mat4 mat = Mat4::Identity();
  for (int i = 0; i < parents.size(); ++i) {
    mat *= parents[i]->ref_model_matrix();
  }
  mat *= model_matrix_;
  return mat;
}

Vec3 RenderNode::local_to_world(double x, double y, double z) {
  std::vector<RenderNode*> parents = parent_nodes();
  Mat4 mat = Mat4::Identity();
  for (int i = 0; i < parents.size(); ++i) {
    mat *= parents[i]->ref_model_matrix();
  }
  mat *= model_matrix_;

  return (mat * Vec4(x, y, z, 1.0)).head<3>();
}

Vec3 RenderNode::world_to_local(double x, double y, double z) {
  std::vector<RenderNode*> parents = parent_nodes();
  Mat4 mat = Mat4::Identity();
  for (int i = 0; i < parents.size(); ++i) {
    mat *= parents[i]->ref_model_matrix();
  }
  mat *= model_matrix_;
  return (mat.inverse() * Vec4(x, y, z, 1.0)).head<3>();
}

Vec3 RenderNode::fast_world_to_local(double x, double y, double z) {
  std::vector<RenderNode*> parents = parent_nodes();
  Mat4 mat = Mat4::Identity();
  for (int i = 0; i < parents.size(); ++i) {
    mat *= parents[i]->ref_model_matrix();
  }
  mat *= model_matrix_;
  return (fast_inverse(mat) * Vec4(x, y, z, 1.0)).head<3>();
}

Vec3 RenderNode::local_to_parent(double x, double y, double z) {
  return (model_matrix_ * Vec4(x, y, z, 1.0)).head<3>();
}

Vec3 RenderNode::parent_to_local(double x, double y, double z) {
  return (model_matrix_.inverse() * Vec4(x, y, z, 1.0)).head<3>();
}

Vec3 RenderNode::fast_parent_to_local(double x, double y, double z) {
  return (fast_inverse(model_matrix_) * Vec4(x, y, z, 1.0)).head<3>();
}

void RenderNode::render(RenderContext* rc) {
  if (!isVisible()) {
    return;
  }
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  rc->push();
  rc->modelview *= model_matrix_;
  glLoadMatrixd(rc->modelview.data());
  glPushAttrib(GL_ALL_ATTRIB_BITS);
  draw(rc);
  for (int i = 0; i < (int)child_nodes_.size(); ++i) {
    child_nodes_[i]->render(rc);
  }
  glPopAttrib();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
  rc->pop();
}

void RenderNode::add_child_node(RenderNode* childNode) { child_nodes_.push_back(childNode); }

RenderNode::~RenderNode() {
  destroy_all_child_nodes();
  destroy_all_objects();
}

void RenderNode::destroy_all_child_nodes() {
  for (int i = 0; i < (int)child_nodes_.size(); ++i) {
    delete child_nodes_[i];
  }
  child_nodes_.clear();
}

std::vector<RenderNode*> RenderNode::parent_nodes() const {
  std::list<RenderNode*> parents;
  RenderNode* parent = parent_node_;
  while (parent != NULL) {
    parents.push_front(parent);
    parent = parent->parent_node_;
  }

  std::vector<RenderNode*> parentObjs;
  parentObjs.resize(parents.size());
  std::copy(parents.begin(), parents.end(), parentObjs.begin());
  return parentObjs;
}

void RenderNode::destroy_all_objects() {
  for (int i = 0; i < (int)render_objects_.size(); ++i) {
    delete render_objects_[i];
  }
  render_objects_.clear();
}

void RenderNode::set_parent_node(RenderNode* parent) {
  if (parent_node_ != NULL) {
    parent_node_->remove_child_node(this);
  }
  if (parent != NULL) {
    parent->add_child_node(this);
  }
  parent_node_ = parent;
}

void RenderNode::remove_child_node(RenderNode* childObj) {
  std::vector<RenderNode*>::iterator itr =
      std::find(child_nodes_.begin(), child_nodes_.end(), childObj);
  if (itr != child_nodes_.end()) {
    child_nodes_.erase(itr);
  }
}

void RenderNode::clear_all_child_nodes() { child_nodes_.clear(); }

void RenderNode::identity_all() {
  model_matrix_ = Mat4::Identity();
  for (int i = 0; i < child_nodes_.size(); ++i) {
    child_nodes_[i]->identity_all();
  }
}

void RenderNode::draw(RenderContext* rc) {
  for (size_t i = 0; i < render_objects_.size(); ++i) {
    render_objects_[i]->draw(rc);
  }
}

} // namespace render

} // namespace insight
