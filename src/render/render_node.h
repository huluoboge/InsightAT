#pragma once

#include "render_global.h"

#include "render_object.h"
#include "render_types.h"
#include <QString>
#include <QVariant>
#include <vector>

namespace insight {

namespace render {
class RenderContext;
class RenderObject;
/**
 * @brief 3D??????????????????????
 */
class RENDER_EXPORT RenderNode : public RenderObject {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit RenderNode(RenderNode* parent = NULL);

public:
  virtual ~RenderNode();

public:
  Mat4 model_matrix() const { return model_matrix_; }
  Mat4& ref_model_matrix() { return model_matrix_; }
  const Mat4& ref_model_matrix() const { return model_matrix_; }

  void identity_all();
  inline Vec3 x() const { return model_matrix_.col(0).head<3>(); }
  inline Vec3 y() const { return model_matrix_.col(1).head<3>(); }
  inline Vec3 z() const { return model_matrix_.col(2).head<3>(); }

  inline Vec3 position() { return model_matrix_.col(3).head<3>(); }
  inline void set_position(const Vec3& pos) {
    model_matrix_(0, 3) = pos.x();
    model_matrix_(1, 3) = pos.y();
    model_matrix_(2, 3) = pos.z();
  }

  inline void set_position(double x, double y, double z) { set_position(Vec3(x, y, z)); }

  inline void position(double& x, double& y, double& z) {
    Vec3 pos = position();
    x = pos.x();
    y = pos.y();
    z = pos.z();
  }

  void translate(const Vec3& v) { translate(v.x(), v.y(), v.z()); }

  void translate(double x, double y, double z) {
    model_matrix_(0, 3) += x;
    model_matrix_(1, 3) += y;
    model_matrix_(2, 3) += z;
  }

  void rotate_x(double angle);
  void rotate_y(double angle);
  void rotate_z(double angle);
  void rotate(double angle, double x, double y, double z); // ??x,y,z???????????????angle???

  void rotate_x_by_pos(double angle, double y, double z);

  void rotate_y_by_pos(double angle, double x, double z);

  void rotate_z_by_pos(double angle, double x, double y);

  void scale(double ratio);

  void scale(double sx, double sy, double sz);

  //???????????????
  Vec3 get_scale() const;

  //????????????????
  void set_scale(double sx, double sy, double sz);

  void scale_by_pos(double ratio, double x, double y, double z);

  inline void scale_by_pos(double ratio, const Vec3& pos) {
    return scale_by_pos(ratio, pos.x(), pos.y(), pos.z());
  }

  Vec3 local_to_world(double x, double y, double z);
  inline Vec3 local_to_world(const Vec3& pos) { return local_to_world(pos.x(), pos.y(), pos.z()); }

  Vec3 world_to_local(double x, double y, double z);
  inline Vec3 world_to_local(const Vec3& pos) { return world_to_local(pos.x(), pos.y(), pos.z()); }

  //????????????????????????????????????????????????????????????хн
  Vec3 fast_world_to_local(double x, double y, double z);
  inline Vec3 fast_world_to_local(const Vec3& pos) {
    return fast_world_to_local(pos.x(), pos.y(), pos.z());
  }

  Vec3 local_to_parent(double x, double y, double z);
  inline Vec3 local_to_parent(const Vec3& pos) {
    return local_to_parent(pos.x(), pos.y(), pos.z());
  }

  Vec3 parent_to_local(double x, double y, double z);
  inline Vec3 parent_to_local(const Vec3& pos) {
    return parent_to_local(pos.x(), pos.y(), pos.z());
  }

  //??????????????????????????????????????????????????????
  Vec3 fast_parent_to_local(double x, double y, double z);
  inline Vec3 fast_parent_to_local(const Vec3& pos) {
    return fast_parent_to_local(pos.x(), pos.y(), pos.z());
  }

  // mat * vec(local) = vec(world)
  Mat4 local_to_world_mat() const;

  std::vector<RenderObject*>& render_objects() { return render_objects_; }

public:
  void add_child_node(RenderNode* child);

  RenderNode* parent_node() { return parent_node_; }
  void set_parent_node(RenderNode* parent);

  const std::vector<RenderNode*>& child_nodes() const { return child_nodes_; }
  std::vector<RenderNode*> child_nodes() { return child_nodes_; }

  int child_node_count() const { return int(child_nodes_.size()); }
  bool have_child_node() const { return !child_nodes_.empty(); }

  void destroy_all_child_nodes();                // delete all child
  void remove_child_node(RenderNode* childNode); // just remove ,not delete
  void clear_all_child_nodes();                  // remove all

  std::vector<RenderNode*> parent_nodes() const;

  void destroy_all_objects();

public:
  virtual void render(RenderContext* rc);

  virtual void draw(RenderContext* rc);

protected:
  friend class RenderContext;
  Mat4 model_matrix_;
  std::vector<RenderNode*> child_nodes_;
  std::vector<RenderObject*> render_objects_;
  RenderNode* parent_node_;
};

} // namespace render

} // namespace insight
