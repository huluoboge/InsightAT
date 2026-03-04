/**
 * @file  database_types.cpp
 * @brief 数据库类型实现（CoordinateSystem、InputPose、Measurement、ATTask 等）。
 */

#include "database_types.h"

#include <cmath>
#include <iomanip>
#include <set>
#include <sstream>

#include <glog/logging.h>

namespace insight {
namespace database {

// ─────────────────────────────────────────────────────────────
// CoordinateSystem 实现
// ─────────────────────────────────────────────────────────────

std::string CoordinateSystem::to_string() const {
  std::ostringstream oss;

  oss << "CoordinateSystem {\n";
  oss << "  Type: ";
  switch (type) {
  case Type::kEPSG:
    oss << "EPSG";
    break;
  case Type::kWKT:
    oss << "WKT";
    break;
  case Type::kENU:
    oss << "ENU";
    break;
  case Type::kLocal:
    oss << "Local";
    break;
  }
  oss << "\n  RotationConvention: ";
  switch (rotation_convention) {

  case RotationConvention::kOmegaPhiKappa:
    oss << "OmegaPhiKappa (ω,φ,κ)";
    break;
  case RotationConvention::kYawPitchRoll:
    oss << "YawPitchRoll (Y,P,R)";
    break;
  }
  oss << "\n  Definition: " << definition << "\n";

  if (reference) {
    oss << "  Reference: lat=" << reference->lat << "°, lon=" << reference->lon
        << "°, alt=" << reference->alt << "m\n";
  }
  if (origin) {
    oss << "  Origin: (" << origin->x << ", " << origin->y << ", " << origin->z << ")\n";
  }
  oss << "}";

  return oss.str();
}

bool CoordinateSystem::is_valid() const {
  if (type == Type::kENU && !reference) {
    LOG(WARNING) << "ENU coordinate system missing reference point";
    return false;
  }
  if (reference && (reference->lat < -90.0 || reference->lat > 90.0 || reference->lon < -180.0 ||
                    reference->lon > 180.0)) {
    LOG(WARNING) << "Invalid reference point";
    return false;
  }
  return true;
}

// ─────────────────────────────────────────────────────────────
// InputPose 实现
// ─────────────────────────────────────────────────────────────

void InputPose::reset() {
  x = y = z = 0.0;
  has_position = false;
  omega = phi = kappa = 0.0;
  has_rotation = false;
  angle_unit = AngleUnit::kDegrees;
}

bool InputPose::has_data() const { return has_position || has_rotation; }

std::string InputPose::to_string() const {
  std::ostringstream oss;
  oss << "InputPose {\n";
  if (has_position) {
    oss << "  Position: (" << x << ", " << y << ", " << z << ")\n";
  }
  if (has_rotation) {
    std::string unit = (angle_unit == AngleUnit::kDegrees) ? "°" : " rad";
    oss << "  Rotation: ω=" << omega << unit << ", φ=" << phi << unit << ", κ=" << kappa << unit
        << "\n";
  }
  oss << "}";
  return oss.str();
}

bool InputPose::is_valid() const {
  if (has_position && (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))) {
    LOG(WARNING) << "InputPose has invalid position";
    return false;
  }
  if (has_rotation && (!std::isfinite(omega) || !std::isfinite(phi) || !std::isfinite(kappa))) {
    LOG(WARNING) << "InputPose has invalid rotation";
    return false;
  }
  return true;
}

// ─────────────────────────────────────────────────────────────
// Measurement 实现
// ─────────────────────────────────────────────────────────────

bool Measurement::GNSSMeasurement::is_valid() const {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    return false;
  }
  if (cov_xx < 0 || cov_yy < 0 || cov_zz < 0) {
    return false;
  }
  return true;
}

bool Measurement::IMUMeasurement::is_valid() const {
  if (has_attitude && (!std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(yaw))) {
    return false;
  }
  if (has_accel &&
      (!std::isfinite(accel_x) || !std::isfinite(accel_y) || !std::isfinite(accel_z))) {
    return false;
  }
  if (has_gyro && (!std::isfinite(gyro_x) || !std::isfinite(gyro_y) || !std::isfinite(gyro_z))) {
    return false;
  }
  return true;
}

// ─────────────────────────────────────────────────────────────
// CameraRig 实现
// ─────────────────────────────────────────────────────────────

bool CameraRig::CameraMount::is_valid() const {
  if (camera_id == static_cast<uint32_t>(-1)) {
    return false;
  }
  if (!std::isfinite(rel_x) || !std::isfinite(rel_y) || !std::isfinite(rel_z)) {
    return false;
  }
  if (!std::isfinite(rel_qx) || !std::isfinite(rel_qy) || !std::isfinite(rel_qz) ||
      !std::isfinite(rel_qw)) {
    return false;
  }
  return true;
}

std::string CameraRig::CameraMount::to_string() const {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(4);
  oss << "CameraMount {\n";
  oss << "  CameraID: " << camera_id << "\n";
  oss << "  Position: " << position_name << "\n";
  oss << "  RelPos: (" << rel_x << ", " << rel_y << ", " << rel_z << ")\n";
  oss << "  RelQuat: (" << rel_qx << ", " << rel_qy << ", " << rel_qz << ", " << rel_qw << ")\n";
  oss << "}";
  return oss.str();
}

bool CameraRig::is_valid() const {
  if (rig_id == static_cast<uint32_t>(-1)) {
    return false;
  }
  if (mounts.empty()) {
    return false;
  }

  // Check all mounts are valid
  std::set<uint32_t> seen_ids;
  for (const auto& mount : mounts) {
    if (!mount.is_valid()) {
      return false;
    }
    if (seen_ids.count(mount.camera_id)) {
      return false; // Duplicate camera_id
    }
    seen_ids.insert(mount.camera_id);
  }

  return true;
}

const CameraRig::CameraMount* CameraRig::find_camera_mount(uint32_t camera_id) const {
  for (const auto& mount : mounts) {
    if (mount.camera_id == camera_id) {
      return &mount;
    }
  }
  return nullptr;
}

std::string CameraRig::to_string() const {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(4);
  oss << "CameraRig {\n";
  oss << "  ID: " << rig_id << "\n";
  oss << "  Name: " << rig_name << "\n";
  oss << "  Status: ";
  switch (calib_status) {
  case CalibrationStatus::kUnknown:
    oss << "Unknown";
    break;
  case CalibrationStatus::kKnown:
    oss << "Known";
    break;
  case CalibrationStatus::kPartial:
    oss << "Partial";
    break;
  }
  oss << "\n  Mounts: " << mounts.size() << "\n";
  for (size_t i = 0; i < mounts.size(); ++i) {
    oss << "    [" << i << "] Camera " << mounts[i].camera_id << " @ " << mounts[i].position_name
        << "\n";
  }
  oss << "}";
  return oss.str();
}

std::string CameraRig::get_summary() const {
  std::ostringstream oss;
  oss << "Rig[id=" << rig_id << "] " << rig_name << " (" << mounts.size() << " cameras)";
  return oss.str();
}

// ─────────────────────────────────────────────────────────────
// GCPMeasurement 实现（独立结构）
// ─────────────────────────────────────────────────────────────

bool GCPMeasurement::is_valid() const {
  // 检查GCP ID
  if (gcp_id == static_cast<uint32_t>(-1)) {
    return false;
  }

  // 检查3D坐标有效性
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    return false;
  }

  // 检查必须至少有一个观测
  if (observations.empty()) {
    return false;
  }

  // 检查每个观测的有效性
  for (const auto& obs : observations) {
    if (obs.image_id == static_cast<uint32_t>(-1)) {
      return false;
    }
    if (!std::isfinite(obs.pixel_x) || !std::isfinite(obs.pixel_y)) {
      return false;
    }
  }

  return true;
}

// ─────────────────────────────────────────────────────────────
// Measurement 实现
// ─────────────────────────────────────────────────────────────

bool Measurement::SLAMMeasurement::is_valid() const {
  if (reference_image_id == static_cast<uint32_t>(-1)) {
    return false;
  }
  if (!std::isfinite(rel_x) || !std::isfinite(rel_y) || !std::isfinite(rel_z)) {
    return false;
  }
  double quat_norm =
      std::sqrt(rel_qx * rel_qx + rel_qy * rel_qy + rel_qz * rel_qz + rel_qw * rel_qw);
  if (std::abs(quat_norm - 1.0) > 0.01) {
    return false;
  }
  if (confidence < 0.0 || confidence > 1.0) {
    return false;
  }
  return true;
}

std::string Measurement::to_string() const {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6);

  oss << "Measurement {\n";
  oss << "  Type: ";
  switch (type) {
  case Type::kGNSS:
    oss << "GNSS";
    break;
  case Type::kIMU:
    oss << "IMU";
    break;
  case Type::kGCP:
    oss << "GCP (deprecated, use Project.gcp_database)";
    break;
  case Type::kSLAM:
    oss << "SLAM";
    break;
  case Type::kOther:
    oss << "Other";
    break;
  }
  oss << "\n  ImageID: " << image_id << "\n  Timestamp: " << timestamp << " ms\n";

  if (gnss) {
    oss << "  GNSS: pos=(" << gnss->x << "," << gnss->y << "," << gnss->z
        << "), sats=" << static_cast<int>(gnss->num_satellites) << "\n";
  }
  if (imu) {
    oss << "  IMU: att=" << imu->has_attitude << ", accel=" << imu->has_accel
        << ", gyro=" << imu->has_gyro << "\n";
  }
  if (slam) {
    oss << "  SLAM: ref_id=" << slam->reference_image_id << ", conf=" << slam->confidence << "\n";
  }
  oss << "}";

  return oss.str();
}

bool Measurement::is_valid() const {
  if (image_id == static_cast<uint32_t>(-1) || timestamp < 0) {
    return false;
  }

  switch (type) {
  case Type::kGNSS:
    return gnss && gnss->is_valid();
  case Type::kIMU:
    return imu && imu->is_valid();
  case Type::kGCP:
    // GCP now stored in Project, not in Measurement
    return true; // Legacy type, no validation needed
  case Type::kSLAM:
    return slam && slam->is_valid();
  case Type::kOther:
    return true;
  default:
    return false;
  }
}

// ─────────────────────────────────────────────────────────────
// CameraModel 实现
// ─────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────
// Image 实现
// ─────────────────────────────────────────────────────────────

std::string Image::to_string() const {
  std::ostringstream oss;
  oss << "Image {\n";
  oss << "  ID: " << image_id << "\n";
  oss << "  File: " << filename << "\n";
  oss << "  InputPose: " << (input_pose.has_data() ? "present" : "none") << "\n";
  if (camera) {
    oss << "  Camera: " << camera->camera_name << "\n";
  }
  oss << "}";
  return oss.str();
}

bool Image::is_valid() const {
  if (image_id == 0) {
    LOG(WARNING) << "Image has invalid ID (0)";
    return false;
  }
  if (filename.empty()) {
    LOG(WARNING) << "Image has empty filename";
    return false;
  }
  if (camera && !camera->is_valid()) {
    LOG(WARNING) << "Image has invalid camera model";
    return false;
  }
  return true;
}

// ─────────────────────────────────────────────────────────────
// ImageGroup 实现
// ─────────────────────────────────────────────────────────────

void ImageGroup::apply_camera_model(const CameraModel& camera, CameraMode mode) {
  camera_mode = mode;

  if (mode == CameraMode::kGroupLevel) {
    // 组级模式：设置group_camera
    group_camera = camera;
  } else {
    // 图像级模式：为所有未设置相机的图像应用
    for (auto& img : images) {
      if (!img.camera) {
        img.camera = camera;
      }
    }
  }
}

const CameraModel* ImageGroup::get_camera_for_image(uint32_t image_id) const {
  if (camera_mode == CameraMode::kGroupLevel) {
    // 组级模式：返回group_camera
    if (group_camera) {
      return &group_camera.value();
    }
    return nullptr;
  } else {
    // 图像级模式：查找图像自己的相机
    for (const auto& img : images) {
      if (img.image_id == image_id && img.camera) {
        return &img.camera.value();
      }
    }
    return nullptr;
  }
}

bool ImageGroup::add_image(const Image& image) {
  if (image.image_id == 0) {
    LOG(WARNING) << "Cannot add image with ID 0";
    return false;
  }

  // 检查ID重复
  for (const auto& img : images) {
    if (img.image_id == image.image_id) {
      LOG(WARNING) << "Image ID " << image.image_id << " already exists in group";
      return false;
    }
  }

  images.push_back(image);
  return true;
}

int ImageGroup::find_image_index(uint32_t image_id) const {
  for (size_t i = 0; i < images.size(); ++i) {
    if (images[i].image_id == image_id) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

bool ImageGroup::is_valid() const {
  if (images.empty()) {
    LOG(WARNING) << "ImageGroup has no images";
    return false;
  }

  // 检查所有image_id唯一
  std::set<uint32_t> ids;
  for (const auto& img : images) {
    if (ids.count(img.image_id)) {
      LOG(WARNING) << "Duplicate image ID: " << img.image_id;
      return false;
    }
    ids.insert(img.image_id);
  }

  if (camera_mode == CameraMode::kGroupLevel) {
    // 组级模式：必须有group_camera
    if (!group_camera || !group_camera->is_valid()) {
      LOG(WARNING) << "ImageGroup in GroupLevel mode missing valid camera";
      return false;
    }
  } else {
    // 图像级模式：每个图像必须有相机参数
    for (const auto& img : images) {
      if (!img.camera || !img.camera->is_valid()) {
        LOG(WARNING) << "Image " << img.image_id << " missing valid camera in ImageLevel mode";
        return false;
      }
    }
  }

  return true;
}

std::string ImageGroup::to_string() const {
  std::ostringstream oss;
  oss << "ImageGroup {\n";
  oss << "  ID: " << group_id << "\n";
  oss << "  Name: " << (group_name.empty() ? "(unnamed)" : group_name) << "\n";
  oss << "  Mode: " << (camera_mode == CameraMode::kGroupLevel ? "GroupLevel" : "ImageLevel")
      << "\n";
  oss << "  Images: " << images.size() << "\n";
  if (group_camera) {
    oss << "  GroupCamera: " << group_camera->camera_name << "\n";
  }
  oss << "}";
  return oss.str();
}

bool ImageGroup::convert_to_image_level() {
  if (camera_mode == CameraMode::kImageLevel) {
    return true; // 已经是图像级模式
  }

  if (!group_camera) {
    LOG(ERROR) << "Cannot convert to ImageLevel: no group camera available";
    return false;
  }

  // 为所有未设置相机的图像应用group_camera
  for (auto& img : images) {
    if (!img.camera) {
      img.camera = group_camera.value();
    }
  }

  camera_mode = CameraMode::kImageLevel;
  group_camera.reset();

  return true;
}

bool ImageGroup::convert_to_group_level() {
  if (camera_mode == CameraMode::kGroupLevel) {
    return true; // 已经是组级模式
  }

  if (images.empty()) {
    LOG(ERROR) << "Cannot convert to GroupLevel: no images";
    return false;
  }

  // 检查所有图像的相机参数是否相同
  const CameraModel* first_camera = nullptr;
  if (images[0].camera) {
    first_camera = &images[0].camera.value();
  }

  for (size_t i = 1; i < images.size(); ++i) {
    if (!images[i].camera) {
      LOG(ERROR) << "Image " << i << " has no camera in ImageLevel mode";
      return false;
    }

    // 简单比较：相机名称和焦距相同
    if (first_camera) {
      if (images[i].camera->camera_name != first_camera->camera_name ||
          std::abs(images[i].camera->focal_length - first_camera->focal_length) > 1e-6) {
        LOG(ERROR) << "Images have different camera parameters";
        return false;
      }
    }
  }

  if (first_camera) {
    group_camera = *first_camera;
  }

  // 清空所有图像的相机参数
  for (auto& img : images) {
    img.camera.reset();
  }

  camera_mode = CameraMode::kGroupLevel;
  return true;
}

// ─────────────────────────────────────────────────────────────
// ATTask 实现
// ─────────────────────────────────────────────────────────────

const CameraModel* ATTask::get_camera_for_image(uint32_t group_id, uint32_t image_id) const {
  for (const auto& group : input_snapshot.image_groups) {
    if (group.group_id == group_id) {
      return group.get_camera_for_image(image_id);
    }
  }
  LOG(WARNING) << "Group " << group_id << " not found in ATTask";
  return nullptr;
}

const ImageGroup* ATTask::find_group_by_image_id(uint32_t image_id) const {
  for (const auto& group : input_snapshot.image_groups) {
    if (group.find_image_index(image_id) >= 0) {
      return &group;
    }
  }
  return nullptr;
}

size_t ATTask::get_total_image_count() const {
  size_t count = 0;
  for (const auto& group : input_snapshot.image_groups) {
    count += group.images.size();
  }
  return count;
}

std::string ATTask::to_string() const {
  std::ostringstream oss;
  oss << "ATTask {\n";
  oss << "  ID: " << id << "\n";
  oss << "  InputSnapshot: " << input_snapshot.measurements.size() << " measurements\n";
  oss << "  ImageGroups: " << input_snapshot.image_groups.size() << " groups\n";

  size_t total_images = get_total_image_count();
  oss << "  TotalImages: " << total_images << "\n";

  for (const auto& group : input_snapshot.image_groups) {
    oss << "    - Group " << group.group_id << ": " << group.images.size() << " images, ";
    oss << "mode="
        << (group.camera_mode == ImageGroup::CameraMode::kGroupLevel ? "GroupLevel" : "ImageLevel")
        << "\n";
  }

  oss << "  OptimizedPoses: " << optimized_poses.size() << "\n";
  oss << "}";
  return oss.str();
}

// ─────────────────────────────────────────────────────────────
// CameraModel 实现
// ─────────────────────────────────────────────────────────────

bool CameraModel::is_valid() const {
  // 基本检查：分辨率
  if (width == 0 || height == 0) {
    return false;
  }

  // 检查焦距有效性
  if (focal_length <= 0.0) {
    return false;
  }

  // 焦距合理范围检查（相对于图像宽度）
  // 典型范围：0.5x - 5.0x 宽度
  double focal_ratio = focal_length / width;
  if (focal_ratio < 0.3 || focal_ratio > 10.0) {
    return false;
  }

  // 检查主点在合理范围内
  if (principal_point_x < -width || principal_point_x > width * 2.0 ||
      principal_point_y < -height || principal_point_y > height * 2.0) {
    return false;
  }

  // 检查宽高比合理
  if (aspect_ratio <= 0.0 || aspect_ratio > 5.0) {
    return false;
  }

  return true;
}

std::string CameraModel::to_string() const {
  std::ostringstream oss;

  oss << "CameraModel {\n";

  // 类型和分辨率
  oss << "  Type: ";
  switch (type) {
  case Type::kPinhole:
    oss << "Pinhole";
    break;
  case Type::kBrownConrady:
    oss << "Brown-Conrady";
    break;
  case Type::kSimpleDistortion:
    oss << "Simple Distortion";
    break;
  case Type::kFisheye:
    oss << "Fisheye";
    break;
  case Type::kOther:
    oss << "Other";
    break;
  }
  oss << "\n  Resolution: " << width << " x " << height << " pixels\n";

  // 传感器物理信息
  if (sensor_width_mm > 0 && sensor_height_mm > 0) {
    oss << "  Sensor: " << sensor_width_mm << " x " << sensor_height_mm << " mm\n";
    oss << "  Pixel Size: " << pixel_size_um << " µm\n";
  }
  if (focal_length_35mm > 0) {
    oss << "  Focal Length (35mm equivalent): " << focal_length_35mm << " mm\n";
  }

  // 内参
  oss << "  Intrinsics:\n";
  oss << "    Focal Length: " << focal_length << " px\n";
  oss << "    Principal Point: (" << principal_point_x << ", " << principal_point_y << ") px\n";
  if (aspect_ratio != 1.0) {
    oss << "    Aspect Ratio (fy/fx): " << aspect_ratio << "\n";
  }
  if (skew != 0.0) {
    oss << "    Skew: " << skew << "\n";
  }

  // 畸变参数
  if (has_distortion()) {
    oss << "  Distortion:\n";
    oss << "    Radial: k1=" << k1 << ", k2=" << k2 << ", k3=" << k3 << "\n";
    oss << "    Tangential: p1=" << p1 << ", p2=" << p2 << "\n";
  }

  // 元数据
  if (!camera_name.empty()) {
    oss << "  Camera Name: " << camera_name << "\n";
  }
  if (!make.empty()) {
    oss << "  Make: " << make;
    if (!model.empty()) {
      oss << " " << model;
    }
    oss << "\n";
  }
  if (!lens_model.empty()) {
    oss << "  Lens: " << lens_model << "\n";
  }

  // 优化标记
  if (optimization_flags.focal_length || optimization_flags.principal_point_x ||
      optimization_flags.principal_point_y || optimization_flags.k1 || optimization_flags.k2 ||
      optimization_flags.k3 || optimization_flags.p1 || optimization_flags.p2) {
    oss << "  Optimization: ";
    std::vector<std::string> flags;
    if (optimization_flags.focal_length)
      flags.push_back("f");
    if (optimization_flags.principal_point_x)
      flags.push_back("ppx");
    if (optimization_flags.principal_point_y)
      flags.push_back("ppy");
    if (optimization_flags.k1)
      flags.push_back("k1");
    if (optimization_flags.k2)
      flags.push_back("k2");
    if (optimization_flags.k3)
      flags.push_back("k3");
    if (optimization_flags.p1)
      flags.push_back("p1");
    if (optimization_flags.p2)
      flags.push_back("p2");

    for (size_t i = 0; i < flags.size(); ++i) {
      if (i > 0)
        oss << ", ";
      oss << flags[i];
    }
    oss << "\n";
  }

  oss << "}";
  return oss.str();
}

// ─────────────────────────────────────────────────────────────
// Project 实现
// ─────────────────────────────────────────────────────────────

size_t Project::get_total_image_count() const {
  size_t count = 0;
  for (const auto& group : image_groups) {
    count += group.images.size();
  }
  return count;
}

size_t Project::get_measurement_count_by_type(Measurement::Type type) const {
  size_t count = 0;
  for (const auto& measurement : measurements) {
    if (measurement.type == type) {
      count++;
    }
  }
  return count;
}

bool Project::is_valid() const {
  // 检查基本元数据
  if (name.empty()) {
    LOG(WARNING) << "Project has empty name";
    return false;
  }
  if (uuid.empty()) {
    LOG(WARNING) << "Project has empty UUID";
    return false;
  }
  if (author.empty()) {
    LOG(WARNING) << "Project has empty author";
    return false;
  }
  if (creation_time <= 0) {
    LOG(WARNING) << "Project has invalid creation time";
    return false;
  }

  // 检查输入坐标系
  if (!input_coordinate_system.is_valid()) {
    LOG(WARNING) << "Project has invalid input coordinate system";
    return false;
  }

  // 检查所有测量数据
  for (const auto& measurement : measurements) {
    if (!measurement.is_valid()) {
      LOG(WARNING) << "Project has invalid measurement";
      return false;
    }
  }

  // 检查所有图像分组
  for (const auto& group : image_groups) {
    if (!group.is_valid()) {
      LOG(WARNING) << "Project has invalid image group: " << group.group_id;
      return false;
    }
  }

  // 检查初始位姿（如果存在）
  if (initial_pose && !initial_pose->is_valid()) {
    LOG(WARNING) << "Project has invalid initial pose";
    return false;
  }

  return true;
}

std::string Project::to_string() const {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2);

  oss << "Project {\n";
  oss << "  Name: " << name << "\n";
  oss << "  UUID: " << uuid << "\n";
  oss << "  Author: " << author << "\n";
  oss << "  CreationTime: " << creation_time << " (Unix timestamp)\n";
  oss << "  LastModified: " << last_modified_time << " (Unix timestamp)\n";
  oss << "  Version: " << project_version << "\n";

  if (!description.empty()) {
    std::string desc = description;
    if (desc.length() > 50) {
      desc = desc.substr(0, 50) + "...";
    }
    oss << "  Description: " << desc << "\n";
  }

  if (!tags.empty()) {
    oss << "  Tags: ";
    for (size_t i = 0; i < tags.size(); ++i) {
      if (i > 0)
        oss << ", ";
      oss << tags[i];
    }
    oss << "\n";
  }

  oss << "  InputCoordinateSystem: " << input_coordinate_system.to_string().substr(0, 30) << "...\n";
  oss << "  Measurements: " << measurements.size() << "\n";
  oss << "    - GNSS: " << get_measurement_count_by_type(Measurement::Type::kGNSS) << "\n";
  oss << "    - IMU: " << get_measurement_count_by_type(Measurement::Type::kIMU) << "\n";
  oss << "    - GCP: " << get_measurement_count_by_type(Measurement::Type::kGCP) << "\n";
  oss << "    - SLAM: " << get_measurement_count_by_type(Measurement::Type::kSLAM) << "\n";

  oss << "  ImageGroups: " << image_groups.size() << "\n";
  size_t total_images = get_total_image_count();
  oss << "  TotalImages: " << total_images << "\n";

  for (const auto& group : image_groups) {
    oss << "    - Group " << group.group_id << ": " << group.images.size() << " images, ";
    oss << "mode="
        << (group.camera_mode == ImageGroup::CameraMode::kGroupLevel ? "GroupLevel" : "ImageLevel")
        << "\n";
  }

  if (initial_pose) {
    oss << "  InitialPose: present\n";
  }

  oss << "  ATTasks: " << at_tasks.size() << "\n";
  oss << "}";

  return oss.str();
}

std::string Project::get_summary() const {
  std::ostringstream oss;
  oss << name << " (UUID: " << uuid.substr(0, 8) << "...)\n";
  oss << "  Images: " << get_total_image_count() << " | ";
  oss << "Measurements: " << measurements.size() << " | ";
  oss << "Author: " << author << "\n";
  oss << "  GNSS: " << get_measurement_count_by_type(Measurement::Type::kGNSS);
  oss << " | IMU: " << get_measurement_count_by_type(Measurement::Type::kIMU);
  oss << " | GCP: " << get_measurement_count_by_type(Measurement::Type::kGCP);
  oss << " | SLAM: " << get_measurement_count_by_type(Measurement::Type::kSLAM) << "\n";

  return oss.str();
}

const ImageGroup* Project::find_group_by_image_id(uint32_t image_id) const {
  for (const auto& group : image_groups) {
    if (group.find_image_index(image_id) >= 0) {
      return &group;
    }
  }
  return nullptr;
}

const CameraModel* Project::get_camera_for_image_id(uint32_t image_id) const {
  const ImageGroup* group = find_group_by_image_id(image_id);
  if (group) {
    return group->get_camera_for_image(image_id);
  }
  return nullptr;
}

// ─────────────────────────────────────────────────────────────
// Project GCP 缓存和查询实现
// ─────────────────────────────────────────────────────────────

std::vector<uint32_t> Project::get_gcps_for_image(uint32_t image_id) const {
  // 如果缓存无效，重建缓存
  if (!cache_valid) {
    rebuild_gcp_cache();
  }

  // 查询缓存
  auto it = image_to_gcp_cache.find(image_id);
  if (it != image_to_gcp_cache.end()) {
    return it->second;
  }

  return {}; // 返回空向量
}

const GCPMeasurement* Project::get_gcp(uint32_t gcp_id) const {
  auto it = gcp_database.find(gcp_id);
  if (it != gcp_database.end()) {
    return &(it->second);
  }
  return nullptr;
}

void Project::invalidate_gcp_cache() const {
  cache_valid = false;
  image_to_gcp_cache.clear();
}

bool Project::rebuild_gcp_cache() const {
  image_to_gcp_cache.clear();

  // 遍历所有GCP，构建索引
  for (const auto& [gcp_id, gcp_meas] : gcp_database) {
    // 遍历每个观测
    for (const auto& obs : gcp_meas.observations) {
      image_to_gcp_cache[obs.image_id].push_back(gcp_id);
    }
  }

  cache_valid = true;
  return true;
}

// ─────────────────────────────────────────────────────────────
// Project CameraRig 相关实现
// ─────────────────────────────────────────────────────────────

const CameraRig* Project::get_camera_rig(uint32_t rig_id) const {
  auto it = camera_rigs.find(rig_id);
  if (it != camera_rigs.end()) {
    return &(it->second);
  }
  return nullptr;
}

const CameraModel* Project::get_camera_for_rig_mount(uint32_t rig_id, uint32_t camera_id) const {
  const auto* rig = get_camera_rig(rig_id);
  if (!rig) {
    return nullptr;
  }

  // 在Rig中查找该相机的挂载点
  const auto* mount = rig->find_camera_mount(camera_id);
  if (!mount) {
    return nullptr;
  }

  // 查找该相机的相机参数
  // 首先在camera_models中查找，或者在image_groups中查找
  for (const auto& group : image_groups) {
    if (group.camera_mode == ImageGroup::CameraMode::kRigBased && group.rig_mount_info &&
        group.rig_mount_info->rig_id == rig_id && group.rig_mount_info->camera_id == camera_id) {
      return group.group_camera ? &(*group.group_camera) : nullptr;
    }
  }

  return nullptr;
}

bool Project::validate_rig(uint32_t rig_id) const {
  const auto* rig = get_camera_rig(rig_id);
  if (!rig || !rig->is_valid()) {
    return false;
  }

  // 检查所有挂载的相机是否有相机参数
  for (const auto& mount : rig->mounts) {
    if (!get_camera_for_rig_mount(rig_id, mount.camera_id)) {
      return false;
    }
  }

  return true;
}

bool CameraModel::has_distortion() const {
  return (k1 != 0.0 || k2 != 0.0 || k3 != 0.0 || p1 != 0.0 || p2 != 0.0);
}

} // namespace database
} // namespace insight
