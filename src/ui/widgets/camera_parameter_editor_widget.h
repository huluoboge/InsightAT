/**
 * @file  CameraParameterEditorWidget.h
 * @brief 相机参数编辑器，支持 Brown-Conrady 与 Group/Image/Rig 模式。
 */

#pragma once

#include "database/database_types.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QSpinBox>
#include <QWidget>

namespace insight::ui::widgets {

/**
 * @brief 相机参数编辑器 Widget - 可复用于多个界面（ImageGroup、AT Task等）
 *
 * 支持与 Bentley ContextCapture 一致的 5 参数畸变模型：
 * - 9 个内参：焦距(px)、主点(px)、图像宽高(px)、传感器宽高(mm)、焦距(mm)、35mm等效焦距
 * - 5 个畸变参数：k1, k2, k3, p1, p2（与 Bentley K₁,K₂,K₃,P₁,P₂ 对应）
 * - 3 种相机模式：GroupLevel / ImageLevel / RigBased
 *
 * 失焦时自动保存数据（通过 field_modified() 信号）
 */
class CameraParameterEditorWidget : public QWidget {
  Q_OBJECT

public:
  explicit CameraParameterEditorWidget(QWidget* parent = nullptr);
  ~CameraParameterEditorWidget();

  void load_camera(const database::CameraModel& camera);
  database::CameraModel get_camera() const;
  void set_group_name(const std::string& name);
  std::string get_group_name() const;

  void show_group_name_field(bool show);
  void set_editable(bool editable);
  void set_mode(database::ImageGroup::CameraMode mode);
  database::ImageGroup::CameraMode get_mode() const;

signals:
  void field_modified();
  void mode_changed(database::ImageGroup::CameraMode mode);
  void auto_estimate_requested();

private slots:
  void on_focal_length_px_edited();
  void on_principal_point_x_edited();
  void on_principal_point_y_edited();
  void on_image_width_edited();
  void on_image_height_edited();
  void on_sensor_width_mm_edited();
  void on_sensor_height_mm_edited();
  void on_focal_length_mm_edited();
  void on_focal_length_35mm_edited();
  void on_distortion_parameter_edited();
  void on_camera_mode_changed(int index);

private:
  void initialize_ui();
  void connect_signals();
  void update_ui_by_mode(database::ImageGroup::CameraMode mode);
  void update_group_level_ui();
  void update_image_level_ui();
  void update_rig_based_ui();

  // ─── UI 组件 ───

  // 基本信息组 (可选显示)
  QGroupBox* m_groupInfoBox;
  QLineEdit* m_groupNameEdit;

  // 相机参数组
  QGroupBox* m_cameraParamsBox;

  // 相机模式选择
  QComboBox* m_cameraModeCombo;
  QLabel* m_cameraModeLabel;

  // ─── GroupLevel 模式下的参数字段 ───
  // 焦距和主点
  QLabel* m_focalLengthPxLabel;
  QDoubleSpinBox* m_focalLengthPxSpinBox;

  QLabel* m_principalPointXLabel;
  QDoubleSpinBox* m_principalPointXSpinBox;

  QLabel* m_principalPointYLabel;
  QDoubleSpinBox* m_principalPointYSpinBox;

  // 图像尺寸
  QLabel* m_imageWidthLabel;
  QSpinBox* m_imageWidthSpinBox;

  QLabel* m_imageHeightLabel;
  QSpinBox* m_imageHeightSpinBox;

  // 传感器参数
  QLabel* m_sensorWidthMmLabel;
  QDoubleSpinBox* m_sensorWidthMmSpinBox;

  QLabel* m_sensorHeightMmLabel;
  QDoubleSpinBox* m_sensorHeightMmSpinBox;

  // 焦距（mm）
  QLabel* m_focalLengthMmLabel;
  QDoubleSpinBox* m_focalLengthMmSpinBox;

  // 35mm等效焦距
  QLabel* m_focalLength35mmLabel;
  QDoubleSpinBox* m_focalLength35mmSpinBox;

  // ─── ImageLevel 模式占位符 ───
  QLabel* m_imageLevelPlaceholder; // 灰显时显示的提示

  // ─── RigBased 模式占位符 ───
  QLabel* m_rigBasedPlaceholder;

  // ─── 畸变参数组 ───
  QGroupBox* m_distortionBox;

  QLabel* m_k1Label;
  QDoubleSpinBox* m_k1SpinBox;

  QLabel* m_k2Label;
  QDoubleSpinBox* m_k2SpinBox;

  QLabel* m_k3Label;
  QDoubleSpinBox* m_k3SpinBox;

  QLabel* m_p1Label;
  QDoubleSpinBox* m_p1SpinBox; ///< p1 = Bentley P₁

  QLabel* m_p2Label;
  QDoubleSpinBox* m_p2SpinBox; ///< p2 = Bentley P₂

  // ─── 按钮组 (暂灰显) ───
  QPushButton* m_autoEstimateButton;
  QPushButton* m_selectPresetButton;

  // ─── 提示文本 ───
  QLabel* m_coordinateSystemHintLabel;

  // ─── 内部状态 ───
  database::ImageGroup::CameraMode m_currentMode;
  bool m_isEditable;
};

}  // namespace insight::ui::widgets
