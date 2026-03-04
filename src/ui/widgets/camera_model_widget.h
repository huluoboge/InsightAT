/**
 * @file CameraModelWidget.h
 * @brief 相机模型编辑小部件
 *
 * 功能：
 * 1. 编辑相机的各项参数（焦距、主点、畸变等）
 * 2. 支持多种相机类型（Pinhole、Fisheye等）
 * 3. 参数验证和约束检查
 * 4. 实时显示参数有效性
 */

#ifndef UI_CAMERAMODELWIDGET_H
#define UI_CAMERAMODELWIDGET_H

#include <QWidget>
#include <memory>

#include "database/database_types.h"

class QLineEdit;
class QLabel;
class QCheckBox;
class QComboBox;
class QGroupBox;
class QDoubleSpinBox;

namespace insight {
namespace ui {

/**
 * @class CameraModelWidget
 * @brief 相机模型编辑小部件
 */
class CameraModelWidget : public QWidget {
  Q_OBJECT

public:
  explicit CameraModelWidget(QWidget* parent = nullptr);
  ~CameraModelWidget();

  /**
   * 获取编辑的相机模型
   *
   * @return CameraModel 对象
   */
  insight::database::CameraModel get_camera_model() const;

  void set_camera_model(const insight::database::CameraModel& camera);

  bool validate_camera() const;

  void clear_all();

signals:
  void camera_model_changed(const insight::database::CameraModel& camera);

private slots:
  void on_parameter_changed();

private:
  void initialize_ui();
  void update_validation_status();
  void update_camera_model();

  // 基本参数
  QComboBox* m_cameraTypeCombo; ///< 相机类型

  // 分辨率
  QDoubleSpinBox* m_widthSpinBox;  ///< 图像宽度 (pixel)
  QDoubleSpinBox* m_heightSpinBox; ///< 图像高度 (pixel)

  // 传感器
  QDoubleSpinBox* m_sensorWidthSpinBox;     ///< 传感器宽 (mm)
  QDoubleSpinBox* m_sensorHeightSpinBox;    ///< 传感器高 (mm)
  QDoubleSpinBox* m_pixelSizeSpinBox;       ///< 像素大小 (μm)
  QDoubleSpinBox* m_focalLength35mmSpinBox; ///< 35mm等效焦距 (mm)

  // 焦距
  QDoubleSpinBox* m_focalLengthSpinBox; ///< 焦距 (pixel)

  // 主点（光学中心）
  QDoubleSpinBox* m_cxSpinBox; ///< 主点 X (pixel)
  QDoubleSpinBox* m_cySpinBox; ///< 主点 Y (pixel)

  // 径向畸变（Bentley K₁,K₂,K₃）
  QDoubleSpinBox* m_k1SpinBox; ///< k1 (1st radial distortion, Bentley K₁)
  QDoubleSpinBox* m_k2SpinBox; ///< k2 (2nd radial distortion, Bentley K₂)
  QDoubleSpinBox* m_k3SpinBox; ///< k3 (3rd radial distortion, Bentley K₃)

  // 切向畸变（Bentley P₁,P₂）
  QDoubleSpinBox* m_p1SpinBox; ///< p1 = Bentley P₁
  QDoubleSpinBox* m_p2SpinBox; ///< p2 = Bentley P₂

  // 验证状态显示
  QLabel* m_validationStatusLabel;  ///< 验证状态标签
  QLabel* m_distortionWarningLabel; ///< 畸变警告标签

  // 内部状态
  insight::database::CameraModel m_camera; ///< 当前编辑的相机模型
  bool m_blockSignals = false;             ///< 阻止信号递归
};

}  // namespace ui
}  // namespace insight

#endif  // UI_CAMERAMODELWIDGET_H
