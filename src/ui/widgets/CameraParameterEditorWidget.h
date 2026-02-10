#pragma once

#include <QWidget>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QLineEdit>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QRadioButton>

#include "database/database_types.h"

namespace insight::ui::widgets {

/**
 * @brief 相机参数编辑器 Widget - 可复用于多个界面（ImageGroup、AT Task等）
 * 
 * 支持 Brown-Conrady 畸变模型的完整参数编辑：
 * - 9 个内参：焦距(px)、主点(px)、图像宽高(px)、传感器宽高(mm)、焦距(mm)、35mm等效焦距
 * - 7 个畸变参数：k1, k2, k3, p1, p2, b1, b2（默认0）
 * - 3 种相机模式：GroupLevel / ImageLevel / RigBased
 * 
 * 失焦时自动保存数据（通过 fieldModified() 信号）
 */
class CameraParameterEditorWidget : public QWidget {
    Q_OBJECT

public:
    explicit CameraParameterEditorWidget(QWidget* parent = nullptr);
    ~CameraParameterEditorWidget();

    // ─── 数据操作 ───
    /**
     * @brief 加载相机参数到 UI 中
     */
    void LoadCamera(const database::CameraModel& camera);

    /**
     * @brief 获取当前编辑的相机参数
     */
    database::CameraModel GetCamera() const;

    /**
     * @brief 设置分组名称（仅当 ShowGroupNameField(true) 时显示）
     */
    void SetGroupName(const std::string& name);

    /**
     * @brief 获取分组名称
     */
    std::string GetGroupName() const;

    // ─── UI 控制 ───
    /**
     * @brief 控制分组名称字段的可见性
     */
    void ShowGroupNameField(bool show);

    /**
     * @brief 设置所有字段的编辑状态
     */
    void SetEditable(bool editable);

    /**
     * @brief 设置相机模式（动态显示/隐藏相关字段）
     */
    void SetMode(database::ImageGroup::CameraMode mode);

    /**
     * @brief 获取当前相机模式
     */
    database::ImageGroup::CameraMode GetMode() const;

signals:
    /**
     * @brief 任何字段失焦时发出此信号
     */
    void fieldModified();

    /**
     * @brief 相机模式改变时发出此信号
     */
    void modeChanged(database::ImageGroup::CameraMode mode);

    /**
     * @brief 请求自动估计参数时发出此信号
     */
    void autoEstimateRequested();

private slots:
    // ─── 内参字段信号槽 ───
    void onFocalLengthPxEdited();
    void onPrincipalPointXEdited();
    void onPrincipalPointYEdited();
    void onImageWidthEdited();
    void onImageHeightEdited();
    void onSensorWidthMmEdited();
    void onSensorHeightMmEdited();
    void onFocalLengthMmEdited();
    void onFocalLength35mmEdited();

    // ─── 畸变参数信号槽 ───
    void onDistortionParameterEdited();

    // ─── 相机模式信号槽 ───
    void onCameraModeChanged(int index);

private:
    // ─── 初始化 ───
    void InitializeUI();
    void ConnectSignals();

    // ─── UI 更新 ───
    void UpdateUIByMode(database::ImageGroup::CameraMode mode);
    void UpdateGroupLevelUI();
    void UpdateImageLevelUI();
    void UpdateRigBasedUI();

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
    QLabel* m_imageLevelPlaceholder;  // 灰显时显示的提示

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
    QDoubleSpinBox* m_p1SpinBox;

    QLabel* m_p2Label;
    QDoubleSpinBox* m_p2SpinBox;

    QLabel* m_b1Label;
    QDoubleSpinBox* m_b1SpinBox;

    QLabel* m_b2Label;
    QDoubleSpinBox* m_b2SpinBox;

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
