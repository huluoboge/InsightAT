#include "CameraParameterEditorWidget.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QScrollArea>
#include <glog/logging.h>

namespace insight::ui::widgets {

CameraParameterEditorWidget::CameraParameterEditorWidget(QWidget* parent)
    : QWidget(parent),
      m_currentMode(database::ImageGroup::CameraMode::kGroupLevel),
      m_isEditable(true) {
    InitializeUI();
    ConnectSignals();
}

CameraParameterEditorWidget::~CameraParameterEditorWidget() = default;

void CameraParameterEditorWidget::InitializeUI() {
    auto mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(8, 8, 8, 8);
    mainLayout->setSpacing(8);

    // ═══════════════════════════════════════════════════════════════
    // GroupBox 1: 分组基本信息 (可选显示)
    // ═══════════════════════════════════════════════════════════════
    m_groupInfoBox = new QGroupBox(tr("Group Info"), this);
    m_groupInfoBox->setStyleSheet("QGroupBox { font-weight: bold; }");
    {
        auto layout = new QHBoxLayout(m_groupInfoBox);

        layout->addWidget(new QLabel(tr("Name:"), this));

        m_groupNameEdit = new QLineEdit(this);
        m_groupNameEdit->setPlaceholderText(tr("Group name"));
        layout->addWidget(m_groupNameEdit);
    }
    mainLayout->addWidget(m_groupInfoBox);

    // ═══════════════════════════════════════════════════════════════
    // GroupBox 2: 相机参数
    // ═══════════════════════════════════════════════════════════════
    m_cameraParamsBox = new QGroupBox(tr("Camera Parameters"), this);
    m_cameraParamsBox->setStyleSheet("QGroupBox { font-weight: bold; }");
    {
        auto layout = new QVBoxLayout(m_cameraParamsBox);

        // 相机模式选择
        {
            auto modeLayout = new QHBoxLayout();
            m_cameraModeLabel = new QLabel(tr("Mode:"), this);
            m_cameraModeCombo = new QComboBox(this);
            m_cameraModeCombo->addItem("GroupLevel", static_cast<int>(database::ImageGroup::CameraMode::kGroupLevel));
            m_cameraModeCombo->addItem("ImageLevel", static_cast<int>(database::ImageGroup::CameraMode::kImageLevel));
            m_cameraModeCombo->addItem("RigBased", static_cast<int>(database::ImageGroup::CameraMode::kRigBased));
            modeLayout->addWidget(m_cameraModeLabel);
            modeLayout->addWidget(m_cameraModeCombo);
            modeLayout->addStretch();
            layout->addLayout(modeLayout);
        }

        // 内参字段 (Grid 布局)
        auto paramsGrid = new QGridLayout();
        paramsGrid->setSpacing(6);
        int row = 0;

        // 焦距 (px)
        m_focalLengthPxLabel = new QLabel(tr("Focal Length (px):"), this);
        m_focalLengthPxSpinBox = new QDoubleSpinBox(this);
        m_focalLengthPxSpinBox->setRange(-10000.0, 10000.0);
        m_focalLengthPxSpinBox->setValue(0.0);
        m_focalLengthPxSpinBox->setDecimals(6);
        paramsGrid->addWidget(m_focalLengthPxLabel, row, 0);
        paramsGrid->addWidget(m_focalLengthPxSpinBox, row, 1);
        row++;

        // 主点 X (px)
        m_principalPointXLabel = new QLabel(tr("Principal Point X (px):"), this);
        m_principalPointXSpinBox = new QDoubleSpinBox(this);
        m_principalPointXSpinBox->setRange(-10000.0, 10000.0);
        m_principalPointXSpinBox->setValue(0.0);
        m_principalPointXSpinBox->setDecimals(6);
        paramsGrid->addWidget(m_principalPointXLabel, row, 0);
        paramsGrid->addWidget(m_principalPointXSpinBox, row, 1);
        row++;

        // 主点 Y (px)
        m_principalPointYLabel = new QLabel(tr("Principal Point Y (px):"), this);
        m_principalPointYSpinBox = new QDoubleSpinBox(this);
        m_principalPointYSpinBox->setRange(-10000.0, 10000.0);
        m_principalPointYSpinBox->setValue(0.0);
        m_principalPointYSpinBox->setDecimals(6);
        paramsGrid->addWidget(m_principalPointYLabel, row, 0);
        paramsGrid->addWidget(m_principalPointYSpinBox, row, 1);
        row++;

        // 图像宽 (px)
        m_imageWidthLabel = new QLabel(tr("Image Width (px):"), this);
        m_imageWidthSpinBox = new QSpinBox(this);
        m_imageWidthSpinBox->setRange(0, 100000);
        m_imageWidthSpinBox->setValue(0);
        paramsGrid->addWidget(m_imageWidthLabel, row, 0);
        paramsGrid->addWidget(m_imageWidthSpinBox, row, 1);
        row++;

        // 图像高 (px)
        m_imageHeightLabel = new QLabel(tr("Image Height (px):"), this);
        m_imageHeightSpinBox = new QSpinBox(this);
        m_imageHeightSpinBox->setRange(0, 100000);
        m_imageHeightSpinBox->setValue(0);
        paramsGrid->addWidget(m_imageHeightLabel, row, 0);
        paramsGrid->addWidget(m_imageHeightSpinBox, row, 1);
        row++;

        // Sensor 宽 (mm)
        m_sensorWidthMmLabel = new QLabel(tr("Sensor Width (mm):"), this);
        m_sensorWidthMmSpinBox = new QDoubleSpinBox(this);
        m_sensorWidthMmSpinBox->setRange(0.0, 1000.0);
        m_sensorWidthMmSpinBox->setValue(0.0);
        m_sensorWidthMmSpinBox->setDecimals(6);
        paramsGrid->addWidget(m_sensorWidthMmLabel, row, 0);
        paramsGrid->addWidget(m_sensorWidthMmSpinBox, row, 1);
        row++;

        // Sensor 高 (mm)
        m_sensorHeightMmLabel = new QLabel(tr("Sensor Height (mm):"), this);
        m_sensorHeightMmSpinBox = new QDoubleSpinBox(this);
        m_sensorHeightMmSpinBox->setRange(0.0, 1000.0);
        m_sensorHeightMmSpinBox->setValue(0.0);
        m_sensorHeightMmSpinBox->setDecimals(6);
        paramsGrid->addWidget(m_sensorHeightMmLabel, row, 0);
        paramsGrid->addWidget(m_sensorHeightMmSpinBox, row, 1);
        row++;

        // 焦距 (mm)
        m_focalLengthMmLabel = new QLabel(tr("Focal Length (mm):"), this);
        m_focalLengthMmSpinBox = new QDoubleSpinBox(this);
        m_focalLengthMmSpinBox->setRange(-1000.0, 1000.0);
        m_focalLengthMmSpinBox->setValue(0.0);
        m_focalLengthMmSpinBox->setDecimals(6);
        paramsGrid->addWidget(m_focalLengthMmLabel, row, 0);
        paramsGrid->addWidget(m_focalLengthMmSpinBox, row, 1);
        row++;

        // 35mm等效焦距
        m_focalLength35mmLabel = new QLabel(tr("35mm Equivalent Focal Length:"), this);
        m_focalLength35mmSpinBox = new QDoubleSpinBox(this);
        m_focalLength35mmSpinBox->setRange(-1000.0, 1000.0);
        m_focalLength35mmSpinBox->setValue(0.0);
        m_focalLength35mmSpinBox->setDecimals(6);
        paramsGrid->addWidget(m_focalLength35mmLabel, row, 0);
        paramsGrid->addWidget(m_focalLength35mmSpinBox, row, 1);

        layout->addLayout(paramsGrid);

        // ImageLevel 模式占位符
        m_imageLevelPlaceholder = new QLabel(
            tr("Note: ImageLevel mode parameters are grayed out here.\n"
               "Image-specific camera parameters are configured in the image list editor."),
            this);
        m_imageLevelPlaceholder->setStyleSheet("color: gray; font-style: italic;");
        m_imageLevelPlaceholder->setVisible(false);
        layout->addWidget(m_imageLevelPlaceholder);

        // RigBased 模式占位符
        m_rigBasedPlaceholder = new QLabel(
            tr("Note: RigBased mode configuration is pending Rig editor implementation.\n"
               "RigBased mode setup coming in a future release."),
            this);
        m_rigBasedPlaceholder->setStyleSheet("color: gray; font-style: italic;");
        m_rigBasedPlaceholder->setVisible(false);
        layout->addWidget(m_rigBasedPlaceholder);

        // 按钮行 (暂灰显)
        {
            auto btnLayout = new QHBoxLayout();
            m_autoEstimateButton = new QPushButton(tr("Auto Estimate"), this);
            m_autoEstimateButton->setEnabled(false);
            m_selectPresetButton = new QPushButton(tr("Select Preset Camera"), this);
            m_selectPresetButton->setEnabled(false);
            btnLayout->addWidget(m_autoEstimateButton);
            btnLayout->addWidget(m_selectPresetButton);
            btnLayout->addStretch();
            layout->addLayout(btnLayout);
        }
    }
    mainLayout->addWidget(m_cameraParamsBox);

    // ═══════════════════════════════════════════════════════════════
    // GroupBox 3: 畸变参数
    // ═══════════════════════════════════════════════════════════════
    m_distortionBox = new QGroupBox(tr("Distortion Parameters (Brown-Conrady)"), this);
    m_distortionBox->setStyleSheet("QGroupBox { font-weight: bold; }");
    {
        auto layout = new QGridLayout(m_distortionBox);
        layout->setSpacing(6);

        int row = 0;

        // k1, k2, k3 (径向畸变)
        m_k1Label = new QLabel(tr("k1:"), this);
        m_k1SpinBox = new QDoubleSpinBox(this);
        m_k1SpinBox->setRange(-1.0, 1.0);
        m_k1SpinBox->setValue(0.0);
        m_k1SpinBox->setSingleStep(0.001);
        m_k1SpinBox->setDecimals(8);
        layout->addWidget(m_k1Label, row, 0);
        layout->addWidget(m_k1SpinBox, row, 1);

        m_k2Label = new QLabel(tr("k2:"), this);
        m_k2SpinBox = new QDoubleSpinBox(this);
        m_k2SpinBox->setRange(-1.0, 1.0);
        m_k2SpinBox->setValue(0.0);
        m_k2SpinBox->setSingleStep(0.001);
        m_k2SpinBox->setDecimals(8);
        layout->addWidget(m_k2Label, row, 2);
        layout->addWidget(m_k2SpinBox, row, 3);

        m_k3Label = new QLabel(tr("k3:"), this);
        m_k3SpinBox = new QDoubleSpinBox(this);
        m_k3SpinBox->setRange(-1.0, 1.0);
        m_k3SpinBox->setValue(0.0);
        m_k3SpinBox->setSingleStep(0.001);
        m_k3SpinBox->setDecimals(8);
        layout->addWidget(m_k3Label, row, 4);
        layout->addWidget(m_k3SpinBox, row, 5);
        row++;

        // p1, p2 (切向畸变)
        m_p1Label = new QLabel(tr("p1:"), this);
        m_p1SpinBox = new QDoubleSpinBox(this);
        m_p1SpinBox->setRange(-1.0, 1.0);
        m_p1SpinBox->setValue(0.0);
        m_p1SpinBox->setSingleStep(0.001);
        m_p1SpinBox->setDecimals(8);
        layout->addWidget(m_p1Label, row, 0);
        layout->addWidget(m_p1SpinBox, row, 1);

        m_p2Label = new QLabel(tr("p2:"), this);
        m_p2SpinBox = new QDoubleSpinBox(this);
        m_p2SpinBox->setRange(-1.0, 1.0);
        m_p2SpinBox->setValue(0.0);
        m_p2SpinBox->setSingleStep(0.001);
        m_p2SpinBox->setDecimals(8);
        layout->addWidget(m_p2Label, row, 2);
        layout->addWidget(m_p2SpinBox, row, 3);
        row++;

        // b1, b2 (棱镜畸变)
        m_b1Label = new QLabel(tr("b1:"), this);
        m_b1SpinBox = new QDoubleSpinBox(this);
        m_b1SpinBox->setRange(-1.0, 1.0);
        m_b1SpinBox->setValue(0.0);
        m_b1SpinBox->setSingleStep(0.001);
        m_b1SpinBox->setDecimals(8);
        layout->addWidget(m_b1Label, row, 0);
        layout->addWidget(m_b1SpinBox, row, 1);

        m_b2Label = new QLabel(tr("b2:"), this);
        m_b2SpinBox = new QDoubleSpinBox(this);
        m_b2SpinBox->setRange(-1.0, 1.0);
        m_b2SpinBox->setValue(0.0);
        m_b2SpinBox->setSingleStep(0.001);
        m_b2SpinBox->setDecimals(8);
        layout->addWidget(m_b2Label, row, 2);
        layout->addWidget(m_b2SpinBox, row, 3);
    }
    mainLayout->addWidget(m_distortionBox);

    // ═══════════════════════════════════════════════════════════════
    // 坐标系提示
    // ═══════════════════════════════════════════════════════════════
    m_coordinateSystemHintLabel = new QLabel(
        tr("Coordinate system starts from 0. "
           "The center of the first pixel is at (0.5, 0.5)."),
        this);
    m_coordinateSystemHintLabel->setStyleSheet(
        "color: #666666; font-size: 11px; font-style: italic;");
    m_coordinateSystemHintLabel->setWordWrap(true);
    mainLayout->addWidget(m_coordinateSystemHintLabel);

    mainLayout->addStretch();

    setLayout(mainLayout);
}

void CameraParameterEditorWidget::ConnectSignals() {
    // 内参字段
    connect(m_focalLengthPxSpinBox, QOverload<>::of(&QDoubleSpinBox::editingFinished),
            this, &CameraParameterEditorWidget::onFocalLengthPxEdited);
    connect(m_principalPointXSpinBox, QOverload<>::of(&QDoubleSpinBox::editingFinished),
            this, &CameraParameterEditorWidget::onPrincipalPointXEdited);
    connect(m_principalPointYSpinBox, QOverload<>::of(&QDoubleSpinBox::editingFinished),
            this, &CameraParameterEditorWidget::onPrincipalPointYEdited);
    connect(m_imageWidthSpinBox, QOverload<>::of(&QSpinBox::editingFinished),
            this, &CameraParameterEditorWidget::onImageWidthEdited);
    connect(m_imageHeightSpinBox, QOverload<>::of(&QSpinBox::editingFinished),
            this, &CameraParameterEditorWidget::onImageHeightEdited);
    connect(m_sensorWidthMmSpinBox, QOverload<>::of(&QDoubleSpinBox::editingFinished),
            this, &CameraParameterEditorWidget::onSensorWidthMmEdited);
    connect(m_sensorHeightMmSpinBox, QOverload<>::of(&QDoubleSpinBox::editingFinished),
            this, &CameraParameterEditorWidget::onSensorHeightMmEdited);
    connect(m_focalLengthMmSpinBox, QOverload<>::of(&QDoubleSpinBox::editingFinished),
            this, &CameraParameterEditorWidget::onFocalLengthMmEdited);
    connect(m_focalLength35mmSpinBox, QOverload<>::of(&QDoubleSpinBox::editingFinished),
            this, &CameraParameterEditorWidget::onFocalLength35mmEdited);

    // 畸变参数
    connect(m_k1SpinBox, QOverload<>::of(&QDoubleSpinBox::editingFinished),
            this, &CameraParameterEditorWidget::onDistortionParameterEdited);
    connect(m_k2SpinBox, QOverload<>::of(&QDoubleSpinBox::editingFinished),
            this, &CameraParameterEditorWidget::onDistortionParameterEdited);
    connect(m_k3SpinBox, QOverload<>::of(&QDoubleSpinBox::editingFinished),
            this, &CameraParameterEditorWidget::onDistortionParameterEdited);
    connect(m_p1SpinBox, QOverload<>::of(&QDoubleSpinBox::editingFinished),
            this, &CameraParameterEditorWidget::onDistortionParameterEdited);
    connect(m_p2SpinBox, QOverload<>::of(&QDoubleSpinBox::editingFinished),
            this, &CameraParameterEditorWidget::onDistortionParameterEdited);
    connect(m_b1SpinBox, QOverload<>::of(&QDoubleSpinBox::editingFinished),
            this, &CameraParameterEditorWidget::onDistortionParameterEdited);
    connect(m_b2SpinBox, QOverload<>::of(&QDoubleSpinBox::editingFinished),
            this, &CameraParameterEditorWidget::onDistortionParameterEdited);

    // 相机模式切换
    connect(m_cameraModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &CameraParameterEditorWidget::onCameraModeChanged);

    // 分组名称
    connect(m_groupNameEdit, &QLineEdit::editingFinished,
            this, &CameraParameterEditorWidget::fieldModified);
}

// ═══════════════════════════════════════════════════════════════
// 公开方法
// ═══════════════════════════════════════════════════════════════

void CameraParameterEditorWidget::LoadCamera(const database::CameraModel& camera) {
    blockSignals(true);  // 防止信号在加载期间触发

    // 加载内参（camera_mode 由外部通过 SetMode() 设置）
    m_focalLengthPxSpinBox->setValue(camera.focal_length);
    m_principalPointXSpinBox->setValue(camera.principal_point_x);
    m_principalPointYSpinBox->setValue(camera.principal_point_y);
    m_imageWidthSpinBox->setValue(camera.width);
    m_imageHeightSpinBox->setValue(camera.height);
    m_sensorWidthMmSpinBox->setValue(camera.sensor_width_mm);
    m_sensorHeightMmSpinBox->setValue(camera.sensor_height_mm);
    m_focalLengthMmSpinBox->setValue(camera.focal_length_35mm);  // 没有单独的focal_length_mm，使用focal_length_35mm
    m_focalLength35mmSpinBox->setValue(camera.focal_length_35mm);

    // 加载畸变参数
    m_k1SpinBox->setValue(camera.k1);
    m_k2SpinBox->setValue(camera.k2);
    m_k3SpinBox->setValue(camera.k3);
    m_p1SpinBox->setValue(camera.p1);
    m_p2SpinBox->setValue(camera.p2);
    m_b1SpinBox->setValue(camera.b1);
    m_b2SpinBox->setValue(camera.b2);

    blockSignals(false);

    // 注意: camera_mode 不属于 CameraModel，应由外部代码通过 SetMode() 调用设置
}

database::CameraModel CameraParameterEditorWidget::GetCamera() const {
    database::CameraModel camera;

    // 注意: camera_mode 不属于 CameraModel，由 ImageGroup 管理

    // 内参
    camera.focal_length = m_focalLengthPxSpinBox->value();
    camera.principal_point_x = m_principalPointXSpinBox->value();
    camera.principal_point_y = m_principalPointYSpinBox->value();
    camera.width = m_imageWidthSpinBox->value();
    camera.height = m_imageHeightSpinBox->value();
    camera.sensor_width_mm = m_sensorWidthMmSpinBox->value();
    camera.sensor_height_mm = m_sensorHeightMmSpinBox->value();
    camera.focal_length_35mm = m_focalLength35mmSpinBox->value();

    // 畸变参数
    camera.k1 = m_k1SpinBox->value();
    camera.k2 = m_k2SpinBox->value();
    camera.k3 = m_k3SpinBox->value();
    camera.p1 = m_p1SpinBox->value();
    camera.p2 = m_p2SpinBox->value();
    camera.b1 = m_b1SpinBox->value();
    camera.b2 = m_b2SpinBox->value();

    return camera;
}

void CameraParameterEditorWidget::SetGroupName(const std::string& name) {
    m_groupNameEdit->setText(QString::fromStdString(name));
}

std::string CameraParameterEditorWidget::GetGroupName() const {
    return m_groupNameEdit->text().toStdString();
}

void CameraParameterEditorWidget::ShowGroupNameField(bool show) {
    m_groupInfoBox->setVisible(show);
}

void CameraParameterEditorWidget::SetEditable(bool editable) {
    m_isEditable = editable;
    m_groupNameEdit->setReadOnly(!editable);
    m_cameraModeCombo->setEnabled(editable);
    m_focalLengthPxSpinBox->setReadOnly(!editable);
    m_principalPointXSpinBox->setReadOnly(!editable);
    m_principalPointYSpinBox->setReadOnly(!editable);
    m_imageWidthSpinBox->setReadOnly(!editable);
    m_imageHeightSpinBox->setReadOnly(!editable);
    m_sensorWidthMmSpinBox->setReadOnly(!editable);
    m_sensorHeightMmSpinBox->setReadOnly(!editable);
    m_focalLengthMmSpinBox->setReadOnly(!editable);
    m_focalLength35mmSpinBox->setReadOnly(!editable);
    m_k1SpinBox->setReadOnly(!editable);
    m_k2SpinBox->setReadOnly(!editable);
    m_k3SpinBox->setReadOnly(!editable);
    m_p1SpinBox->setReadOnly(!editable);
    m_p2SpinBox->setReadOnly(!editable);
    m_b1SpinBox->setReadOnly(!editable);
    m_b2SpinBox->setReadOnly(!editable);
}

void CameraParameterEditorWidget::SetMode(database::ImageGroup::CameraMode mode) {
    m_currentMode = mode;
    int modeIndex = m_cameraModeCombo->findData(static_cast<int>(mode));
    m_cameraModeCombo->setCurrentIndex(modeIndex >= 0 ? modeIndex : 0);
    UpdateUIByMode(mode);
}

database::ImageGroup::CameraMode CameraParameterEditorWidget::GetMode() const {
    return m_currentMode;
}

// ═══════════════════════════════════════════════════════════════
// 槽函数
// ═══════════════════════════════════════════════════════════════

void CameraParameterEditorWidget::onFocalLengthPxEdited() {
    if (m_isEditable) {
        emit fieldModified();
    }
}

void CameraParameterEditorWidget::onPrincipalPointXEdited() {
    if (m_isEditable) {
        emit fieldModified();
    }
}

void CameraParameterEditorWidget::onPrincipalPointYEdited() {
    if (m_isEditable) {
        emit fieldModified();
    }
}

void CameraParameterEditorWidget::onImageWidthEdited() {
    if (m_isEditable) {
        emit fieldModified();
    }
}

void CameraParameterEditorWidget::onImageHeightEdited() {
    if (m_isEditable) {
        emit fieldModified();
    }
}

void CameraParameterEditorWidget::onSensorWidthMmEdited() {
    if (m_isEditable) {
        emit fieldModified();
    }
}

void CameraParameterEditorWidget::onSensorHeightMmEdited() {
    if (m_isEditable) {
        emit fieldModified();
    }
}

void CameraParameterEditorWidget::onFocalLengthMmEdited() {
    if (m_isEditable) {
        emit fieldModified();
    }
}

void CameraParameterEditorWidget::onFocalLength35mmEdited() {
    if (m_isEditable) {
        emit fieldModified();
    }
}

void CameraParameterEditorWidget::onDistortionParameterEdited() {
    if (m_isEditable) {
        emit fieldModified();
    }
}

void CameraParameterEditorWidget::onCameraModeChanged(int index) {
    auto mode = static_cast<database::ImageGroup::CameraMode>(
        m_cameraModeCombo->itemData(index).toInt());
    m_currentMode = mode;
    UpdateUIByMode(mode);
    if (m_isEditable) {
        emit modeChanged(mode);
        emit fieldModified();
    }
}

// ═══════════════════════════════════════════════════════════════
// 私有方法
// ═══════════════════════════════════════════════════════════════

void CameraParameterEditorWidget::UpdateUIByMode(database::ImageGroup::CameraMode mode) {
    switch (mode) {
        case database::ImageGroup::CameraMode::kGroupLevel:
            UpdateGroupLevelUI();
            break;
        case database::ImageGroup::CameraMode::kImageLevel:
            UpdateImageLevelUI();
            break;
        case database::ImageGroup::CameraMode::kRigBased:
            UpdateRigBasedUI();
            break;
    }
}

void CameraParameterEditorWidget::UpdateGroupLevelUI() {
    // GroupLevel: 所有参数可见且可编辑
    m_focalLengthPxLabel->setVisible(true);
    m_focalLengthPxSpinBox->setVisible(true);
    m_focalLengthPxSpinBox->setEnabled(true);
    m_principalPointXLabel->setVisible(true);
    m_principalPointXSpinBox->setVisible(true);
    m_principalPointXSpinBox->setEnabled(true);
    m_principalPointYLabel->setVisible(true);
    m_principalPointYSpinBox->setVisible(true);
    m_principalPointYSpinBox->setEnabled(true);
    m_imageWidthLabel->setVisible(true);
    m_imageWidthSpinBox->setVisible(true);
    m_imageWidthSpinBox->setEnabled(true);
    m_imageHeightLabel->setVisible(true);
    m_imageHeightSpinBox->setVisible(true);
    m_imageHeightSpinBox->setEnabled(true);
    m_sensorWidthMmLabel->setVisible(true);
    m_sensorWidthMmSpinBox->setVisible(true);
    m_sensorWidthMmSpinBox->setEnabled(true);
    m_sensorHeightMmLabel->setVisible(true);
    m_sensorHeightMmSpinBox->setVisible(true);
    m_sensorHeightMmSpinBox->setEnabled(true);
    m_focalLengthMmLabel->setVisible(true);
    m_focalLengthMmSpinBox->setVisible(true);
    m_focalLengthMmSpinBox->setEnabled(true);
    m_focalLength35mmLabel->setVisible(true);
    m_focalLength35mmSpinBox->setVisible(true);
    m_focalLength35mmSpinBox->setEnabled(true);

    // 启用畸变参数
    m_k1SpinBox->setEnabled(true);
    m_k2SpinBox->setEnabled(true);
    m_k3SpinBox->setEnabled(true);
    m_p1SpinBox->setEnabled(true);
    m_p2SpinBox->setEnabled(true);
    m_b1SpinBox->setEnabled(true);
    m_b2SpinBox->setEnabled(true);

    m_distortionBox->setVisible(true);
    m_distortionBox->setEnabled(true);

    m_imageLevelPlaceholder->setVisible(false);
    m_rigBasedPlaceholder->setVisible(false);
}

void CameraParameterEditorWidget::UpdateImageLevelUI() {
    // ImageLevel: 参数显示但灰显（仅用于显示，实际在图像列表编辑）
    m_focalLengthPxLabel->setVisible(true);
    m_focalLengthPxSpinBox->setVisible(true);
    m_focalLengthPxSpinBox->setEnabled(false);
    m_principalPointXLabel->setVisible(true);
    m_principalPointXSpinBox->setVisible(true);
    m_principalPointXSpinBox->setEnabled(false);
    m_principalPointYLabel->setVisible(true);
    m_principalPointYSpinBox->setVisible(true);
    m_principalPointYSpinBox->setEnabled(false);
    m_imageWidthLabel->setVisible(true);
    m_imageWidthSpinBox->setVisible(true);
    m_imageWidthSpinBox->setEnabled(false);
    m_imageHeightLabel->setVisible(true);
    m_imageHeightSpinBox->setVisible(true);
    m_imageHeightSpinBox->setEnabled(false);
    m_sensorWidthMmLabel->setVisible(true);
    m_sensorWidthMmSpinBox->setVisible(true);
    m_sensorWidthMmSpinBox->setEnabled(false);
    m_sensorHeightMmLabel->setVisible(true);
    m_sensorHeightMmSpinBox->setVisible(true);
    m_sensorHeightMmSpinBox->setEnabled(false);
    m_focalLengthMmLabel->setVisible(true);
    m_focalLengthMmSpinBox->setVisible(true);
    m_focalLengthMmSpinBox->setEnabled(false);
    m_focalLength35mmLabel->setVisible(true);
    m_focalLength35mmSpinBox->setVisible(true);
    m_focalLength35mmSpinBox->setEnabled(false);

    m_distortionBox->setVisible(true);
    m_distortionBox->setEnabled(false);

    m_imageLevelPlaceholder->setVisible(true);
    m_rigBasedPlaceholder->setVisible(false);
}

void CameraParameterEditorWidget::UpdateRigBasedUI() {
    // RigBased: 隐藏所有参数，显示占位符
    m_focalLengthPxLabel->setVisible(false);
    m_focalLengthPxSpinBox->setVisible(false);
    m_principalPointXLabel->setVisible(false);
    m_principalPointXSpinBox->setVisible(false);
    m_principalPointYLabel->setVisible(false);
    m_principalPointYSpinBox->setVisible(false);
    m_imageWidthLabel->setVisible(false);
    m_imageWidthSpinBox->setVisible(false);
    m_imageHeightLabel->setVisible(false);
    m_imageHeightSpinBox->setVisible(false);
    m_sensorWidthMmLabel->setVisible(false);
    m_sensorWidthMmSpinBox->setVisible(false);
    m_sensorHeightMmLabel->setVisible(false);
    m_sensorHeightMmSpinBox->setVisible(false);
    m_focalLengthMmLabel->setVisible(false);
    m_focalLengthMmSpinBox->setVisible(false);
    m_focalLength35mmLabel->setVisible(false);
    m_focalLength35mmSpinBox->setVisible(false);

    m_distortionBox->setVisible(false);

    m_imageLevelPlaceholder->setVisible(false);
    m_rigBasedPlaceholder->setVisible(true);
}

}  // namespace insight::ui::widgets
