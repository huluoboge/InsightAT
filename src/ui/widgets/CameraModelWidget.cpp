/**
 * @file CameraModelWidget.cpp
 * @brief 相机模型编辑小部件 - 实现
 */

#include "CameraModelWidget.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QCheckBox>
#include <QComboBox>
#include <QGroupBox>
#include <QDoubleSpinBox>
#include <QMessageBox>
#include <glog/logging.h>

namespace insight {
namespace ui {

CameraModelWidget::CameraModelWidget(QWidget* parent)
    : QWidget(parent)
{
    // 初始化相机模型为默认值
    m_camera = insight::database::CameraModel();
    initializeUI();
}

CameraModelWidget::~CameraModelWidget() = default;

insight::database::CameraModel CameraModelWidget::getCameraModel() const
{
    // 读取 UI 值并返回相机模型
    const_cast<CameraModelWidget*>(this)->updateCameraModel();
    return m_camera;
}

void CameraModelWidget::setCameraModel(const insight::database::CameraModel& camera)
{
    m_blockSignals = true;
    m_camera = camera;

    // 更新 UI 显示
    m_widthSpinBox->setValue(camera.width);
    m_heightSpinBox->setValue(camera.height);
    m_focalLengthSpinBox->setValue(camera.focal_length);
    m_sensorWidthSpinBox->setValue(camera.sensor_width_mm);
    m_sensorHeightSpinBox->setValue(camera.sensor_height_mm);
    m_pixelSizeSpinBox->setValue(camera.pixel_size_um);
    m_focalLength35mmSpinBox->setValue(camera.focal_length_35mm);
    m_cxSpinBox->setValue(camera.principal_point_x);
    m_cySpinBox->setValue(camera.principal_point_y);

    m_k1SpinBox->setValue(camera.k1);
    m_k2SpinBox->setValue(camera.k2);
    m_k3SpinBox->setValue(camera.k3);
    m_k4SpinBox->setValue(camera.k4);
    m_p1SpinBox->setValue(camera.p1);
    m_p2SpinBox->setValue(camera.p2);
    m_b1SpinBox->setValue(camera.b1);
    m_b2SpinBox->setValue(camera.b2);

    m_blockSignals = false;
    updateValidationStatus();
}

bool CameraModelWidget::validateCamera() const
{
    const auto& camera = m_camera;

    // 检查分辨率
    if (camera.width <= 0 || camera.height <= 0) {
        LOG(ERROR) << "Invalid image resolution: " << camera.width 
                   << " x " << camera.height;
        return false;
    }

    // 检查焦距
    if (camera.focal_length <= 0) {
        LOG(ERROR) << "Invalid focal length: " << camera.focal_length;
        return false;
    }

    // 检查传感器尺寸
    if (camera.sensor_width_mm <= 0 || camera.sensor_height_mm <= 0) {
        LOG(ERROR) << "Invalid sensor size: " << camera.sensor_width_mm 
                   << " x " << camera.sensor_height_mm;
        return false;
    }

    // 主点检查（可选，警告）
    if (camera.principal_point_x < 0 || camera.principal_point_y < 0) {
        LOG(WARNING) << "Principal point may be invalid";
    }

    return true;
}

void CameraModelWidget::clearAll()
{
    m_blockSignals = true;
    
    m_widthSpinBox->setValue(0);
    m_heightSpinBox->setValue(0);
    m_focalLengthSpinBox->setValue(0);
    m_sensorWidthSpinBox->setValue(0);
    m_sensorHeightSpinBox->setValue(0);
    m_pixelSizeSpinBox->setValue(0);
    m_focalLength35mmSpinBox->setValue(0);
    m_cxSpinBox->setValue(0);
    m_cySpinBox->setValue(0);

    m_k1SpinBox->setValue(0);
    m_k2SpinBox->setValue(0);
    m_k3SpinBox->setValue(0);
    m_k4SpinBox->setValue(0);
    m_p1SpinBox->setValue(0);
    m_p2SpinBox->setValue(0);
    m_b1SpinBox->setValue(0);
    m_b2SpinBox->setValue(0);

    m_blockSignals = false;
    updateValidationStatus();
}

void CameraModelWidget::initializeUI()
{
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setSpacing(10);
    mainLayout->setContentsMargins(10, 10, 10, 10);

    // ─────────────────────────────────────────────────────────
    // 相机类型选择
    // ─────────────────────────────────────────────────────────
    
    QHBoxLayout* typeLayout = new QHBoxLayout();
    typeLayout->addWidget(new QLabel("相机类型：", this));
    m_cameraTypeCombo = new QComboBox(this);
    m_cameraTypeCombo->addItem("Pinhole");
    m_cameraTypeCombo->addItem("Brown-Conrady");
    m_cameraTypeCombo->addItem("Simple");
    m_cameraTypeCombo->addItem("Fisheye");
    typeLayout->addWidget(m_cameraTypeCombo);
    typeLayout->addStretch();
    mainLayout->addLayout(typeLayout);

    // ─────────────────────────────────────────────────────────
    // 分辨率和传感器
    // ─────────────────────────────────────────────────────────
    
    QGroupBox* sensorGroup = new QGroupBox("分辨率和传感器", this);
    QVBoxLayout* sensorLayout = new QVBoxLayout(sensorGroup);

    // 分辨率
    QHBoxLayout* resLayout = new QHBoxLayout();
    resLayout->addWidget(new QLabel("分辨率 (pixel)：", this));
    resLayout->addWidget(new QLabel("宽：", this));
    m_widthSpinBox = new QDoubleSpinBox(this);
    m_widthSpinBox->setRange(1, 100000);
    m_widthSpinBox->setValue(3648);
    m_widthSpinBox->setDecimals(0);
    resLayout->addWidget(m_widthSpinBox);
    resLayout->addWidget(new QLabel("高：", this));
    m_heightSpinBox = new QDoubleSpinBox(this);
    m_heightSpinBox->setRange(1, 100000);
    m_heightSpinBox->setValue(2736);
    m_heightSpinBox->setDecimals(0);
    resLayout->addWidget(m_heightSpinBox);
    resLayout->addStretch();
    sensorLayout->addLayout(resLayout);

    // 传感器尺寸
    QHBoxLayout* sensorSizeLayout = new QHBoxLayout();
    sensorSizeLayout->addWidget(new QLabel("传感器尺寸 (mm)：", this));
    sensorSizeLayout->addWidget(new QLabel("宽：", this));
    m_sensorWidthSpinBox = new QDoubleSpinBox(this);
    m_sensorWidthSpinBox->setRange(0.1, 100);
    m_sensorWidthSpinBox->setValue(13.2);
    m_sensorWidthSpinBox->setDecimals(2);
    sensorSizeLayout->addWidget(m_sensorWidthSpinBox);
    sensorSizeLayout->addWidget(new QLabel("高：", this));
    m_sensorHeightSpinBox = new QDoubleSpinBox(this);
    m_sensorHeightSpinBox->setRange(0.1, 100);
    m_sensorHeightSpinBox->setValue(9.9);
    m_sensorHeightSpinBox->setDecimals(2);
    sensorSizeLayout->addWidget(m_sensorHeightSpinBox);
    sensorSizeLayout->addStretch();
    sensorLayout->addLayout(sensorSizeLayout);

    // 像素大小和 35mm 等效焦距
    QHBoxLayout* pixelLayout = new QHBoxLayout();
    pixelLayout->addWidget(new QLabel("像素大小 (μm)：", this));
    m_pixelSizeSpinBox = new QDoubleSpinBox(this);
    m_pixelSizeSpinBox->setRange(0.1, 100);
    m_pixelSizeSpinBox->setValue(3.6);
    m_pixelSizeSpinBox->setDecimals(2);
    pixelLayout->addWidget(m_pixelSizeSpinBox);
    pixelLayout->addWidget(new QLabel("35mm等效焦距 (mm)：", this));
    m_focalLength35mmSpinBox = new QDoubleSpinBox(this);
    m_focalLength35mmSpinBox->setRange(0, 1000);
    m_focalLength35mmSpinBox->setValue(35);
    m_focalLength35mmSpinBox->setDecimals(1);
    pixelLayout->addWidget(m_focalLength35mmSpinBox);
    pixelLayout->addStretch();
    sensorLayout->addLayout(pixelLayout);

    mainLayout->addWidget(sensorGroup);

    // ─────────────────────────────────────────────────────────
    // 焦距和主点
    // ─────────────────────────────────────────────────────────
    
    QGroupBox* intrinsicGroup = new QGroupBox("内参数 (Intrinsics)", this);
    QVBoxLayout* intrinsicLayout = new QVBoxLayout(intrinsicGroup);

    // 焦距
    QHBoxLayout* focalLayout = new QHBoxLayout();
    focalLayout->addWidget(new QLabel("焦距 (pixel)：", this));
    m_focalLengthSpinBox = new QDoubleSpinBox(this);
    m_focalLengthSpinBox->setRange(0.1, 100000);
    m_focalLengthSpinBox->setValue(1000);
    m_focalLengthSpinBox->setDecimals(2);
    focalLayout->addWidget(m_focalLengthSpinBox);
    focalLayout->addStretch();
    intrinsicLayout->addLayout(focalLayout);

    // 主点（光学中心）
    QHBoxLayout* ppLayout = new QHBoxLayout();
    ppLayout->addWidget(new QLabel("主点 (pixel)：", this));
    ppLayout->addWidget(new QLabel("cx：", this));
    m_cxSpinBox = new QDoubleSpinBox(this);
    m_cxSpinBox->setRange(-10000, 10000);
    m_cxSpinBox->setValue(1824);
    m_cxSpinBox->setDecimals(1);
    ppLayout->addWidget(m_cxSpinBox);
    ppLayout->addWidget(new QLabel("cy：", this));
    m_cySpinBox = new QDoubleSpinBox(this);
    m_cySpinBox->setRange(-10000, 10000);
    m_cySpinBox->setValue(1368);
    m_cySpinBox->setDecimals(1);
    ppLayout->addWidget(m_cySpinBox);
    ppLayout->addStretch();
    intrinsicLayout->addLayout(ppLayout);

    mainLayout->addWidget(intrinsicGroup);

    // ─────────────────────────────────────────────────────────
    // 畸变参数 (Distortion)
    // ─────────────────────────────────────────────────────────
    
    QGroupBox* distortionGroup = new QGroupBox("畸变参数 (Distortion)", this);
    QVBoxLayout* distortionLayout = new QVBoxLayout(distortionGroup);

    // 径向畸变
    QHBoxLayout* radialLayout = new QHBoxLayout();
    radialLayout->addWidget(new QLabel("径向：", this));
    radialLayout->addWidget(new QLabel("k1", this));
    m_k1SpinBox = new QDoubleSpinBox(this);
    m_k1SpinBox->setRange(-1, 1);
    m_k1SpinBox->setValue(0);
    m_k1SpinBox->setDecimals(6);
    m_k1SpinBox->setSingleStep(0.001);
    radialLayout->addWidget(m_k1SpinBox);
    radialLayout->addWidget(new QLabel("k2", this));
    m_k2SpinBox = new QDoubleSpinBox(this);
    m_k2SpinBox->setRange(-1, 1);
    m_k2SpinBox->setValue(0);
    m_k2SpinBox->setDecimals(6);
    m_k2SpinBox->setSingleStep(0.001);
    radialLayout->addWidget(m_k2SpinBox);
    radialLayout->addWidget(new QLabel("k3", this));
    m_k3SpinBox = new QDoubleSpinBox(this);
    m_k3SpinBox->setRange(-1, 1);
    m_k3SpinBox->setValue(0);
    m_k3SpinBox->setDecimals(6);
    m_k3SpinBox->setSingleStep(0.001);
    radialLayout->addWidget(m_k3SpinBox);
    radialLayout->addWidget(new QLabel("k4", this));
    m_k4SpinBox = new QDoubleSpinBox(this);
    m_k4SpinBox->setRange(-1, 1);
    m_k4SpinBox->setValue(0);
    m_k4SpinBox->setDecimals(6);
    m_k4SpinBox->setSingleStep(0.001);
    radialLayout->addWidget(m_k4SpinBox);
    radialLayout->addStretch();
    distortionLayout->addLayout(radialLayout);

    // 切向畸变
    QHBoxLayout* tangentialLayout = new QHBoxLayout();
    tangentialLayout->addWidget(new QLabel("切向：", this));
    tangentialLayout->addWidget(new QLabel("p1", this));
    m_p1SpinBox = new QDoubleSpinBox(this);
    m_p1SpinBox->setRange(-1, 1);
    m_p1SpinBox->setValue(0);
    m_p1SpinBox->setDecimals(6);
    m_p1SpinBox->setSingleStep(0.001);
    tangentialLayout->addWidget(m_p1SpinBox);
    tangentialLayout->addWidget(new QLabel("p2", this));
    m_p2SpinBox = new QDoubleSpinBox(this);
    m_p2SpinBox->setRange(-1, 1);
    m_p2SpinBox->setValue(0);
    m_p2SpinBox->setDecimals(6);
    m_p2SpinBox->setSingleStep(0.001);
    tangentialLayout->addWidget(m_p2SpinBox);
    tangentialLayout->addStretch();
    distortionLayout->addLayout(tangentialLayout);

    // 薄棱畸变
    QHBoxLayout* thinPrismLayout = new QHBoxLayout();
    thinPrismLayout->addWidget(new QLabel("薄棱：", this));
    thinPrismLayout->addWidget(new QLabel("b1", this));
    m_b1SpinBox = new QDoubleSpinBox(this);
    m_b1SpinBox->setRange(-1, 1);
    m_b1SpinBox->setValue(0);
    m_b1SpinBox->setDecimals(6);
    m_b1SpinBox->setSingleStep(0.001);
    thinPrismLayout->addWidget(m_b1SpinBox);
    thinPrismLayout->addWidget(new QLabel("b2", this));
    m_b2SpinBox = new QDoubleSpinBox(this);
    m_b2SpinBox->setRange(-1, 1);
    m_b2SpinBox->setValue(0);
    m_b2SpinBox->setDecimals(6);
    m_b2SpinBox->setSingleStep(0.001);
    thinPrismLayout->addWidget(m_b2SpinBox);
    thinPrismLayout->addStretch();
    distortionLayout->addLayout(thinPrismLayout);

    mainLayout->addWidget(distortionGroup);

    // ─────────────────────────────────────────────────────────
    // 验证状态
    // ─────────────────────────────────────────────────────────
    
    m_validationStatusLabel = new QLabel("状态：✓ 有效", this);
    m_validationStatusLabel->setStyleSheet("color: green;");
    mainLayout->addWidget(m_validationStatusLabel);

    m_distortionWarningLabel = new QLabel("", this);
    m_distortionWarningLabel->setStyleSheet("color: orange;");
    mainLayout->addWidget(m_distortionWarningLabel);

    mainLayout->addStretch();

    // ─────────────────────────────────────────────────────────
    // 连接信号槽
    // ─────────────────────────────────────────────────────────
    
    connect(m_cameraTypeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &CameraModelWidget::onParameterChanged);
    
    connect(m_widthSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &CameraModelWidget::onParameterChanged);
    connect(m_heightSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &CameraModelWidget::onParameterChanged);
    connect(m_focalLengthSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &CameraModelWidget::onParameterChanged);
    connect(m_sensorWidthSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &CameraModelWidget::onParameterChanged);
    connect(m_sensorHeightSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &CameraModelWidget::onParameterChanged);
    connect(m_pixelSizeSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &CameraModelWidget::onParameterChanged);
    connect(m_focalLength35mmSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &CameraModelWidget::onParameterChanged);
    connect(m_cxSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &CameraModelWidget::onParameterChanged);
    connect(m_cySpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &CameraModelWidget::onParameterChanged);

    connect(m_k1SpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &CameraModelWidget::onParameterChanged);
    connect(m_k2SpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &CameraModelWidget::onParameterChanged);
    connect(m_k3SpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &CameraModelWidget::onParameterChanged);
    connect(m_k4SpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &CameraModelWidget::onParameterChanged);
    connect(m_p1SpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &CameraModelWidget::onParameterChanged);
    connect(m_p2SpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &CameraModelWidget::onParameterChanged);
    connect(m_b1SpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &CameraModelWidget::onParameterChanged);
    connect(m_b2SpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &CameraModelWidget::onParameterChanged);
}

void CameraModelWidget::onParameterChanged()
{
    if (m_blockSignals) {
        return;
    }

    updateCameraModel();
    updateValidationStatus();
    emit cameraModelChanged(m_camera);
}

void CameraModelWidget::updateValidationStatus()
{
    bool isValid = validateCamera();

    if (isValid) {
        m_validationStatusLabel->setText("状态：✓ 有效");
        m_validationStatusLabel->setStyleSheet("color: green;");
    } else {
        m_validationStatusLabel->setText("状态：✗ 无效（焦距或传感器参数异常）");
        m_validationStatusLabel->setStyleSheet("color: red;");
    }

    // 检查畸变
    if (!m_camera.HasDistortion()) {
        m_distortionWarningLabel->setText("⚠ 无畸变参数，假设使用无畸变模型");
        m_distortionWarningLabel->setStyleSheet("color: orange;");
    } else {
        m_distortionWarningLabel->setText("");
    }
}

void CameraModelWidget::updateCameraModel()
{
    m_camera.width = static_cast<uint32_t>(m_widthSpinBox->value());
    m_camera.height = static_cast<uint32_t>(m_heightSpinBox->value());
    m_camera.focal_length = m_focalLengthSpinBox->value();
    m_camera.sensor_width_mm = m_sensorWidthSpinBox->value();
    m_camera.sensor_height_mm = m_sensorHeightSpinBox->value();
    m_camera.pixel_size_um = m_pixelSizeSpinBox->value();
    m_camera.focal_length_35mm = m_focalLength35mmSpinBox->value();
    m_camera.principal_point_x = m_cxSpinBox->value();
    m_camera.principal_point_y = m_cySpinBox->value();

    m_camera.k1 = m_k1SpinBox->value();
    m_camera.k2 = m_k2SpinBox->value();
    m_camera.k3 = m_k3SpinBox->value();
    m_camera.k4 = m_k4SpinBox->value();
    m_camera.p1 = m_p1SpinBox->value();
    m_camera.p2 = m_p2SpinBox->value();
    m_camera.b1 = m_b1SpinBox->value();
    m_camera.b2 = m_b2SpinBox->value();
}

}  // namespace ui
}  // namespace insight
