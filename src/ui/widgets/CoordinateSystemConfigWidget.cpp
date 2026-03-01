#include "CoordinateSystemConfigWidget.h"
#include "SpatialReferenceDialog.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QMessageBox>
#include <QDebug>

namespace insight {
namespace ui {

CoordinateSystemConfigWidget::CoordinateSystemConfigWidget(QWidget* parent)
    : QWidget(parent), m_currentType(database::CoordinateSystem::Type::kLocal) {
    setWindowTitle("Configure Coordinate System");
    initializeUI();
    connectSignals();
    updateUIState();
}

void CoordinateSystemConfigWidget::initializeUI() {
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setSpacing(10);
    mainLayout->setContentsMargins(10, 10, 10, 10);

    // ════════════════════════════════════════════════════════
    // 坐标系类型选择组
    // ════════════════════════════════════════════════════════
    QGroupBox* typeGroupBox = new QGroupBox("Coordinate System Type", this);
    QVBoxLayout* typeLayout = new QVBoxLayout(typeGroupBox);

    m_radioLocal = new QRadioButton("Local Coordinate System", this);
    m_radioEPSG = new QRadioButton("EPSG Code", this);
    m_radioENU = new QRadioButton("ENU (East-North-Up)", this);
    m_radioWKT = new QRadioButton("WKT Definition", this);

    // 默认选中 LOCAL
    m_radioLocal->setChecked(true);
    m_currentType = database::CoordinateSystem::Type::kLocal;

    typeLayout->addWidget(m_radioLocal);
    typeLayout->addWidget(m_radioEPSG);
    typeLayout->addWidget(m_radioENU);
    typeLayout->addWidget(m_radioWKT);
    typeGroupBox->setLayout(typeLayout);
    mainLayout->addWidget(typeGroupBox);

    // ════════════════════════════════════════════════════════
    // 动态内容区（QStackedWidget）
    // ════════════════════════════════════════════════════════
    m_stackedWidget = new QStackedWidget(this);

    // 页面 0: LOCAL（空页面）
    m_pageLocal = new QWidget();
    QVBoxLayout* localLayout = new QVBoxLayout(m_pageLocal);
    localLayout->addWidget(new QLabel("No additional configuration required for Local Coordinate System."));
    m_stackedWidget->addWidget(m_pageLocal);

    // 页面 1: EPSG
    m_pageEPSG = new QWidget();
    QVBoxLayout* epsgLayout = new QVBoxLayout(m_pageEPSG);
    QHBoxLayout* epsgInputLayout = new QHBoxLayout();
    m_epsgEdit = new QLineEdit(this);
    m_epsgEdit->setReadOnly(true);
    m_epsgEdit->setPlaceholderText("Click Browse to select EPSG code...");
    m_epsgBrowseBtn = new QPushButton("Browse", this);
    epsgInputLayout->addWidget(new QLabel("EPSG Code:"));
    epsgInputLayout->addWidget(m_epsgEdit);
    epsgInputLayout->addWidget(m_epsgBrowseBtn);
    epsgLayout->addLayout(epsgInputLayout);
    m_epsgErrorLabel = new QLabel(this);
    m_epsgErrorLabel->setStyleSheet("color: red;");
    epsgLayout->addWidget(m_epsgErrorLabel);
    epsgLayout->addStretch();
    m_pageEPSG->setLayout(epsgLayout);
    m_stackedWidget->addWidget(m_pageEPSG);

    // 页面 2: ENU
    m_pageENU = new QWidget();
    QVBoxLayout* enuLayout = new QVBoxLayout(m_pageENU);

    // Reference Point (纬度、经度、高度)
    QGroupBox* refPointBox = new QGroupBox("Reference Point (WGS84)", m_pageENU);
    QGridLayout* refPointLayout = new QGridLayout(refPointBox);
    m_enuRefLatSpinBox = new QDoubleSpinBox();
    m_enuRefLatSpinBox->setRange(-90.0, 90.0);
    m_enuRefLatSpinBox->setDecimals(6);
    m_enuRefLatSpinBox->setSingleStep(0.000001);
    m_enuRefLatSpinBox->setValue(0.0);
    m_enuRefLonSpinBox = new QDoubleSpinBox();
    m_enuRefLonSpinBox->setRange(-180.0, 180.0);
    m_enuRefLonSpinBox->setDecimals(6);
    m_enuRefLonSpinBox->setSingleStep(0.000001);
    m_enuRefLonSpinBox->setValue(0.0);
    m_enuRefAltSpinBox = new QDoubleSpinBox();
    m_enuRefAltSpinBox->setRange(-1e6, 1e6);
    m_enuRefAltSpinBox->setDecimals(2);
    m_enuRefAltSpinBox->setSingleStep(1.0);
    m_enuRefAltSpinBox->setValue(0.0);

    refPointLayout->addWidget(new QLabel("Latitude (°):"), 0, 0);
    refPointLayout->addWidget(m_enuRefLatSpinBox, 0, 1);
    refPointLayout->addWidget(new QLabel("Longitude (°):"), 1, 0);
    refPointLayout->addWidget(m_enuRefLonSpinBox, 1, 1);
    refPointLayout->addWidget(new QLabel("Altitude (m):"), 2, 0);
    refPointLayout->addWidget(m_enuRefAltSpinBox, 2, 1);
    refPointBox->setLayout(refPointLayout);
    enuLayout->addWidget(refPointBox);

    // Origin (可选)
    QGroupBox* originBox = new QGroupBox("Local Origin (Optional)", m_pageENU);
    QGridLayout* originLayout = new QGridLayout(originBox);
    m_enuOriginXSpinBox = new QDoubleSpinBox();
    m_enuOriginXSpinBox->setRange(-1e9, 1e9);
    m_enuOriginXSpinBox->setDecimals(3);
    m_enuOriginXSpinBox->setSingleStep(1.0);
    m_enuOriginXSpinBox->setValue(0.0);
    m_enuOriginYSpinBox = new QDoubleSpinBox();
    m_enuOriginYSpinBox->setRange(-1e9, 1e9);
    m_enuOriginYSpinBox->setDecimals(3);
    m_enuOriginYSpinBox->setSingleStep(1.0);
    m_enuOriginYSpinBox->setValue(0.0);
    m_enuOriginZSpinBox = new QDoubleSpinBox();
    m_enuOriginZSpinBox->setRange(-1e9, 1e9);
    m_enuOriginZSpinBox->setDecimals(3);
    m_enuOriginZSpinBox->setSingleStep(1.0);
    m_enuOriginZSpinBox->setValue(0.0);

    originLayout->addWidget(new QLabel("Origin X (m):"), 0, 0);
    originLayout->addWidget(m_enuOriginXSpinBox, 0, 1);
    originLayout->addWidget(new QLabel("Origin Y (m):"), 1, 0);
    originLayout->addWidget(m_enuOriginYSpinBox, 1, 1);
    originLayout->addWidget(new QLabel("Origin Z (m):"), 2, 0);
    originLayout->addWidget(m_enuOriginZSpinBox, 2, 1);
    originBox->setLayout(originLayout);
    enuLayout->addWidget(originBox);

    m_enuErrorLabel = new QLabel(this);
    m_enuErrorLabel->setStyleSheet("color: red;");
    enuLayout->addWidget(m_enuErrorLabel);
    enuLayout->addStretch();
    m_pageENU->setLayout(enuLayout);
    m_stackedWidget->addWidget(m_pageENU);

    // 页面 3: WKT
    m_pageWKT = new QWidget();
    QVBoxLayout* wktLayout = new QVBoxLayout(m_pageWKT);
    QHBoxLayout* wktBtnLayout = new QHBoxLayout();
    m_wktBrowseBtn = new QPushButton("Browse (Select from Database)", this);
    wktBtnLayout->addStretch();
    wktBtnLayout->addWidget(m_wktBrowseBtn);
    wktLayout->addLayout(wktBtnLayout);
    m_wktEdit = new QPlainTextEdit(this);
    m_wktEdit->setPlaceholderText("Enter WKT definition or click Browse to select...");
    m_wktEdit->setMinimumHeight(120);
    wktLayout->addWidget(new QLabel("WKT Definition (Editable):"));
    wktLayout->addWidget(m_wktEdit);
    m_wktErrorLabel = new QLabel(this);
    m_wktErrorLabel->setStyleSheet("color: red;");
    wktLayout->addWidget(m_wktErrorLabel);
    m_pageWKT->setLayout(wktLayout);
    m_stackedWidget->addWidget(m_pageWKT);

    mainLayout->addWidget(m_stackedWidget);

    // ════════════════════════════════════════════════════════
    // RotationConvention 选择
    // ════════════════════════════════════════════════════════
    QGroupBox* rotationGroupBox = new QGroupBox("Rotation Convention", this);
    QVBoxLayout* rotationLayout = new QVBoxLayout(rotationGroupBox);

    m_radioPhotogrammetry = new QRadioButton("Photogrammetry (ω, φ, κ)", this);
    m_radioAerospace = new QRadioButton("Aerospace (Yaw, Pitch, Roll)", this);

    // 默认选中摄影测量
    m_radioPhotogrammetry->setChecked(true);

    rotationLayout->addWidget(m_radioPhotogrammetry);
    rotationLayout->addWidget(m_radioAerospace);
    rotationGroupBox->setLayout(rotationLayout);
    mainLayout->addWidget(rotationGroupBox);

    mainLayout->addStretch();
    setLayout(mainLayout);
}

void CoordinateSystemConfigWidget::connectSignals() {
    // 坐标系类型选择
    connect(m_radioLocal, QOverload<bool>::of(&QRadioButton::toggled),
            this, &CoordinateSystemConfigWidget::onCoordinateTypeChanged);
    connect(m_radioEPSG, QOverload<bool>::of(&QRadioButton::toggled),
            this, &CoordinateSystemConfigWidget::onCoordinateTypeChanged);
    connect(m_radioENU, QOverload<bool>::of(&QRadioButton::toggled),
            this, &CoordinateSystemConfigWidget::onCoordinateTypeChanged);
    connect(m_radioWKT, QOverload<bool>::of(&QRadioButton::toggled),
            this, &CoordinateSystemConfigWidget::onCoordinateTypeChanged);

    // EPSG 相关
    connect(m_epsgBrowseBtn, &QPushButton::clicked, this, &CoordinateSystemConfigWidget::onEPSGBrowse);
    connect(m_epsgEdit, &QLineEdit::textChanged, this, &CoordinateSystemConfigWidget::onEPSGTextChanged);

    // ENU 相关
    connect(m_enuRefLatSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &CoordinateSystemConfigWidget::onENUReferenceChanged);
    connect(m_enuRefLonSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &CoordinateSystemConfigWidget::onENUReferenceChanged);
    connect(m_enuRefAltSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &CoordinateSystemConfigWidget::onENUReferenceChanged);
    connect(m_enuOriginXSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &CoordinateSystemConfigWidget::onENUOriginChanged);
    connect(m_enuOriginYSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &CoordinateSystemConfigWidget::onENUOriginChanged);
    connect(m_enuOriginZSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &CoordinateSystemConfigWidget::onENUOriginChanged);

    // WKT 相关
    connect(m_wktBrowseBtn, &QPushButton::clicked, this, &CoordinateSystemConfigWidget::onWKTBrowse);
    connect(m_wktEdit, &QPlainTextEdit::textChanged, this, &CoordinateSystemConfigWidget::onWKTTextChanged);

    // RotationConvention
    connect(m_radioPhotogrammetry, QOverload<bool>::of(&QRadioButton::toggled),
            this, &CoordinateSystemConfigWidget::onRotationConventionChanged);
}

void CoordinateSystemConfigWidget::updateUIState() {
    // 更新 QStackedWidget 页面
    if (m_radioLocal->isChecked()) {
        m_stackedWidget->setCurrentWidget(m_pageLocal);
        m_currentType = database::CoordinateSystem::Type::kLocal;
    } else if (m_radioEPSG->isChecked()) {
        m_stackedWidget->setCurrentWidget(m_pageEPSG);
        m_currentType = database::CoordinateSystem::Type::kEPSG;
    } else if (m_radioENU->isChecked()) {
        m_stackedWidget->setCurrentWidget(m_pageENU);
        m_currentType = database::CoordinateSystem::Type::kENU;
    } else if (m_radioWKT->isChecked()) {
        m_stackedWidget->setCurrentWidget(m_pageWKT);
        m_currentType = database::CoordinateSystem::Type::kWKT;
    }

    validateForm();
}

void CoordinateSystemConfigWidget::validateForm() {
    bool valid = false;

    switch (m_currentType) {
        case database::CoordinateSystem::Type::kLocal:
            valid = validateLocalMode();
            break;
        case database::CoordinateSystem::Type::kEPSG:
            valid = validateEPSGMode();
            break;
        case database::CoordinateSystem::Type::kENU:
            valid = validateENUMode();
            break;
        case database::CoordinateSystem::Type::kWKT:
            valid = validateWKTMode();
            break;
    }

    if (m_isValid != valid) {
        m_isValid = valid;
        emit validationChanged(valid);
    }
}

bool CoordinateSystemConfigWidget::validateLocalMode() const {
    // LOCAL 总是有效
    return true;
}

bool CoordinateSystemConfigWidget::validateEPSGMode() const {
    // EPSG 需要非空的 definition
    bool valid = !m_epsgEdit->text().isEmpty();
    m_epsgErrorLabel->setText(valid ? "" : "EPSG code is required");
    return valid;
}

bool CoordinateSystemConfigWidget::validateENUMode() const {
    // ENU：纬度 [-90, 90]，经度 [-180, 180]
    double lat = m_enuRefLatSpinBox->value();
    double lon = m_enuRefLonSpinBox->value();

    bool latValid = lat >= -90.0 && lat <= 90.0;
    bool lonValid = lon >= -180.0 && lon <= 180.0;
    bool valid = latValid && lonValid;

    QString errorMsg;
    if (!latValid) {
        errorMsg += "Latitude must be in [-90, 90]°. ";
    }
    if (!lonValid) {
        errorMsg += "Longitude must be in [-180, 180]°. ";
    }
    m_enuErrorLabel->setText(errorMsg);
    return valid;
}

bool CoordinateSystemConfigWidget::validateWKTMode() const {
    // WKT：至少包含"PROJCS"或"GEOGCS"
    QString wkt = m_wktEdit->toPlainText().trimmed();
    bool valid = wkt.contains("PROJCS") || wkt.contains("GEOGCS");
    m_wktErrorLabel->setText(valid ? "" : "WKT must contain 'PROJCS' or 'GEOGCS'");
    return valid;
}

void CoordinateSystemConfigWidget::onCoordinateTypeChanged() {
    updateUIState();
}

void CoordinateSystemConfigWidget::onEPSGBrowse() {
    SpatialReferenceDialog dialog(this);
    if (dialog.exec() == QDialog::Accepted) {
        // 获取选择的坐标系
        auto coord = dialog.SelectCoordinate();
        // 填充 EPSG code
        m_epsgEdit->setText(QString::fromStdString(coord.EPSGName));
        validateForm();
    }
}

void CoordinateSystemConfigWidget::onEPSGTextChanged() {
    validateForm();
}

void CoordinateSystemConfigWidget::onENUReferenceChanged() {
    validateForm();
}

void CoordinateSystemConfigWidget::onENUOriginChanged() {
    // Origin 是可选的，只需要验证 Reference Point
    validateForm();
}

void CoordinateSystemConfigWidget::onWKTBrowse() {
    SpatialReferenceDialog dialog(this);
    if (dialog.exec() == QDialog::Accepted) {
        auto coord = dialog.SelectCoordinate();
        // 填充 WKT definition
        m_wktEdit->setPlainText(QString::fromStdString(coord.WKT));
        validateForm();
    }
}

void CoordinateSystemConfigWidget::onWKTTextChanged() {
    validateForm();
}

void CoordinateSystemConfigWidget::onRotationConventionChanged() {
    // 旋转规约已更新，无需特殊验证
}

database::CoordinateSystem CoordinateSystemConfigWidget::GetCoordinateSystem() const {
    database::CoordinateSystem coordSys;

    // 坐标系类型和定义
    coordSys.type = m_currentType;

    switch (m_currentType) {
        case database::CoordinateSystem::Type::kLocal:
            // LOCAL 无需定义
            break;

        case database::CoordinateSystem::Type::kEPSG:
            coordSys.definition = m_epsgEdit->text().toStdString();
            break;

        case database::CoordinateSystem::Type::kENU: {
            // 构建参考点
            database::CoordinateSystem::ReferencePoint ref;
            ref.lat = m_enuRefLatSpinBox->value();
            ref.lon = m_enuRefLonSpinBox->value();
            ref.alt = m_enuRefAltSpinBox->value();
            coordSys.reference = ref;

            // 构建原点（如果不为0）
            double ox = m_enuOriginXSpinBox->value();
            double oy = m_enuOriginYSpinBox->value();
            double oz = m_enuOriginZSpinBox->value();
            if (ox != 0.0 || oy != 0.0 || oz != 0.0) {
                database::CoordinateSystem::Origin origin;
                origin.x = ox;
                origin.y = oy;
                origin.z = oz;
                coordSys.origin = origin;
            }
            break;
        }

        case database::CoordinateSystem::Type::kWKT:
            coordSys.definition = m_wktEdit->toPlainText().toStdString();
            break;
    }

    // RotationConvention
    if (m_radioPhotogrammetry->isChecked()) {
        coordSys.rotation_convention = database::CoordinateSystem::RotationConvention::kOmegaPhiKappa;
    } else {
        coordSys.rotation_convention = database::CoordinateSystem::RotationConvention::kYawPitchRoll;
    }

    return coordSys;
}

void CoordinateSystemConfigWidget::SetCoordinateSystem(const database::CoordinateSystem& coordSys) {
    // 设置坐标系类型
    switch (coordSys.type) {
        case database::CoordinateSystem::Type::kLocal:
            m_radioLocal->setChecked(true);
            break;
        case database::CoordinateSystem::Type::kEPSG:
            m_radioEPSG->setChecked(true);
            m_epsgEdit->setText(QString::fromStdString(coordSys.definition));
            break;
        case database::CoordinateSystem::Type::kENU:
            m_radioENU->setChecked(true);
            if (coordSys.reference.has_value()) {
                m_enuRefLatSpinBox->setValue(coordSys.reference->lat);
                m_enuRefLonSpinBox->setValue(coordSys.reference->lon);
                m_enuRefAltSpinBox->setValue(coordSys.reference->alt);
            }
            if (coordSys.origin.has_value()) {
                m_enuOriginXSpinBox->setValue(coordSys.origin->x);
                m_enuOriginYSpinBox->setValue(coordSys.origin->y);
                m_enuOriginZSpinBox->setValue(coordSys.origin->z);
            }
            break;
        case database::CoordinateSystem::Type::kWKT:
            m_radioWKT->setChecked(true);
            m_wktEdit->setPlainText(QString::fromStdString(coordSys.definition));
            break;
    }

    // 设置 RotationConvention
    if (coordSys.rotation_convention == database::CoordinateSystem::RotationConvention::kOmegaPhiKappa) {
        m_radioPhotogrammetry->setChecked(true);
    } else {
        m_radioAerospace->setChecked(true);
    }

    updateUIState();
}

bool CoordinateSystemConfigWidget::IsValid() const {
    return m_isValid;
}

}  // namespace ui
}  // namespace insight
