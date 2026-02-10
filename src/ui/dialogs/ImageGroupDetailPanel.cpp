#include "ImageGroupDetailPanel.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QCloseEvent>
#include <QProcess>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QMessageBox>
#include <QApplication>
#include <QFileInfo>
#include <QDir>
#include <glog/logging.h>

#include "models/ProjectDocument.h"
#include "../UISystemConfig.h"

namespace insight::ui::dialogs {

ImageGroupDetailPanel::ImageGroupDetailPanel(QWidget* parent)
    : QDialog(parent),
      m_projectDocument(nullptr),
      m_currentGroup(nullptr) {
    setWindowTitle(tr("Edit Image Group"));
    setWindowModality(Qt::NonModal);
    setMinimumWidth(600);
    setMinimumHeight(800);
    
    InitializeUI();
    ConnectSignals();
}

ImageGroupDetailPanel::~ImageGroupDetailPanel() = default;

void ImageGroupDetailPanel::SetProjectDocument(ProjectDocument* doc) {
    m_projectDocument = doc;
}

void ImageGroupDetailPanel::LoadGroup(database::ImageGroup* group) {
    if (!group) {
        return;
    }

    m_currentGroup = group;

    // 更新窗口标题
    setWindowTitle(tr("Edit Image Group: %1").arg(QString::fromStdString(group->group_name)));

    // 加载相机参数到编辑器
    if (group->group_camera) {
        m_cameraEditor->LoadCamera(*group->group_camera);
    } else {
        // 创建默认相机模型
        database::CameraModel defaultCamera;
        // 注意: camera_mode 属于 ImageGroup，不属于 CameraModel
        m_cameraEditor->LoadCamera(defaultCamera);
    }

    // 设置编辑器的模式
    m_cameraEditor->SetMode(group->camera_mode);

    // 设置分组名称
    m_cameraEditor->SetGroupName(group->group_name);

    // 显示分组名称字段
    m_cameraEditor->ShowGroupNameField(true);

    // 显示对话框
    show();
    raise();
    activateWindow();
}

void ImageGroupDetailPanel::InitializeUI() {
    auto mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(12, 12, 12, 12);
    mainLayout->setSpacing(8);

    // 标题 label
    auto titleLabel = new QLabel(tr("Group: (no selection)"), this);
    titleLabel->setStyleSheet("font-weight: bold; font-size: 13px;");
    mainLayout->addWidget(titleLabel);

    // 相机参数编辑器
    m_cameraEditor = new widgets::CameraParameterEditorWidget(this);
    mainLayout->addWidget(m_cameraEditor);

    // 底部按钮
    auto btnLayout = new QHBoxLayout();
    btnLayout->addStretch();

    auto closeBtn = new QPushButton(tr("Close"), this);
    closeBtn->setMinimumWidth(80);
    connect(closeBtn, &QPushButton::clicked, this, &QDialog::close);
    btnLayout->addWidget(closeBtn);

    mainLayout->addLayout(btnLayout);

    setLayout(mainLayout);
}

void ImageGroupDetailPanel::ConnectSignals() {
    connect(m_cameraEditor, &widgets::CameraParameterEditorWidget::fieldModified,
            this, &ImageGroupDetailPanel::onCameraParameterModified);
    connect(m_cameraEditor, &widgets::CameraParameterEditorWidget::modeChanged,
            this, &ImageGroupDetailPanel::onCameraParameterModeChanged);
    connect(m_cameraEditor, &widgets::CameraParameterEditorWidget::autoEstimateRequested,
            this, &ImageGroupDetailPanel::onAutoEstimateRequested);
}

void ImageGroupDetailPanel::SaveGroupData() {
    if (!m_currentGroup || !m_projectDocument) {
        return;
    }

    // 保存分组名称
    std::string newName = m_cameraEditor->GetGroupName();
    if (newName != m_currentGroup->group_name) {
        m_currentGroup->group_name = newName;
        // 更新窗口标题
        setWindowTitle(tr("Edit Image Group: %1").arg(QString::fromStdString(newName)));
    }

    // 保存相机参数（仅 GroupLevel 模式）
    auto camera = m_cameraEditor->GetCamera();
    if (m_currentGroup->camera_mode == database::ImageGroup::CameraMode::kGroupLevel) {
        m_currentGroup->group_camera = camera;
    }

    // 更新相机模式（从编辑器获取）
    auto newMode = m_cameraEditor->GetMode();
    m_currentGroup->camera_mode = newMode;

    // 通知 ProjectDocument 分组数据已改变
    m_projectDocument->notifyImageGroupChanged(m_currentGroup->group_id);

    // 发出信号
    emit groupDataChanged(m_currentGroup->group_id);

    LOG(INFO) << "Saved image group: " << m_currentGroup->group_name;
}

// ═══════════════════════════════════════════════════════════════
// 槽函数
// ═══════════════════════════════════════════════════════════════

void ImageGroupDetailPanel::onCameraParameterModified() {
    SaveGroupData();
}

void ImageGroupDetailPanel::onCameraParameterModeChanged(database::ImageGroup::CameraMode mode) {
    if (m_currentGroup) {
        m_currentGroup->camera_mode = mode;
        SaveGroupData();
    }
}

void ImageGroupDetailPanel::onGroupNameModified() {
    SaveGroupData();
}

void ImageGroupDetailPanel::onAutoEstimateRequested() {
    if (!m_currentGroup || !m_projectDocument) return;
    if (m_currentGroup->images.empty()) return;

    // 1. 准备输入 JSON
    QJsonObject inputObj;
    QJsonArray imagesArray;
    
    // 获取项目根目录以处理相对路径
    QString projectDir = QFileInfo(m_projectDocument->filepath()).absolutePath();

    for (const auto& img : m_currentGroup->images) {
        QString fullPath = QString::fromStdString(img.filename);
        if (QFileInfo(fullPath).isRelative()) {
            fullPath = projectDir + "/" + fullPath;
        }
        imagesArray.append(fullPath);
    }
    inputObj["image_paths"] = imagesArray;
    
    std::string configPath = UISystemConfig::instance().configPath();
    // 确保是绝对路径
    if (QFileInfo(QString::fromStdString(configPath)).isRelative()) {
        configPath = (qApp->applicationDirPath() + "/" + QString::fromStdString(configPath)).toStdString();
    }
    
    std::string sensorDbPath = configPath + "/camera_sensor_database.txt";
    inputObj["sensor_db_path"] = QString::fromStdString(sensorDbPath);
    
    // 日志目录
    QString logDir = qApp->applicationDirPath() + "/logs";
    QDir().mkpath(logDir);
    inputObj["log_dir"] = logDir;

    QJsonObject root;
    root["estimator_input"] = inputObj;
    QJsonDocument inputDoc(root);
    QByteArray inputData = inputDoc.toJson();

    // 2. 启动子进程
    QApplication::setOverrideCursor(Qt::WaitCursor);
    
    QProcess process;
    QString program = qApp->applicationDirPath() + "/CameraEstimator";
#ifdef Q_OS_WIN
    program += ".exe";
#endif

    process.start(program, QStringList());
    if (!process.waitForStarted()) {
        QApplication::restoreOverrideCursor();
        QMessageBox::critical(this, tr("Error"), tr("Failed to start CameraEstimator algorithm at %1").arg(program));
        return;
    }

    process.write(inputData);
    process.closeWriteChannel();

    if (!process.waitForFinished(600000)) { // 10 mins for many images
        QApplication::restoreOverrideCursor();
        QMessageBox::critical(this, tr("Error"), tr("CameraEstimator timed out or crashed."));
        return;
    }

    QByteArray outputData = process.readAllStandardOutput();
    QByteArray errorData = process.readAllStandardError();
    
    QApplication::restoreOverrideCursor();

    if (process.exitCode() != 0) {
        LOG(ERROR) << "CameraEstimator failed: " << errorData.constData();
        QMessageBox::critical(this, tr("Error"), tr("Algorithm failed:\n%1").arg(QString::fromUtf8(errorData)));
        return;
    }

    // 3. 解析结果
    QJsonDocument outputDoc = QJsonDocument::fromJson(outputData);
    if (outputDoc.isNull()) {
        QMessageBox::critical(this, tr("Error"), tr("Invalid output JSON from algorithm."));
        return;
    }

    QJsonObject rootObj = outputDoc.object();
    QJsonObject outputObj = rootObj["estimator_output"].toObject();
    QJsonArray groupsArray = outputObj["groups"].toArray();

    if (groupsArray.isEmpty()) {
        QMessageBox::warning(this, tr("Warning"), tr("No camera information could be estimated."));
        return;
    }

    // 辅助 lambda: 从 JSON 构建 CameraModel
    auto jsonToCamera = [](const QJsonObject& camObj) {
        database::CameraModel cam;
        cam.make = camObj["make"].toString().toStdString();
        cam.model = camObj["model"].toString().toStdString();
        cam.width = camObj["width"].toInt();
        cam.height = camObj["height"].toInt();
        cam.sensor_width_mm = camObj["sensor_width_mm"].toDouble();
        cam.focal_length = camObj["focal_length_px"].toDouble();
        cam.focal_length_35mm = camObj["focal_length_35mm"].toDouble();
        cam.principal_point_x = cam.width / 2.0;
        cam.principal_point_y = cam.height / 2.0;
        cam.camera_name = cam.make + " " + cam.model;
        return cam;
    };

    // 只有 1 个组的情况
    if (groupsArray.size() == 1) {
        QJsonObject groupObj = groupsArray[0].toObject();
        database::CameraModel cam = jsonToCamera(groupObj["camera"].toObject());
        
        m_currentGroup->group_camera = cam;
        m_cameraEditor->LoadCamera(cam);
        SaveGroupData();
        
        QMessageBox::information(this, tr("Success"), tr("Camera parameters estimated successfully."));
    } 
    // 多组情况：询问是否分裂
    else {
        auto res = QMessageBox::question(this, tr("Split Image Group?"),
            tr("The algorithm detected %1 different camera types/resolutions within this group.\n\n"
               "Do you want to split this group into %1 separate groups?").arg(groupsArray.size()),
            QMessageBox::Yes | QMessageBox::No);

        if (res == QMessageBox::Yes) {
            // 执行分裂
            // 备份原组的所有图像，因为我们会修改原组
            std::vector<database::Image> allImages = m_currentGroup->images;
            std::string originalName = m_currentGroup->group_name;
            uint32_t originalId = m_currentGroup->group_id;

            for (int i = 0; i < groupsArray.size(); ++i) {
                QJsonObject groupInfo = groupsArray[i].toObject();
                database::CameraModel cam = jsonToCamera(groupInfo["camera"].toObject());
                QJsonArray indices = groupInfo["image_indices"].toArray();

                if (i == 0) {
                    // 第 0 组：更新当前组
                    m_currentGroup->images.clear();
                    for (auto idx : indices) {
                        m_currentGroup->images.push_back(allImages[idx.toInt()]);
                    }
                    m_currentGroup->group_camera = cam;
                    m_currentGroup->group_name = originalName + "_1";
                } else {
                    // 其他组：创建新组
                    QString newName = QString::fromStdString(originalName) + "_" + QString::number(i + 1);
                    uint32_t newGroupId = m_projectDocument->createImageGroup(newName, database::ImageGroup::CameraMode::kGroupLevel);
                    
                    // 查找新创建的组
                    for (auto& group : m_projectDocument->project().image_groups) {
                        if (group.group_id == newGroupId) {
                            group.group_camera = cam;
                            group.images.clear();
                            for (auto idx : indices) {
                                group.images.push_back(allImages[idx.toInt()]);
                            }
                            break;
                        }
                    }
                }
            }

            // 刷新 UI
            LoadGroup(m_currentGroup); // 重新加载当前组（虽然它的 ID 没变，但内容变了）
            m_projectDocument->notifyImageGroupChanged(originalId);
            emit groupDataChanged(originalId);
            
            QMessageBox::information(this, tr("Split Completed"), 
                tr("Original group split into %1 groups successfully.").arg(groupsArray.size()));
        }
    }
}

// ═══════════════════════════════════════════════════════════════
// 事件处理
// ═══════════════════════════════════════════════════════════════

void ImageGroupDetailPanel::closeEvent(QCloseEvent* event) {
    // 关闭时确保数据已保存
    SaveGroupData();
    event->accept();
}

}  // namespace insight::ui::dialogs
