/**
 * @file ImageGroupDialog.h
 * @brief 图像分组对话框
 * 
 * 功能：
 * 1. 创建或编辑图像分组
 * 2. 设置分组名称和描述
 * 3. 选择相机参数模式（GroupLevel/ImageLevel/RigBased）
 * 4. 根据模式显示相应的配置选项
 */

#ifndef UI_IMAGEGROUPDIALOG_H
#define UI_IMAGEGROUPDIALOG_H

#include <QDialog>
#include <QString>
#include <memory>

#include "database/database_types.h"

class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QComboBox;
class QRadioButton;
class QButtonGroup;

namespace insight {

namespace ui {

class CameraModelWidget;

/**
 * @class ImageGroupDialog
 * @brief 图像分组对话框
 */
class ImageGroupDialog : public QDialog {
    Q_OBJECT

public:
    explicit ImageGroupDialog(
        insight::database::Project* project = nullptr,
        QWidget* parent = nullptr);
    ~ImageGroupDialog();

    /**
     * 获取创建的分组对象
     * 
     * @return ImageGroup 对象引用
     */
    const insight::database::ImageGroup& getImageGroup() const;

signals:
    /**
     * 分组创建完成信号
     */
    void imageGroupCreated(const insight::database::ImageGroup& group);

private slots:
    /**
     * 处理相机模式改变
     */
    void onCameraModeChanged();

    /**
     * 处理创建按钮
     */
    void onCreateGroup();

private:
    /**
     * 初始化UI
     */
    void initializeUI();

    /**
     * 验证输入
     * 
     * @return 有效返回true
     */
    bool validateInput();

    /**
     * 获取下一个可用的分组 ID
     */
    uint32_t getNextGroupId() const;

    // 指向项目的指针
    insight::database::Project* m_project;

    // 分组数据
    insight::database::ImageGroup m_imageGroup;

    // UI 组件
    QLineEdit* m_groupNameEdit;         ///< 分组名称输入框
    QPlainTextEdit* m_descriptionEdit;  ///< 分组描述输入框
    
    // 相机模式选择
    QRadioButton* m_groupLevelRadio;    ///< 组级模式
    QRadioButton* m_imageLevelRadio;    ///< 图像级模式
    QRadioButton* m_rigBasedRadio;      ///< Rig 模式
    QButtonGroup* m_cameraModelGroup;   ///< 模式按钮组
    
    // 组级相机参数（仅在 GroupLevel 模式显示）
    CameraModelWidget* m_cameraWidget;  ///< 相机参数编辑器
    
    // Rig 配置（仅在 RigBased 模式显示）
    QComboBox* m_rigCombo;              ///< Rig 选择下拉框
    QComboBox* m_rigMountCombo;         ///< Rig 内相机选择下拉框
    
    // 按钮
    QPushButton* m_createButton;        ///< 创建按钮
    QPushButton* m_cancelButton;        ///< 取消按钮
};

}  // namespace ui
}  // namespace insight

#endif  // UI_IMAGEGROUPDIALOG_H
