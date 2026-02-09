#ifndef WATCONTROLPOINTSWIZARDDIALOG_H
#define WATCONTROLPOINTSWIZARDDIALOG_H

#include <QDialog>
#include "ui_GPSPointsWizardDialog.h"

#include "GPSPointsWizardModel.h"

#include <Eigen/Core>

#include "GPSPointsWizardDelegate.h"
#include "Common/Coordinates.h"

namespace insight{

/**
 * @struct FieldConfiguration
 * @brief 字段配置结构 - 定义导入对话框支持的字段
 */
struct FieldConfiguration
{
    QList<QString> requiredFields;    ///< 必需字段列表
    QList<QString> optionalFields;    ///< 可选字段列表（如旋转角度）
    
    FieldConfiguration() = default;
    
    /**
     * 获取所有字段（必需 + 可选）
     */
    QList<QString> getAllFields() const
    {
        QList<QString> all = requiredFields;
        all.append(optionalFields);
        return all;
    }
};

class GPSPointsWizardDialog : public QDialog, ImportDataBaseDocument
{
	Q_OBJECT

public:
	
    GPSPointsWizardDialog(QWidget *parent = 0);
    ~GPSPointsWizardDialog();
	void setFile(const QString &fileFullPath);
	
    GPSPointsDocument *getDoc() { return &m_document; }
	bool isImportByName()const;
	
	/**
	 * @brief 是否导入旋转数据
	 * @return true 表示用户勾选了导入旋转
	 */
	bool isImportRotation() const;
	
public slots:
	void updateModel();
	void preview();              // 预览
	void checkEnablePreview();
	void validImport();
	void enableSelectImportOption(bool enable);
	
	// 不再使用：hasOmegaPhiKappa(), angleUnit(), coordinateSystem(), eulerAngleSystem()
	// 这些旋转相关的参数应该在坐标系设置中配置，不在这里

protected:
	bool valid();
	void getFieldIndex(int &rowFrom, std::vector<int> &fieldIndex);
	
	/**
	 * @brief 获取字段配置 - 子类可以重写以定制字段
	 * @return 字段配置结构
	 */
	virtual FieldConfiguration getFieldConfiguration() const;
	
	virtual bool showCustemDialog() { return true; }
	
    Ui::GPSPointsWizardDialog ui;
    GPSPointsDocument m_document;
    GPSPointsWizardModel *m_model;
};

}//name space insight

#endif // WATCONTROLPOINTSWIZARDDIALOG_H
