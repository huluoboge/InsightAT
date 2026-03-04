/**
 * @file  GPSPointsWizardDialog.h
 * @brief 控制点/测量数据向导对话框基类，字段配置与预览表。
 */

#ifndef WATCONTROLPOINTSWIZARDDIALOG_H
#define WATCONTROLPOINTSWIZARDDIALOG_H

#include "gps_points_wizard_delegate.h"
#include "gps_points_wizard_model.h"
#include "ui_gps_points_wizard_dialog.h"
#include "ui/utils/coordinates.h"

#include <Eigen/Core>

#include <QDialog>

namespace insight {

/**
 * @struct FieldConfiguration
 * @brief 字段配置结构 - 定义导入对话框支持的字段
 */
struct FieldConfiguration {
  QList<QString> requiredFields; ///< 必需字段列表
  QList<QString> optionalFields; ///< 可选字段列表（如旋转角度）

  FieldConfiguration() = default;

  /**
   * 获取所有字段（必需 + 可选）
   */
  QList<QString> getAllFields() const {
    QList<QString> all = requiredFields;
    all.append(optionalFields);
    return all;
  }
};

class GPSPointsWizardDialog : public QDialog, ImportDataBaseDocument {
  Q_OBJECT

public:
  GPSPointsWizardDialog(QWidget* parent = 0);
  ~GPSPointsWizardDialog();
  void setFile(const QString& fileFullPath);

  GPSPointsDocument* get_doc() { return &m_document; }
  bool is_import_by_name() const;

  /**
   * @brief 是否导入旋转数据
   * @return true 表示用户勾选了导入旋转
   */
  bool is_import_rotation() const;

public slots:
  void update_model();
  void preview();
  void check_enable_preview();
  void valid_import();
  void enable_select_import_option(bool enable);

protected:
  bool valid();
  void get_field_index(int& rowFrom, std::vector<int>& fieldIndex);

  virtual FieldConfiguration get_field_configuration() const;

  virtual bool show_custom_dialog() { return true; }

  // ImportDataBaseDocument interface
  QList<QString> field_names() const override;
  bool check_field_data(int rowFrom, const std::vector<int>& fieldIndex) override;

  Ui::GPSPointsWizardDialog ui;
  GPSPointsDocument m_document;
  GPSPointsWizardModel* m_model;
};

}  // namespace insight

#endif  // WATCONTROLPOINTSWIZARDDIALOG_H
