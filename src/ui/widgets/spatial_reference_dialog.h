/**
 * @file  SpatialReferenceDialog.h
 * @brief 空间参考选择对话框（EPSG/WKT 等）。
 */

#ifndef UI_WIDGETS_SPATIALREFERENCEDIALOG_H
#define UI_WIDGETS_SPATIALREFERENCEDIALOG_H

#include "ui/utils/coordinates.h"
#include "ui_spatial_reference_tool.h"
#include <QDialog>

namespace insight {
namespace ui {

class SpatialReferenceDialog : public QDialog {
  Q_OBJECT

public:
  SpatialReferenceDialog(QWidget* parent = nullptr);
  ~SpatialReferenceDialog();

public:
  Coordinate select_coordinate() const;

private slots:
  void on_item_clicked(const QModelIndex& index);
  void on_add_favorite();
  void on_filter();
  void on_clear_filter();
  void on_clear_favorite();

private:
  Ui::SpatialReferenceTool ui;
  void init();
  void show_trees();
  void show_all();
  void show_some(const QString& name);

  QTreeWidgetItem* _favorite;
  QTreeWidgetItem* _geo;
  QTreeWidgetItem* _proj;
  Coordinate _curCoordinate;
  std::map<std::string, Coordinate> _geoCoordinate;
  std::map<std::string, Coordinate> _projCoordinate;
};

}  // namespace ui
}  // namespace insight

#endif  // UI_WIDGETS_SPATIALREFERENCEDIALOG_H
