/**
 * @file  SpatialReferenceDialog.cpp
 * @brief 空间参考选择对话框实现，使用 UISystemConfig。
 */

#include "spatial_reference_dialog.h"

#include "../system_config.h"
#include "ui/utils/coordinates.h"
#include "../utils/qstring_convert.h"
#include "../../util/string_utils.h"

#include <QDialogButtonBox>
#include <QSettings>
#include <QTreeWidgetItem>

#include <glog/logging.h>

namespace insight {
namespace ui {

SpatialReferenceDialog::SpatialReferenceDialog(QWidget* parent) : QDialog(parent) {
  ui.setupUi(this);
  init();
}

SpatialReferenceDialog::~SpatialReferenceDialog() {}

void SpatialReferenceDialog::init() {
  // 获取树形结构的节点
  _favorite = ui.treeWidget_XYCoord->topLevelItem(2);
  _geo = ui.treeWidget_XYCoord->topLevelItem(1);
  _proj = ui.treeWidget_XYCoord->topLevelItem(0);

  ui.treeWidget_XYCoord->sortByColumn(0, Qt::AscendingOrder);
  ui.buttonBox->button(QDialogButtonBox::Ok)->setEnabled(false);

  // 连接树形控件的点击事件
  connect(ui.treeWidget_XYCoord, &QTreeWidget::clicked, this,
          &SpatialReferenceDialog::on_item_clicked);

  // 连接其他按钮
  connect(ui.pushButton, &QPushButton::clicked, this, &SpatialReferenceDialog::on_add_favorite);
  connect(ui.lineEdit_filter, &QLineEdit::textChanged, this, &SpatialReferenceDialog::on_filter);
  connect(ui.pushButton_clearFilter, &QPushButton::clicked, this,
          &SpatialReferenceDialog::on_clear_filter);
  connect(ui.pushButton_clearFavorite, &QPushButton::clicked, this,
          &SpatialReferenceDialog::on_clear_favorite);

  show_all();
  show_trees();
}

void SpatialReferenceDialog::on_item_clicked(const QModelIndex& index) {
  if (!index.isValid())
    return;
  if (!index.data(Qt::UserRole).isValid())
    return;

  int type = index.data(Qt::UserRole).toInt();
  QString CoordinateName = index.data(Qt::UserRole + 1).toString();

  if (1 == type) {
    // 投影坐标系
    _curCoordinate = _projCoordinate.at(to_s(CoordinateName));
    ui.plainTextEdit_curCoord->setPlainText(to_qs(_curCoordinate.WKT));
    ui.buttonBox->button(QDialogButtonBox::Ok)->setEnabled(true);
  } else if (0 == type) {
    // 地理坐标系
    _curCoordinate = _geoCoordinate.at(to_s(CoordinateName));
    ui.plainTextEdit_curCoord->setPlainText(to_qs(_curCoordinate.WKT));
    ui.buttonBox->button(QDialogButtonBox::Ok)->setEnabled(true);
  } else {
    ui.buttonBox->button(QDialogButtonBox::Ok)->setEnabled(false);
  }
}

Coordinate SpatialReferenceDialog::select_coordinate() const { return _curCoordinate; }

void SpatialReferenceDialog::on_add_favorite() {
  QSettings settings;
  QStringList names = settings.value("favoriteCoordinates", QStringList()).toStringList();

  if (ui.buttonBox->button(QDialogButtonBox::Ok)->isEnabled()) {
    QString coordName = to_qs(_curCoordinate.CoordinateName);
    // 检查是否已经在收藏列表中
    if (!names.contains(coordName)) {
      names.push_back(coordName);
      settings.setValue("favoriteCoordinates", names);
      show_trees();
    }
  }
}

void SpatialReferenceDialog::on_filter() {
  QString nm = ui.lineEdit_filter->text();
  if (nm.isEmpty()) {
    show_all();
  } else {
    show_some(nm);
  }
  show_trees();
}

void SpatialReferenceDialog::on_clear_filter() { ui.lineEdit_filter->clear(); }

void SpatialReferenceDialog::show_trees() {
  // 清空地理坐标系节点
  QList<QTreeWidgetItem*> items = _geo->takeChildren();
  for (QTreeWidgetItem* item : items) {
    delete item;
  }
  items.clear();

  // 添加地理坐标系
  for (auto& pair : _geoCoordinate) {
    QTreeWidgetItem* item = new QTreeWidgetItem();
    item->setText(0, to_qs(pair.second.CoordinateName));
    item->setText(1, to_qs(pair.second.EPSGName));
    item->setData(0, Qt::UserRole, 0); // 0 = 地理坐标系
    item->setData(1, Qt::UserRole, 0);
    item->setData(0, Qt::UserRole + 1, to_qs(pair.second.CoordinateName));
    item->setData(1, Qt::UserRole + 1, to_qs(pair.second.CoordinateName));
    _geo->addChild(item);
  }

  // 清空投影坐标系节点
  items = _proj->takeChildren();
  for (QTreeWidgetItem* item : items) {
    delete item;
  }
  items.clear();

  // 添加投影坐标系
  for (auto& pair : _projCoordinate) {
    QTreeWidgetItem* item = new QTreeWidgetItem();
    item->setText(0, to_qs(pair.second.CoordinateName));
    item->setText(1, to_qs(pair.second.EPSGName));
    item->setData(0, Qt::UserRole, 1); // 1 = 投影坐标系
    item->setData(1, Qt::UserRole, 1);
    item->setData(0, Qt::UserRole + 1, to_qs(pair.second.CoordinateName));
    item->setData(1, Qt::UserRole + 1, to_qs(pair.second.CoordinateName));
    _proj->addChild(item);
  }

  // 添加收藏坐标系
  QSettings settings;
  QStringList names = settings.value("favoriteCoordinates", QStringList()).toStringList();
  items = _favorite->takeChildren();
  for (QTreeWidgetItem* item : items) {
    delete item;
  }
  items.clear();

  for (QString& s : names) {
    auto it = _geoCoordinate.find(to_s(s));
    if (it != _geoCoordinate.end()) {
      QTreeWidgetItem* item = new QTreeWidgetItem();
      item->setText(0, to_qs(it->second.CoordinateName));
      item->setText(1, to_qs(it->second.EPSGName));
      item->setData(0, Qt::UserRole, 0);
      item->setData(1, Qt::UserRole, 0);
      item->setData(0, Qt::UserRole + 1, to_qs(it->second.CoordinateName));
      item->setData(1, Qt::UserRole + 1, to_qs(it->second.CoordinateName));
      _favorite->addChild(item);
    } else {
      it = _projCoordinate.find(to_s(s));
      if (it != _projCoordinate.end()) {
        QTreeWidgetItem* item = new QTreeWidgetItem();
        item->setText(0, to_qs(it->second.CoordinateName));
        item->setText(1, to_qs(it->second.EPSGName));
        item->setData(0, Qt::UserRole, 1);
        item->setData(1, Qt::UserRole, 1);
        item->setData(0, Qt::UserRole + 1, to_qs(it->second.CoordinateName));
        item->setData(1, Qt::UserRole + 1, to_qs(it->second.CoordinateName));
        _favorite->addChild(item);
      }
    }
  }
}

void SpatialReferenceDialog::show_all() {
  // 使用 UISystemConfig 获取所有坐标系
  UISystemConfig& config = UISystemConfig::instance();

  _geoCoordinate.clear();
  _projCoordinate.clear();

  const auto& geoCoords = config.get_geo_coordinates();
  for (const auto& coord : geoCoords) {
    _geoCoordinate[coord.CoordinateName] = coord;
  }

  const auto& projCoords = config.get_proj_coordinates();
  for (const auto& coord : projCoords) {
    _projCoordinate[coord.CoordinateName] = coord;
  }
}

void SpatialReferenceDialog::show_some(const QString& name) {
  // 使用 UISystemConfig 过滤坐标系
  UISystemConfig& config = UISystemConfig::instance();

  _geoCoordinate.clear();
  _projCoordinate.clear();

  const auto& geoCoords = config.get_geo_coordinates();
  for (const auto& coord : geoCoords) {
    // 按坐标系名称搜索
    bool matchName = to_qs(coord.CoordinateName).contains(name, Qt::CaseInsensitive);
    // 按EPSG代码搜索（支持 "EPSG:4326" 或 "4326" 的格式）
    bool matchEPSG = to_qs(coord.EPSGName).contains(name, Qt::CaseInsensitive);

    if (matchName || matchEPSG) {
      _geoCoordinate[coord.CoordinateName] = coord;
    }
  }

  const auto& projCoords = config.get_proj_coordinates();
  for (const auto& coord : projCoords) {
    // 按坐标系名称搜索
    bool matchName = to_qs(coord.CoordinateName).contains(name, Qt::CaseInsensitive);
    // 按EPSG代码搜索（支持 "EPSG:4326" 或 "4326" 的格式）
    bool matchEPSG = to_qs(coord.EPSGName).contains(name, Qt::CaseInsensitive);

    if (matchName || matchEPSG) {
      _projCoordinate[coord.CoordinateName] = coord;
    }
  }
}

void SpatialReferenceDialog::on_clear_favorite() {
  QSettings settings;
  settings.setValue("favoriteCoordinates", QStringList());
  show_trees();
}

}  // namespace ui
}  // namespace insight
