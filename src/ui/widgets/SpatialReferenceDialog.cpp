/**
 * @file SpatialReferenceDialog.cpp
 * @brief 空间参考工具对话框实现 - 使用 UISystemConfig
 */

#include "SpatialReferenceDialog.h"
#include "UISystemConfig.h"
#include "../../Common/Coordinates.h"
#include "../../Common/string_utils.h"
#include "../Gui/Utils.h"
#include <QSettings>
#include <QTreeWidgetItem>
#include <QDialogButtonBox>
#include <glog/logging.h>

namespace insight {
namespace ui {

SpatialReferenceDialog::SpatialReferenceDialog(QWidget *parent)
    : QDialog(parent)
{
    ui.setupUi(this);
    init();
}

SpatialReferenceDialog::~SpatialReferenceDialog()
{
}

void SpatialReferenceDialog::init()
{
    // 获取树形结构的节点
    _favorite = ui.treeWidget_XYCoord->topLevelItem(2);
    _geo = ui.treeWidget_XYCoord->topLevelItem(1);
    _proj = ui.treeWidget_XYCoord->topLevelItem(0);
    
    ui.treeWidget_XYCoord->sortByColumn(0, Qt::AscendingOrder);
    ui.buttonBox->button(QDialogButtonBox::Ok)->setEnabled(false);
    
    // 连接树形控件的点击事件
    connect(ui.treeWidget_XYCoord, &QTreeWidget::clicked, 
            this, &SpatialReferenceDialog::onItemClicked);
    
    // 连接其他按钮
    connect(ui.pushButton, &QPushButton::clicked, 
            this, &SpatialReferenceDialog::onAddFavorite);
    connect(ui.lineEdit_filter, &QLineEdit::textChanged,
            this, &SpatialReferenceDialog::onFilter);
    connect(ui.pushButton_clearFilter, &QPushButton::clicked,
            this, &SpatialReferenceDialog::onClearFilter);
    connect(ui.pushButton_clearFavorite, &QPushButton::clicked,
            this, &SpatialReferenceDialog::onClearFavorite);
    
    showAll();
    showTrees();
}

void SpatialReferenceDialog::onItemClicked(const QModelIndex &index)
{
    if (!index.isValid()) return;
    if (!index.data(Qt::UserRole).isValid()) return;
    
    int type = index.data(Qt::UserRole).toInt();
    QString CoordinateName = index.data(Qt::UserRole + 1).toString();
    
    if (1 == type) {
        // 投影坐标系
        _curCoordinate = _projCoordinate.at(tos(CoordinateName));
        ui.plainTextEdit_curCoord->setPlainText(toqs(_curCoordinate.WKT));
        ui.buttonBox->button(QDialogButtonBox::Ok)->setEnabled(true);
    } else if (0 == type) {
        // 地理坐标系
        _curCoordinate = _geoCoordinate.at(tos(CoordinateName));
        ui.plainTextEdit_curCoord->setPlainText(toqs(_curCoordinate.WKT));
        ui.buttonBox->button(QDialogButtonBox::Ok)->setEnabled(true);
    } else {
        ui.buttonBox->button(QDialogButtonBox::Ok)->setEnabled(false);
    }
}

Coordinate SpatialReferenceDialog::SelectCoordinate() const
{
    return _curCoordinate;
}

void SpatialReferenceDialog::onAddFavorite()
{
    QSettings settings;
    QStringList names = settings.value("favoriteCoordinates", QStringList()).toStringList();
    
    if (ui.buttonBox->button(QDialogButtonBox::Ok)->isEnabled()) {
        QString coordName = toqs(_curCoordinate.CoordinateName);
        // 检查是否已经在收藏列表中
        if (!names.contains(coordName)) {
            names.push_back(coordName);
            settings.setValue("favoriteCoordinates", names);
            showTrees();
        }
    }
}

void SpatialReferenceDialog::onFilter()
{
    QString nm = ui.lineEdit_filter->text();
    if (nm.isEmpty()) {
        showAll();
    } else {
        showSome(nm);
    }
    showTrees();
}

void SpatialReferenceDialog::onClearFilter()
{
    ui.lineEdit_filter->clear();
}

void SpatialReferenceDialog::showTrees()
{
    // 清空地理坐标系节点
    QList<QTreeWidgetItem *> items = _geo->takeChildren();
    for (QTreeWidgetItem *item : items) {
        delete item;
    }
    items.clear();
    
    // 添加地理坐标系
    for (auto& pair : _geoCoordinate) {
        QTreeWidgetItem *item = new QTreeWidgetItem();
        item->setText(0, toqs(pair.second.CoordinateName));
        item->setText(1, toqs(pair.second.EPSGName));
        item->setData(0, Qt::UserRole, 0);  // 0 = 地理坐标系
        item->setData(1, Qt::UserRole, 0);
        item->setData(0, Qt::UserRole + 1, toqs(pair.second.CoordinateName));
        item->setData(1, Qt::UserRole + 1, toqs(pair.second.CoordinateName));
        _geo->addChild(item);
    }
    
    // 清空投影坐标系节点
    items = _proj->takeChildren();
    for (QTreeWidgetItem *item : items) {
        delete item;
    }
    items.clear();
    
    // 添加投影坐标系
    for (auto& pair : _projCoordinate) {
        QTreeWidgetItem *item = new QTreeWidgetItem();
        item->setText(0, toqs(pair.second.CoordinateName));
        item->setText(1, toqs(pair.second.EPSGName));
        item->setData(0, Qt::UserRole, 1);  // 1 = 投影坐标系
        item->setData(1, Qt::UserRole, 1);
        item->setData(0, Qt::UserRole + 1, toqs(pair.second.CoordinateName));
        item->setData(1, Qt::UserRole + 1, toqs(pair.second.CoordinateName));
        _proj->addChild(item);
    }
    
    // 添加收藏坐标系
    QSettings settings;
    QStringList names = settings.value("favoriteCoordinates", QStringList()).toStringList();
    items = _favorite->takeChildren();
    for (QTreeWidgetItem *item : items) {
        delete item;
    }
    items.clear();
    
    for (QString &s : names) {
        auto it = _geoCoordinate.find(tos(s));
        if (it != _geoCoordinate.end()) {
            QTreeWidgetItem *item = new QTreeWidgetItem();
            item->setText(0, toqs(it->second.CoordinateName));
            item->setText(1, toqs(it->second.EPSGName));
            item->setData(0, Qt::UserRole, 0);
            item->setData(1, Qt::UserRole, 0);
            item->setData(0, Qt::UserRole + 1, toqs(it->second.CoordinateName));
            item->setData(1, Qt::UserRole + 1, toqs(it->second.CoordinateName));
            _favorite->addChild(item);
        } else {
            it = _projCoordinate.find(tos(s));
            if (it != _projCoordinate.end()) {
                QTreeWidgetItem *item = new QTreeWidgetItem();
                item->setText(0, toqs(it->second.CoordinateName));
                item->setText(1, toqs(it->second.EPSGName));
                item->setData(0, Qt::UserRole, 1);
                item->setData(1, Qt::UserRole, 1);
                item->setData(0, Qt::UserRole + 1, toqs(it->second.CoordinateName));
                item->setData(1, Qt::UserRole + 1, toqs(it->second.CoordinateName));
                _favorite->addChild(item);
            }
        }
    }
}

void SpatialReferenceDialog::showAll()
{
    // 使用 UISystemConfig 获取所有坐标系
    UISystemConfig& config = UISystemConfig::instance();
    
    _geoCoordinate.clear();
    _projCoordinate.clear();
    
    const auto& geoCoords = config.getGeoCoordinates();
    for (const auto& coord : geoCoords) {
        _geoCoordinate[coord.CoordinateName] = coord;
    }
    
    const auto& projCoords = config.getProjCoordinates();
    for (const auto& coord : projCoords) {
        _projCoordinate[coord.CoordinateName] = coord;
    }
}

void SpatialReferenceDialog::showSome(const QString &name)
{
    // 使用 UISystemConfig 过滤坐标系
    UISystemConfig& config = UISystemConfig::instance();
    
    _geoCoordinate.clear();
    _projCoordinate.clear();
    
    const auto& geoCoords = config.getGeoCoordinates();
    for (const auto& coord : geoCoords) {
        // 按坐标系名称搜索
        bool matchName = toqs(coord.CoordinateName).contains(name, Qt::CaseInsensitive);
        // 按EPSG代码搜索（支持 "EPSG:4326" 或 "4326" 的格式）
        bool matchEPSG = toqs(coord.EPSGName).contains(name, Qt::CaseInsensitive);
        
        if (matchName || matchEPSG) {
            _geoCoordinate[coord.CoordinateName] = coord;
        }
    }
    
    const auto& projCoords = config.getProjCoordinates();
    for (const auto& coord : projCoords) {
        // 按坐标系名称搜索
        bool matchName = toqs(coord.CoordinateName).contains(name, Qt::CaseInsensitive);
        // 按EPSG代码搜索（支持 "EPSG:4326" 或 "4326" 的格式）
        bool matchEPSG = toqs(coord.EPSGName).contains(name, Qt::CaseInsensitive);
        
        if (matchName || matchEPSG) {
            _projCoordinate[coord.CoordinateName] = coord;
        }
    }
}

void SpatialReferenceDialog::onClearFavorite()
{
    QSettings settings;
    settings.setValue("favoriteCoordinates", QStringList());
    showTrees();
}

}  // namespace ui
}  // namespace insight
