#ifndef UI_WIDGETS_SPATIALREFERENCEDIALOG_H
#define UI_WIDGETS_SPATIALREFERENCEDIALOG_H

#include <QDialog>
#include "ui_SpatialReferenceTool.h"
#include "../../Common/Coordinates.h"

namespace insight {
namespace ui {

class SpatialReferenceDialog : public QDialog
{
    Q_OBJECT

public:
    SpatialReferenceDialog(QWidget *parent = nullptr);
    ~SpatialReferenceDialog();

public:
    Coordinate SelectCoordinate() const;

private slots:
    void onItemClicked(const QModelIndex &index);
    void onAddFavorite();
    void onFilter();
    void onClearFilter();
    void onClearFavorite();

private:
    Ui::SpatialReferenceTool ui;
    void init();
    void showTrees();
    void showAll();
    void showSome(const QString &name);
    
    QTreeWidgetItem *_favorite;
    QTreeWidgetItem *_geo;
    QTreeWidgetItem *_proj;
    Coordinate _curCoordinate;
    std::map<std::string, Coordinate> _geoCoordinate;
    std::map<std::string, Coordinate> _projCoordinate;
};

}  // namespace ui
}  // namespace insight

#endif  // UI_WIDGETS_SPATIALREFERENCEDIALOG_H
