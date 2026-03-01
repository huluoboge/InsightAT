#ifndef WPOSPATIALREFERENCETOOL_H
#define WPOSPATIALREFERENCETOOL_H

#include <QDialog>
#include "ui_SpatialReferenceTool.h"
#include "Common/Coordinates.h"
#include "InsightATGlobal.h"

namespace insight{
class SpatialReferenceTool : public QDialog
{
	Q_OBJECT

public:
	SpatialReferenceTool(QWidget *parent = 0);
	~SpatialReferenceTool();
public:
	insight::Coordinate SelectCoordinate() const;
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

}//name space insight


#endif // WPOSPATIALREFERENCETOOL_H
