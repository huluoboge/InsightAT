#include "SpatialReferenceTool.h"
#include "Settings.h"
#include <QDebug>
#include "Common/Project.h"
#include "Document.h"
#include "Common/string_utils.h"


namespace insight{
SpatialReferenceTool::SpatialReferenceTool(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	init();
}

SpatialReferenceTool::~SpatialReferenceTool()
{

}

void SpatialReferenceTool::init()
{
	//1 favorite
	//treeWidget_XYCoord
	QTreeWidgetItem *root = ui.treeWidget_XYCoord->invisibleRootItem();

	_favorite = ui.treeWidget_XYCoord->topLevelItem(2);
	_geo = ui.treeWidget_XYCoord->topLevelItem(1);
	_proj = ui.treeWidget_XYCoord->topLevelItem(0);
	ui.treeWidget_XYCoord->sortByColumn(0,Qt::AscendingOrder);
	ui.buttonBox->button(QDialogButtonBox::Ok)->setEnabled(false);
	showAll();
	showTrees();
}

void SpatialReferenceTool::onItemClicked(const QModelIndex &index)
{
	if (!index.isValid()) return;
	if (!index.data(Qt::UserRole).isValid()) return;
	int type = index.data(Qt::UserRole).toInt();
	QString CoordinateName = index.data(Qt::UserRole + 1).toString();
	if (1 == type)
	{
		_curCoordinate = _projCoordinate.at(tos(CoordinateName));
		ui.plainTextEdit_curCoord->setPlainText(toqs(_curCoordinate.WKT));
		ui.buttonBox->button(QDialogButtonBox::Ok)->setEnabled(true);
	}
	else if (0 == type)
	{
		_curCoordinate = _geoCoordinate.at(tos(CoordinateName));
		ui.plainTextEdit_curCoord->setPlainText(toqs(_curCoordinate.WKT));
		ui.buttonBox->button(QDialogButtonBox::Ok)->setEnabled(true);
	}
	else
	{
		ui.buttonBox->button(QDialogButtonBox::Ok)->setEnabled(false);
	}
}

Coordinate SpatialReferenceTool::SelectCoordinate() const
{
	return _curCoordinate;
}

void SpatialReferenceTool::onAddFavorite()
{
	QStringList names = settings().favoriteCoordinates();
	if (ui.buttonBox->button(QDialogButtonBox::Ok)->isEnabled())
	{
		names.push_back(toqs(_curCoordinate.CoordinateName));
		settings().setFavoriteCoordinate(names);
		showTrees();
	}
}

void SpatialReferenceTool::onFilter()
{
	QString nm = ui.lineEdit_filter->text();
	if (nm.isEmpty())
	{
		showAll();
	}
	else
	{
		showSome(nm);
	}
	showTrees();
}

void SpatialReferenceTool::onClearFilter()
{
	ui.lineEdit_filter->clear();
}

void SpatialReferenceTool::showTrees()
{
	QList<QTreeWidgetItem *> items = _geo->takeChildren();
	for (QTreeWidgetItem *item : items)
	{
		delete item;
	}
	items.clear();
	for (std::map<std::string, Coordinate>::iterator itr = _geoCoordinate.begin();
		itr != _geoCoordinate.end(); ++itr)
	{
		QTreeWidgetItem *item = new QTreeWidgetItem();
		item->setText(0, toqs(itr->second.CoordinateName));
		item->setText(1, toqs(itr->second.EPSGName));
		item->setData(0, Qt::UserRole, 0);
		item->setData(1, Qt::UserRole, 0);
		item->setData(0, Qt::UserRole + 1, toqs(itr->second.CoordinateName));
		item->setData(1, Qt::UserRole + 1, toqs(itr->second.CoordinateName));
		_geo->addChild(item);
	}
	items = _proj->takeChildren();
	for (QTreeWidgetItem *item : items)
	{
		delete item;
	}
	items.clear();
	for (std::map<std::string, Coordinate>::iterator itr =_projCoordinate.begin();
		itr != _projCoordinate.end(); ++itr)
	{
		QTreeWidgetItem *item = new QTreeWidgetItem();
		item->setText(0, toqs(itr->second.CoordinateName));
		item->setText(1, toqs(itr->second.EPSGName));
		item->setData(0, Qt::UserRole, 1);
		item->setData(1, Qt::UserRole, 1);
		item->setData(0, Qt::UserRole + 1, toqs(itr->second.CoordinateName));
		item->setData(1, Qt::UserRole + 1, toqs(itr->second.CoordinateName));
		_proj->addChild(item);
	}
	
	QStringList names = settings().favoriteCoordinates();
	items = _favorite->takeChildren();
	for (QTreeWidgetItem *item : items)
	{
		delete item;
	}
	items.clear();
	for (QString &s : names)
	{
		std::map<std::string, Coordinate>::const_iterator itr = _geoCoordinate.find(tos(s));
		if (itr != _geoCoordinate.end())
		{
			QTreeWidgetItem *item = new QTreeWidgetItem();
			item->setText(0, toqs(itr->second.CoordinateName));
			item->setText(1, toqs(itr->second.EPSGName));
			item->setData(0, Qt::UserRole, 0);
			item->setData(1, Qt::UserRole, 0);
			item->setData(0, Qt::UserRole + 1, toqs(itr->second.CoordinateName));
			item->setData(1, Qt::UserRole + 1, toqs(itr->second.CoordinateName));
			_favorite->addChild(item);
		}
		else
		{
			itr = _projCoordinate.find(tos(s));
			if (itr != _projCoordinate.end())
			{
				QTreeWidgetItem *item = new QTreeWidgetItem();
				item->setText(0, toqs(itr->second.CoordinateName));
				item->setText(1, toqs(itr->second.EPSGName));
				item->setData(0, Qt::UserRole, 1);
				item->setData(1, Qt::UserRole, 1);
				item->setData(0, Qt::UserRole + 1, toqs(itr->second.CoordinateName));
				item->setData(1, Qt::UserRole + 1, toqs(itr->second.CoordinateName));
				_favorite->addChild(item);
			}
		}
	}
}

void SpatialReferenceTool::showAll()
{
	_geoCoordinate.clear();
// 	for (std::map<int, Coordinate>::iterator itr = SystemConfig::instance().GeoCoordinate.begin();
// 		itr != SystemConfig::instance().GeoCoordinate.end(); ++itr)
// 	{
	for (int i = 0; i < SystemConfig::instance().GeoCoordinate.size(); ++i)
	{
		_geoCoordinate[SystemConfig::instance().GeoCoordinate[i].CoordinateName] =
			SystemConfig::instance().GeoCoordinate[i];
	}
	_projCoordinate.clear();
// 	for (std::map<int, Coordinate>::iterator itr = SystemConfig::instance().ProjCoordinate.begin();
// 		itr != SystemConfig::instance().ProjCoordinate.end(); ++itr)
	for (int i = 0; i < SystemConfig::instance().ProjCoordinate.size(); ++i)
	{
		_projCoordinate[SystemConfig::instance().ProjCoordinate[i].CoordinateName] = 
			SystemConfig::instance().ProjCoordinate[i];
	}
}



void SpatialReferenceTool::showSome(const QString &name)
{
	_geoCoordinate.clear();
	_projCoordinate.clear();
	for (int i = 0; i < SystemConfig::instance().GeoCoordinate.size(); ++i)
	{
		if (toqs(SystemConfig::instance().GeoCoordinate[i].CoordinateName).contains(name, Qt::CaseInsensitive))
		{
			_geoCoordinate[SystemConfig::instance().GeoCoordinate[i].CoordinateName] =
				SystemConfig::instance().GeoCoordinate[i];
		}

	}

	for (int i = 0; i < SystemConfig::instance().ProjCoordinate.size(); ++i)
	{
		if (toqs(SystemConfig::instance().ProjCoordinate[i].CoordinateName).contains(name, Qt::CaseInsensitive))
		{
			_projCoordinate[SystemConfig::instance().ProjCoordinate[i].CoordinateName] =
				SystemConfig::instance().ProjCoordinate[i];
		}

	}

// 	for (std::map<int, Coordinate>::iterator itr = SystemConfig::instance().GeoCoordinate.begin();
// 		itr != SystemConfig::instance().GeoCoordinate.end(); ++itr)
// 	{
// 		if (toqs(itr->second.CoordinateName).contains(name,Qt::CaseInsensitive))
// 			_geoCoordinate[itr->second.CoordinateName] = itr->second;
// 	}
// 	_projCoordinate.clear();
// 	for (std::map<int, Coordinate>::iterator itr = SystemConfig::instance().ProjCoordinate.begin();
// 		itr != SystemConfig::instance().ProjCoordinate.end(); ++itr)
// 	{
// 		if (toqs(itr->second.CoordinateName).contains(name, Qt::CaseInsensitive))
// 			_projCoordinate[itr->second.CoordinateName] = itr->second;
// 	}
}

void SpatialReferenceTool::onClearFavorite()
{
	settings().setFavoriteCoordinate(QStringList());
	showTrees();
}


}//name space insight
