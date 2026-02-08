#ifndef PROJECTCOORDINATEWIDGET_H
#define PROJECTCOORDINATEWIDGET_H

#include <QWidget>
#include "SubWidget.h"
#include "Common/Project.h"
#include "ui_ProjectCoordinateWidget.h"
#include "Common/numeric.h"

namespace insight{

class ProjectCoordinateWidget : public SubWidget
{
	Q_OBJECT

public:
	ProjectCoordinateWidget(QWidget *parent = 0);
	~ProjectCoordinateWidget();
	void init();
	void refreshDatas();
    void enable(){}
    void disable(){}
	public slots:
	void on_pushButton_gps_clicked();
	void on_pushButton_clicked();

	float flyingHeight();
	float averageElevation();
private slots:
    insight::CoordInfomation mappingCoord();
    insight::CoordInfomation gpsCoord();
private:
	bool isRefresh = false;
	Ui::ProjectCoordinateWidget ui;
};

}//name space insight
#endif // PROJECTCOORDINATEWIDGET_H
