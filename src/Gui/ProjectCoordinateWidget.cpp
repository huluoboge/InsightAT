#include "ProjectCoordinateWidget.h"

#include "Document.h"
#include "SpatialReferenceTool.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QListWidget>

namespace insight{
ProjectCoordinateWidget::ProjectCoordinateWidget(QWidget *parent)
	: SubWidget(parent)
{
	ui.setupUi(this);
    connect(ui.radioButton_5, &QRadioButton::toggled, [this](bool isChecked){
		if (isRefresh) return;
        ui.lineEdit_gpsCoordName->setEnabled(!isChecked);
        ui.lineEdit_gpsCoordEPSG->setEnabled(!isChecked);
        project().infomation.gpsCoordinate.localSystem = isChecked;
	});
    connect(ui.radioButton, &QRadioButton::toggled, [this](bool isChecked){
        if (isRefresh) return;
        ui.lineEdit_coord_name->setEnabled(!isChecked);
        ui.lineEdit_coordEPSG->setEnabled(!isChecked);
        project().infomation.coordinate.localSystem = isChecked;
    });
}

ProjectCoordinateWidget::~ProjectCoordinateWidget()
{

}

void ProjectCoordinateWidget::init()
{
	refreshDatas();
}

void ProjectCoordinateWidget::refreshDatas()
{
	isRefresh = true;
	if (project().infomation.relativeFlightAltitude != ProjectInfomation::UNKNOWN_ALTITUTE)
	{
		ui.doubleSpinBox_flyingHeight->setValue(project().infomation.relativeFlightAltitude);
	}
	if (project().infomation.averageElevationOfGround != ProjectInfomation::UNKNOWN_ALTITUTE)
	{
		ui.doubleSpinBox_averageElevation->setValue(project().infomation.averageElevationOfGround);
	}
    ui.radioButton_5->setChecked(project().infomation.gpsCoordinate.localSystem);
    ui.radioButton_6->setChecked(!project().infomation.gpsCoordinate.localSystem);
    ui.radioButton->setChecked(project().infomation.coordinate.localSystem);
    ui.radioButton_2->setChecked(!project().infomation.coordinate.localSystem);
	ui.lineEdit_gpsCoordName->setText(toqs(project().infomation.gpsCoordinate.name));
	ui.lineEdit_gpsCoordEPSG->setText(toqs(project().infomation.gpsCoordinate.epsg));

	ui.lineEdit_coord_name->setText(toqs(project().infomation.coordinate.name));
	ui.lineEdit_coordEPSG->setText(toqs(project().infomation.coordinate.epsg));

	isRefresh = false;
}

void ProjectCoordinateWidget::on_pushButton_gps_clicked()
{
    SpatialReferenceTool tool;
	if (QDialog::Accepted == tool.exec())
	{
		Coordinate coord = tool.SelectCoordinate();
		ui.lineEdit_gpsCoordName->setText(toqs(coord.CoordinateName));
		ui.lineEdit_gpsCoordEPSG->setText(toqs(coord.EPSGName));
		project().infomation.gpsCoordinate.epsg = coord.EPSGName;
		project().infomation.gpsCoordinate.wkt = coord.WKT;
		project().infomation.gpsCoordinate.name = coord.CoordinateName;
		refreshDatas();
	}
}

void ProjectCoordinateWidget::on_pushButton_clicked()
{
    SpatialReferenceTool tool;
	if (QDialog::Accepted == tool.exec())
	{
		Coordinate coord = tool.SelectCoordinate();
		ui.lineEdit_coord_name->setText(toqs(coord.CoordinateName));
		ui.lineEdit_coordEPSG->setText(toqs(coord.EPSGName));
		project().infomation.coordinate.epsg = coord.EPSGName;
		project().infomation.coordinate.wkt = coord.WKT;
		project().infomation.coordinate.name = coord.CoordinateName;
		refreshDatas();
	}
}

insight::CoordInfomation ProjectCoordinateWidget::mappingCoord()
{
	insight::CoordInfomation coordinate;
	//coordinate.wkt = tos(ui.plainTextEdit_wkt->toPlainText());
	coordinate.epsg = tos(ui.lineEdit_coordEPSG->text());
	coordinate.name = tos(ui.lineEdit_coord_name->text());
//	coordinate.localSystem = ui.checkBox_localSystem->isChecked();
	//coordinate.ENUCenter = enuCenter();
	return coordinate;
}

insight::CoordInfomation ProjectCoordinateWidget::gpsCoord()
{
    insight::CoordInfomation coordinate;
    //coordinate.wkt = tos(ui.plainTextEdit_wkt->toPlainText());
    coordinate.epsg = tos(ui.lineEdit_gpsCoordEPSG->text());
    coordinate.name = tos(ui.lineEdit_gpsCoordName->text());
//	coordinate.localSystem = ui.checkBox_localSystem->isChecked();
    //coordinate.ENUCenter = enuCenter();
    return coordinate;
}
float ProjectCoordinateWidget::flyingHeight()
{
	return ui.doubleSpinBox_flyingHeight->value();
}

float ProjectCoordinateWidget::averageElevation()
{
	return ui.doubleSpinBox_averageElevation->value();
}




}//name space insight

