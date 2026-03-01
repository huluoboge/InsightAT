#include "ProjectInfoWidget.h"
#include <QFileInfo>
#include <QDir>
#include <QMessageBox>
#include <QFileDialog>
#include "Settings.h"
#include "Document.h"

namespace insight{
ProjectInfoWidget::ProjectInfoWidget(QWidget *parent)
	: SubWidget(parent)
{
	ui.setupUi(this);
	//QObject::connect(ui.lineEdit_projectName, SIGNAL(textChanged(QString)), this, SLOT(valid()));
}

ProjectInfoWidget::~ProjectInfoWidget()
{

}

void ProjectInfoWidget::init()
{
	refreshDatas();
}

void ProjectInfoWidget::refreshDatas()
{
	if (doc().isOpen())
	{
		ui.lineEdit_projectName->setText(toqs(project().infomation.name));
		ui.lineEdit_operator->setText(toqs(project().infomation.author));
		QDateTime dt = QDateTime::fromString(toqs(project().infomation.date), "yyyy/MM/dd hh:mm:ss");
		ui.dateTimeEdit_date->setDateTime(dt);
		ui.plainTextEdit_description->setPlainText(toqs(project().infomation.description));
	}
}

QString ProjectInfoWidget::name() const
{
	return ui.lineEdit_projectName->text();
}

QDateTime ProjectInfoWidget::date() const
{
	return ui.dateTimeEdit_date->dateTime();
}

QString ProjectInfoWidget::author() const
{
	return ui.lineEdit_operator->text();
}

QString ProjectInfoWidget::description() const
{
	return ui.plainTextEdit_description->toPlainText();
}

}//name space insight
