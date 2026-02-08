#ifndef CHECKIMAGEWIDGET_H
#define CHECKIMAGEWIDGET_H

#include <QWidget>
#include "ui_CheckImageWidget.h"
#include "Common/Project.h"

class CheckImageWidget : public QWidget
{
	Q_OBJECT

public:
	CheckImageWidget(QWidget *parent = 0);
	~CheckImageWidget();

	void setConsistency(const std::vector<insight::ImageConsistency> &consistency){ _consistency = consistency; }
	void refreshDatas();
private:
	Ui::CheckImageWidget ui;
	std::vector<insight::ImageConsistency> _consistency;
};

#endif // CHECKIMAGEWIDGET_H
