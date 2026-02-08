#ifndef GCPWIDGET_H
#define GCPWIDGET_H

#include <QWidget>
#include "ui_gcpwidget.h"
#include "SubWidget.h"

namespace insight{
class GCPWidget : public  SubWidget
{
	Q_OBJECT

	signals:
	void gcpStartEditing(int gcp_id);
	void gcpCleared();
public:
	GCPWidget(QWidget *parent = 0);
	~GCPWidget();
	virtual void init(){ refreshDatas(); }
	virtual void refreshDatas();
    virtual void enable(){}
    virtual void disable(){}

	void enableEdit(bool val){ _edit = val; }
public slots:
	void onImport();
	void onAdd();
	void onClear();
	void onDelete();
	void saveDatas();//save from widget to project
private:
	Ui::GCPWidget ui;
	bool _refreshing = false;
	bool _edit = false;
};

}//name space insight
#endif // GCPWIDGET_H
