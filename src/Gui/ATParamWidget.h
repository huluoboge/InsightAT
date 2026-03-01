#ifndef ATPARAMWIDGET_H
#define ATPARAMWIDGET_H

#include <QWidget>
#include "InsightATGlobal.h"

#include "ui_ATParamWidget.h"

namespace insight{

struct ATTask;
class ATParamWidget : public QWidget
{
	Q_OBJECT

public:
	ATParamWidget(QWidget *parent = 0);
	~ATParamWidget();
	void init();
	void saveData();

    void setTask(const std::string &task){_taskId = task;}
	ATTask *getTask();
private:
	Ui::ATParamWidget ui;
//	int _mode = 1;
    std::string _taskId;
};
}//name space insight
#endif // ATPARAMWIDGET_H
