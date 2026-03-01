#ifndef ATCONFIGWIDGET_H
#define ATCONFIGWIDGET_H

#include <QWidget>
#include "SubWidget.h"
#include <cstdint>

#include "InsightATGlobal.h"
#include "ProjectEditWidget.h"
#include "AT3dRenderWidget.h"

#include "Common/db_types.h"
#include "Common/Project.h"
#include "Document.h"

namespace Ui {
class ATConfigWidget;
}

namespace insight{

class EngineRequest;
class ResponseHead;

class ATConfigWidget : public SubWidget
{
    Q_OBJECT
signals:
    void refreshRefine();
public:
    explicit ATConfigWidget(QWidget *parent = nullptr);
    ~ATConfigWidget();
    void setTask(const std::string &task);
    std::string task() const;
    void init();
    void refreshDatas();
    void enable(){}
    void disable(){}

public slots:
    void on_pushButton_AT_clicked();
	void on_pushButton_paramSetting_clicked();
    void on_pushButton_gcpBA_clicked();
    void on_pushButton_exportCC_clicked();
	void on_pushButton_gcpEdit_clicked();
    void on_pushButton_stop_clicked();
    void on_pushButton_checkAT_clicked();
    void on_pushButton_more_clicked();
    void on_pushButton_refineBA_clicked();
    void on_pushButton_retriangleBA_clicked();
    void on_response(const ResponseHead& );
    void on_showMessage(const QByteArray &);
    void on_refreshRefined();
    void on_finished(int exitCode);
private:
    void bindMessage(SingleEngine &engine);
    void unbindMessage(SingleEngine &engine);
    void setButtonEnableState(bool enable);
    bool checkStart();
    ATTask *getTask();
    void queryTaskStatus();
    Ui::ATConfigWidget *ui;
    ProjectEditWidget *_originEditWidget;
    ProjectEditWidget *_refinedEditWidget;
    AT3dRenderWidget *_renderWidget;
    std::string _taskId;
    EngineRequest *_request;
    int _curTaskType = 0;
};

}//name space insight
#endif // ATCONFIGWIDGET_H
