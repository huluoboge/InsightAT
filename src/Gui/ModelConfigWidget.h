#ifndef MODELCONFIGWIDGET_H
#define MODELCONFIGWIDGET_H

#include "SubWidget.h"
#include <QWidget>
#include <cstdint>

#include "AT3dRenderWidget.h"
#include "InsightATGlobal.h"
#include "ProjectEditWidget.h"

#include "Common/Project.h"
#include "Common/db_types.h"
#include "Document.h"

namespace Ui {
class ModelConfigWidget;
}

namespace insight{

class EngineRequest;
class ResponseHead;

class ModelConfigWidget : public SubWidget {
    Q_OBJECT
    // signals:
public:
    explicit ModelConfigWidget(QWidget* parent = nullptr);
    ~ModelConfigWidget();
    void setTask(const std::string& task);
    std::string task() const;
    void init();
    void refreshDatas();
    void enable() { }
    void disable() { }

public slots:
    void on_pushButton_autoblock_clicked();
    void on_pushButton_Model_clicked();

    void on_pushButton_stop_clicked();
    void on_pushButton_more_clicked();
    void on_response(const ResponseHead&);
    void on_showMessage(const QByteArray&);
    // void on_refreshRefined();
    void on_finished(int exitCode);
    void on_gridCountChanged();

private:
    void bindMessage(SingleEngine& engine);
    void unbindMessage(SingleEngine& engine);
    void setButtonEnableState(bool enable);
    bool checkStart();
    ModelTask* getTask();
    void queryTaskStatus();
    Ui::ModelConfigWidget* ui;
    AT3dRenderWidget* _renderWidget;
    std::string _taskId;
    EngineRequest* _request;
    int _curTaskType = 0;
    bool enableUpdateGrid = true;
    struct SetGridFunction {
        ModelTask* task = nullptr;
        ModelConfigWidget* config = nullptr;
        ModelConfigWidget* p() { return config; }
        void operator()(int xcount, int ycount, int zcount);
    } _gridFunc;

    friend class SetGridFunction;
};

}//name space insight
#endif // MODELCONFIGWIDGET_H
