#include "ui_ATConfigWidget.h"

#include "AT3dRenderWidget.h"
#include "ATConfigWidget.h"
#include "ATParamWidget.h"
#include "AdvanceAT.h"
#include "Document.h"
#include <QDebug>
#include <QDialogButtonBox>
#include <QMessageBox>
#include <QProcess>
#include <QSizePolicy>

#include "Common/Project.h"
// #include "Sfm/sfm.h"

namespace insight{

ATConfigWidget::ATConfigWidget(QWidget* parent)
    : SubWidget(parent)
    , ui(new Ui::ATConfigWidget)
{
    ui->setupUi(this);
    _originEditWidget = ui->widget_originImages;
    _refinedEditWidget = ui->widget_refinedImages;
    _renderWidget = ui->widget3d;
    _request = new EngineRequest(this);

    connect(_request, SIGNAL(response(const ResponseHead&)),
        this,
        SLOT(on_response(const ResponseHead&)));
    connect(this, SIGNAL(refreshRefine()), this, SLOT(on_refreshRefined()));
    ui->plainTextEdit->setCenterOnScroll(true);
    ui->plainTextEdit->setReadOnly(true);
    ui->plainTextEdit->setTextInteractionFlags(Qt::TextBrowserInteraction);

    ui->checkBox_featureDetect->setVisible(false);
    ui->checkBox_matching->setVisible(false);
    ui->checkBox_at->setVisible(false);
}

ATConfigWidget::~ATConfigWidget()
{
    SingleEngine& engine = SingleEngine::instance();
    //unbind message first;
    unbindMessage(engine);
    delete ui;
}

void ATConfigWidget::setTask(const std::string& taskId)
{
    _taskId = taskId;
    _originEditWidget->setShowTask(taskId, true);
    _refinedEditWidget->setShowTask(taskId, false);
    ui->lineEdit_projectID->setText(toqs(_taskId));
}

std::string ATConfigWidget::task() const
{
    return _taskId;
}

ATTask* ATConfigWidget::getTask()
{
    for (auto& t : project().atTaskList) {
        if (t.id == _taskId) {
            return &t;
        }
    }
    return nullptr;
}

void ATConfigWidget::init()
{
    unbindMessage(SingleEngine::instance());
    Task* t = getTask();
    if (t && t->id == SingleEngine::instance().currentTaskName()) {
        bindMessage(SingleEngine::instance());
    }
    refreshDatas();
}

bool ATConfigWidget::checkStart()
{
    ATTask* task = getTask();
    SingleEngine& engine = SingleEngine::instance();
    if (engine.isRunning() || task == nullptr) {
        QMessageBox::information(this, tr("Warnning"), tr("Processing is running..."));
        return false;
    }
    return true;
}

void ATConfigWidget::refreshDatas()
{
    _originEditWidget->disableCameraEdit();
    _originEditWidget->refreshDatas();
    _refinedEditWidget->refreshDatas();
    ATTask* task = getTask();
    if (task == nullptr) {
        return;
    }
    _renderWidget->refreshDatas(*task);

    ui->lineEdit_projectID->setText(toqs(task->id));
    //query task status
    queryTaskStatus();
}

void ATConfigWidget::queryTaskStatus()
{
    ATTask* task = getTask();

    if (task == nullptr)
        return;

    if (task->info.ATStatus == ATTask::eFinished) {
        ui->groupBox_controlEdit->setEnabled(true);
        ui->groupBox_export->setEnabled(true);
    } else {
        ui->groupBox_controlEdit->setEnabled(false);
        ui->groupBox_export->setEnabled(false);

#if 0
        if (task->info.ATStatus == ATTask::eCommitted)
        {
            EngineRequest *request = new EngineRequest(this);
            QObject::connect(request, &EngineRequest::response,
                             [=](const ResponseHead &head) {
                                 if (head.result != 0)
                                 {
                                     LOG(INFO) << head.errorMsg;
                                 }
                                 else
                                 {
                                     switch (head.queryStatus)
                                     {
                                     case eCreated:
                                         LOG(INFO) << "Task is Created";
                                         break;
                                     case eAppending:
                                         LOG(INFO) << "Task is Appending";
                                         break;
                                     case eRunning:
                                         LOG(INFO) << "Task is Running";
                                         break;
                                     case eFinished:
                                         LOG(INFO) << "Task is Finished";
                                         break;
                                     case eCanceled:
                                         LOG(INFO) << "Task is Canceled";
                                         break;
                                     }
                                 }
                                 request->deleteLater();
                             });

            RequestHead req;
            sprintf(req.name, "%s", task->id.c_str());
            sprintf(req.taskPath, "%s", task->taskDir.c_str());
            req.dataBytes = 0;
            req.commandType = eQuery;
            req.queryType = eQueryStatus;
            request->send(req);
        }
#endif
    }
}

void ATConfigWidget::on_pushButton_AT_clicked()
{
    if (!checkStart())
        return;
    qDebug() << "on_pushButton_AT_clicked";
    ATTask* task = getTask();
    if (task == nullptr)
        return;

    SingleEngine& engine = SingleEngine::instance();
    if (engine.isRunning())
        return;

    bindMessage(engine);
    EngineTaskATPtr engineTask(new EngineTaskAT);
    engineTask->type = eAT;
    engineTask->name = task->id;
    engineTask->folder = task->taskDir;
    engineTask->doFeat = ui->checkBox_featureDetect->isChecked();
    engineTask->doMatch = ui->checkBox_matching->isChecked();
    engineTask->doAT = ui->checkBox_at->isChecked();
    engine.startTask(std::dynamic_pointer_cast<EngineTask>(engineTask));
    setButtonEnableState(false);
    auto conn = std::make_shared<QMetaObject::Connection>();
    *conn = QObject::connect(&engine, &SingleEngine::finished, [this, conn](int exitCode) {
        SingleEngine::instance().flush();
        QObject::disconnect(*conn);
        QByteArray msg = QString("Exit process code=%1").arg(exitCode).toLatin1();
        this->on_showMessage(msg);
        this->unbindMessage(SingleEngine::instance());
        setButtonEnableState(true);
        if (exitCode == EXIT_SUCCESS) {
            ATTask* t = getTask();
            if (t) {
                t->readInfos();
                t->readRefined();
                t->readOriginMapCoord();
                this->refreshRefine();
                queryTaskStatus();
            }
        }
    });
}

void ATConfigWidget::on_pushButton_checkAT_clicked()
{
    qDebug() << "on_pushButton_checkAT_clicked";
    if (!checkStart())
        return;

    ATTask* task = getTask();
    if (task == nullptr)
        return;

    SingleEngine& engine = SingleEngine::instance();
    if (engine.isRunning())
        return;
    bindMessage(engine);
    EngineTaskPtr engineTask(new EngineTask);
    engineTask->type = eCheckAT;
    engineTask->name = task->id;
    engineTask->folder = task->taskDir;

    engine.startTask(engineTask);
    setButtonEnableState(false);
    auto conn = std::make_shared<QMetaObject::Connection>();
    *conn = QObject::connect(&engine, &SingleEngine::finished, [this, conn](int exitCode) {
        SingleEngine::instance().flush();
        QObject::disconnect(*conn);
        QByteArray msg = QString("Exit process code=%1").arg(exitCode).toLatin1();
        this->on_showMessage(msg);
        this->unbindMessage(SingleEngine::instance());
        // _renderWidget->setEnabled(true);
        // _refinedEditWidget->setEnabled(true);
        // _originEditWidget->setEnabled(true);
        // ui->groupBox_AT->setEnabled(true);
        // ui->pushButton_stop->setEnabled(false);
        setButtonEnableState(true);
        if (exitCode == EXIT_SUCCESS) {
            ATTask* t = getTask();
            if (t) {
                t->readRefined();
                t->readOriginMapCoord();
                this->refreshRefine();
                queryTaskStatus();
            }
        }
    });
}

void ATConfigWidget::on_pushButton_paramSetting_clicked()
{
    ATTask* task = getTask();
    if (task == nullptr)
        return;
    QDialog dlg(this);
    QVBoxLayout* layout = new QVBoxLayout;
    ATParamWidget* widget = new ATParamWidget;
    widget->setTask(task->id);
    widget->init();
    layout->addWidget(widget);
    dlg.resize(dlg.sizeHint());
    dlg.setLayout(layout);
    dlg.exec();
    widget->saveData();
    task->writeDatas();
}

void ATConfigWidget::on_pushButton_gcpBA_clicked()
{
    qDebug() << "on_pushButton_gcpBA_clicked";
    if (!checkStart())
        return;

    ATTask* task = getTask();
    if (task == nullptr)
        return;
    SingleEngine& engine = SingleEngine::instance();
    if (engine.isRunning())
        return;
    bindMessage(engine);
    EngineTaskPtr engineTask(new EngineTask);
    engineTask->type = eGCPBA;
    engineTask->name = task->id;
    engineTask->folder = task->taskDir;

    engine.startTask(engineTask);
    setButtonEnableState(false);
    auto conn = std::make_shared<QMetaObject::Connection>();
    *conn = QObject::connect(&engine, &SingleEngine::finished, [this, conn](int exitCode) {
        SingleEngine::instance().flush();
        QObject::disconnect(*conn);
        QByteArray msg = QString("Exit process code=%1").arg(exitCode).toLatin1();
        this->on_showMessage(msg);
        this->unbindMessage(SingleEngine::instance());
        setButtonEnableState(true);
        if (exitCode == EXIT_SUCCESS) {
            ATTask* t = getTask();
            if (t) {
                t->readRefined();
                this->refreshRefine();
                queryTaskStatus();
            }
        }
    });
}

void ATConfigWidget::on_pushButton_refineBA_clicked()
{
    qDebug() << "on_pushButton_refineBA_clicked";
    if (!checkStart())
        return;

    ATTask* task = getTask();
    if (task == nullptr)
        return;
    SingleEngine& engine = SingleEngine::instance();
    if (engine.isRunning())
        return;
    bindMessage(engine);
    EngineTaskPtr engineTask(new EngineTask);
    engineTask->type = eRefineBA;
    engineTask->name = task->id;
    engineTask->folder = task->taskDir;

    engine.startTask(engineTask);
    setButtonEnableState(false);
    auto conn = std::make_shared<QMetaObject::Connection>();
    *conn = QObject::connect(&engine, &SingleEngine::finished, [this, conn](int exitCode) {
        SingleEngine::instance().flush();
        QObject::disconnect(*conn);
        QByteArray msg = QString("Exit process code=%1").arg(exitCode).toLatin1();
        this->on_showMessage(msg);
        this->unbindMessage(SingleEngine::instance());
        setButtonEnableState(true);
        if (exitCode == EXIT_SUCCESS) {
            ATTask* t = getTask();
            if (t) {
                t->readRefined();
                this->refreshRefine();
                queryTaskStatus();
            }
        }
    });
}

void ATConfigWidget::on_pushButton_retriangleBA_clicked()
{
    qDebug() << "on_pushButton_retriangleBA_clicked";
    if (!checkStart())
        return;

    ATTask* task = getTask();
    if (task == nullptr)
        return;
    SingleEngine& engine = SingleEngine::instance();
    if (engine.isRunning())
        return;
    bindMessage(engine);
    EngineTaskPtr engineTask(new EngineTask);
    engineTask->type = eRetirangleBA;
    engineTask->name = task->id;
    engineTask->folder = task->taskDir;

    engine.startTask(engineTask);
    setButtonEnableState(false);
    auto conn = std::make_shared<QMetaObject::Connection>();
    *conn = QObject::connect(&engine, &SingleEngine::finished, [this, conn](int exitCode) {
        SingleEngine::instance().flush();
        QObject::disconnect(*conn);
        QByteArray msg = QString("Exit process code=%1").arg(exitCode).toLatin1();
        this->on_showMessage(msg);
        this->unbindMessage(SingleEngine::instance());
        setButtonEnableState(true);
        if (exitCode == EXIT_SUCCESS) {
            ATTask* t = getTask();
            if (t) {
                t->readRefined();
                this->refreshRefine();
                queryTaskStatus();
            }
        }
    });
}

void ATConfigWidget::on_pushButton_exportCC_clicked()
{
    qDebug() << "on_pushButton_exportCC_clicked";
    if (!checkStart())
        return;

    ATTask* task = getTask();
    if (task == nullptr)
        return;

    SingleEngine& engine = SingleEngine::instance();
    if (engine.isRunning())
        return;
    bindMessage(engine);
    EngineTaskPtr engineTask(new EngineTask);
    engineTask->type = eExportCC;
    engineTask->name = task->id;
    engineTask->folder = task->taskDir;

    engine.startTask(engineTask);
    setButtonEnableState(false);
    auto conn = std::make_shared<QMetaObject::Connection>();
    *conn = QObject::connect(&engine, &SingleEngine::finished, [this, conn](int exitCode) {
        SingleEngine::instance().flush();
        QObject::disconnect(*conn);
        QByteArray msg = QString("Exit process code=%1").arg(exitCode).toLatin1();
        this->on_showMessage(msg);
        this->unbindMessage(SingleEngine::instance());
        setButtonEnableState(true);
        if (exitCode == EXIT_SUCCESS) {
            ATTask* t = getTask();
            if (t) {
                t->readRefined();
                this->refreshRefine();
            }
        }
    });
}

void ATConfigWidget::on_pushButton_gcpEdit_clicked()
{
    if (!checkStart())
        return;

    ATTask* task = getTask();
    if (task == nullptr)
        return;

#ifdef WIN32
    QString program = qApp->applicationDirPath() + "/ControlEdit.exe";
#else
    QString program = qApp->applicationDirPath() + "/ControlEdit";
#endif
    QStringList args;
    args << toqs(task->taskDir)
         << "0"; //disable open action
    qDebug() << args;
    QProcess::execute(program, args);
    //refresh result
    ATTask* t = getTask();
    if (t) {
        t->refreshGCPList();
        _renderWidget->refreshDatas(*t);
    }
}

void ATConfigWidget::on_pushButton_more_clicked()
{
    ui->checkBox_featureDetect->setVisible(!ui->checkBox_at->isVisible());
    ui->checkBox_matching->setVisible(!ui->checkBox_at->isVisible());
    ui->checkBox_at->setVisible(!ui->checkBox_at->isVisible());
    return;

    if (!checkStart())
        return;

    ATTask* task = getTask();
    if (task == nullptr)
        return;
    QDialog dlg(this);
    QVBoxLayout* layout = new QVBoxLayout;
    AdvanceAT* widget = new AdvanceAT;
    // widget->setTask(task->id);
    // widget->init();
    layout->addWidget(widget);
    dlg.resize(dlg.sizeHint());
    dlg.setLayout(layout);
    dlg.exec();
    // widget->saveData();
    // task->writeDatas();
}
void ATConfigWidget::on_pushButton_stop_clicked()
{
    ATTask* task = getTask();
    if (!task)
        return;
    SingleEngine& engine = SingleEngine::instance();
    if (engine.isRunning() && engine.currentTaskName() == task->id)
        engine.stop();
}

void ATConfigWidget::on_response(const ResponseHead& head)
{
    if (head.result != 0) {
        LOG(INFO) << head.errorMsg;
    }
}
void ATConfigWidget::on_showMessage(const QByteArray& msg)
{
    ui->plainTextEdit->appendPlainText(QString::fromLocal8Bit(msg));
}

void ATConfigWidget::bindMessage(SingleEngine& engine)
{
    connect(&engine, SIGNAL(message(const QByteArray&)),
        this, SLOT(on_showMessage(const QByteArray&)));
}

void ATConfigWidget::unbindMessage(SingleEngine& engine)
{
    disconnect(&engine, SIGNAL(message(const QByteArray&)),
        this, SLOT(on_showMessage(const QByteArray&)));
}

void ATConfigWidget::setButtonEnableState(bool enable)
{
    _renderWidget->setEnabled(enable);
    _refinedEditWidget->setEnabled(enable);
    _originEditWidget->setEnabled(enable);
    ui->groupBox_AT->setEnabled(enable);
    ui->groupBox_controlEdit->setEnabled(enable);
    ui->groupBox_export->setEnabled(enable);
    ui->pushButton_stop->setEnabled(!enable);
}

void ATConfigWidget::on_refreshRefined()
{
    qDebug() << "on_refreshRefined";
    _refinedEditWidget->refreshDatas();
    ATTask* task = getTask();
    if (task == nullptr) {
        return;
    }
    _renderWidget->refreshDatas(*task);
    qDebug() << "on_refreshRefined_END";
}

void ATConfigWidget::on_finished(int exitCode)
{
    QByteArray msg = QString("Exit process code=%1").arg(exitCode).toLatin1();
    this->on_showMessage(msg);
}

}//name space insight
