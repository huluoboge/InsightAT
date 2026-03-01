#include "ui_ModelConfigWidget.h"

#include "AT3dRenderWidget.h"
#include "Document.h"
#include "ModelConfigWidget.h"
#include <QDebug>
#include <QDialogButtonBox>
#include <QMessageBox>
#include <QProcess>
#include <QSizePolicy>

#include "Common/Project.h"

namespace insight{

ModelConfigWidget::ModelConfigWidget(QWidget* parent)
    : SubWidget(parent)
    , ui(new Ui::ModelConfigWidget)

{
    _gridFunc.config = this;
    ui->setupUi(this);
    _renderWidget = ui->widget3d;
    _request = new EngineRequest(this);

    connect(_request, SIGNAL(response(const ResponseHead&)),
        this,
        SLOT(on_response(const ResponseHead&)));

    connect(ui->spinBox_xcount, SIGNAL(valueChanged(int)), this, SLOT(on_gridCountChanged()));
    connect(ui->spinBox_ycount, SIGNAL(valueChanged(int)), this, SLOT(on_gridCountChanged()));
    connect(ui->spinBox_zcount, SIGNAL(valueChanged(int)), this, SLOT(on_gridCountChanged()));
    connect(ui->radioButton2d, SIGNAL(toggled(bool)), this, SLOT(on_gridCountChanged()));
    // connect(this, SIGNAL(refreshRefine()), this, SLOT(on_refreshRefined()));
    // ui->plainTextEdit->setCenterOnScroll(true);
    // ui->plainTextEdit->setReadOnly(true);
    // ui->plainTextEdit->setTextInteractionFlags(Qt::TextBrowserInteraction);

    // ui->checkBox_featureDetect->setVisible(false);
    // ui->checkBox_matching->setVisible(false);
    // ui->checkBox_at->setVisible(false);
}

ModelConfigWidget::~ModelConfigWidget()
{
    SingleEngine& engine = SingleEngine::instance();
    //unbind message first;
    unbindMessage(engine);
    delete ui;
}

void ModelConfigWidget::setTask(const std::string& taskId)
{
    _taskId = taskId;
    // _originEditWidget->setShowTask(taskId, true);
    // _refinedEditWidget->setShowTask(taskId, false);
    ui->lineEdit_projectID->setText(toqs(_taskId));
}

std::string ModelConfigWidget::task() const
{
    return _taskId;
}

ModelTask* ModelConfigWidget::getTask()
{
    for (auto& t : project().modelTaskList) {
        if (t.id == _taskId) {
            return &t;
        }
    }
    return nullptr;
}

void ModelConfigWidget::init()
{
    unbindMessage(SingleEngine::instance());
    Task* t = getTask();
    if (t && t->id == SingleEngine::instance().currentTaskName()) {
        bindMessage(SingleEngine::instance());
    }
    refreshDatas();
}

bool ModelConfigWidget::checkStart()
{
    Task* task = getTask();
    SingleEngine& engine = SingleEngine::instance();
    if (engine.isRunning() || task == nullptr) {
        QMessageBox::information(this, tr("Warnning"), tr("Processing is running..."));
        return false;
    }
    return true;
}

void ModelConfigWidget::refreshDatas()
{
    ModelTask* task = getTask();
    if (task == nullptr) {
        return;
    }
    ui->spinBox_xcount->setValue(task->grid.xcount);
    ui->spinBox_ycount->setValue(task->grid.ycount);
    ui->spinBox_zcount->setValue(task->grid.zcount);
    _renderWidget->refreshDatas(*task);
    _gridFunc.task = task;
    _renderWidget->setGridCallBack(_gridFunc);
    ui->lineEdit_projectID->setText(toqs(task->id));
    //query task status
    queryTaskStatus();
}

void ModelConfigWidget::queryTaskStatus()
{
    ModelTask* task = getTask();

    if (task == nullptr)
        return;

    // if (task->info.ATStatus == ATTask::eFinished) {
    //     ui->groupBox_controlEdit->setEnabled(true);
    //     ui->groupBox_export->setEnabled(true);
    // } else {
    //     ui->groupBox_controlEdit->setEnabled(false);
    //     ui->groupBox_export->setEnabled(false);
    // }
}

void ModelConfigWidget::on_pushButton_Model_clicked()
{
    if (!checkStart())
        return;

    qDebug() << "on_pushButton_Model_clicked";
    ModelTask* task = getTask();
    if (task == nullptr)
        return;

    SingleEngine& engine = SingleEngine::instance();
    if (engine.isRunning())
        return;

    bindMessage(engine);
    EngineTaskATPtr engineTask(new EngineTaskAT);
    engineTask->type = eModel;
    engineTask->name = task->id;
    engineTask->folder = task->taskDir;
    //TODO(HY)
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
        // if (exitCode == EXIT_SUCCESS) {
        //     ModelTask* t = getTask();
        //     if (t) {
        //         t->readInfos();
        //         t->readRefined();
        //         t->readOriginMapCoord();
        //         this->refreshRefine();
        //         queryTaskStatus();
        //     }
        // }
    });
}

void ModelConfigWidget::on_pushButton_more_clicked()
{
    ui->checkBox_pointcloud->setVisible(!ui->checkBox_pointcloud->isVisible());
    ui->checkBox_texturemodel->setVisible(!ui->checkBox_texturemodel->isVisible());
    ui->checkBox_lod->setVisible(!ui->checkBox_lod->isVisible());
    return;

    if (!checkStart())
        return;

    ModelTask* task = getTask();
    if (task == nullptr)
        return;
    QDialog dlg(this);
    QVBoxLayout* layout = new QVBoxLayout;
    dlg.resize(dlg.sizeHint());
    dlg.setLayout(layout);
    dlg.exec();
}
void ModelConfigWidget::on_pushButton_stop_clicked()
{
    Task* task = getTask();
    if (!task)
        return;
    SingleEngine& engine = SingleEngine::instance();
    if (engine.isRunning() && engine.currentTaskName() == task->id)
        engine.stop();
}

void ModelConfigWidget::on_response(const ResponseHead& head)
{
    if (head.result != 0) {
        LOG(INFO) << head.errorMsg;
    }
}
void ModelConfigWidget::on_showMessage(const QByteArray& msg)
{
    // ui->plainTextEdit->appendPlainText(QString::fromLocal8Bit(msg));
    std::cout << tos(msg);
}

void ModelConfigWidget::bindMessage(SingleEngine& engine)
{
    connect(&engine, SIGNAL(message(const QByteArray&)),
        this, SLOT(on_showMessage(const QByteArray&)));
}

void ModelConfigWidget::unbindMessage(SingleEngine& engine)
{
    disconnect(&engine, SIGNAL(message(const QByteArray&)),
        this, SLOT(on_showMessage(const QByteArray&)));
}

void ModelConfigWidget::setButtonEnableState(bool enable)
{
    _renderWidget->setEnabled(enable);
}

void ModelConfigWidget::on_finished(int exitCode)
{
    QByteArray msg = QString("Exit process code=%1").arg(exitCode).toLatin1();
    this->on_showMessage(msg);
}

void ModelConfigWidget::on_pushButton_autoblock_clicked()
{
}

void ModelConfigWidget::on_gridCountChanged()
{
    if (!enableUpdateGrid)
        return;
    int xcount = ui->spinBox_xcount->value();
    int ycount = ui->spinBox_ycount->value();
    int zcount = ui->spinBox_zcount->value();
    if (ui->radioButton2d->isChecked()) {
        zcount = 1;
    }
    _renderWidget->onSetGridCount(xcount, ycount, zcount);
}

void ModelConfigWidget::SetGridFunction::operator()(int xcount, int ycount, int zcount)
{
    if (task == nullptr)
        return;
    task->grid.xcount = xcount;
    task->grid.ycount = ycount;
    task->grid.zcount = zcount;
}

}//name space insight
