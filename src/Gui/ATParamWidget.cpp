#include "ATParamWidget.h"
#include "Common/Project.h"
#include "Document.h"

namespace insight{

ATParamWidget::ATParamWidget(QWidget* parent)
    : QWidget(parent)
{
    ui.setupUi(this);
    init();
}

ATParamWidget::~ATParamWidget()
{
}

ATTask* ATParamWidget::getTask()
{
    ATTask* task = nullptr;
    for (auto& t : project().atTaskList) {
        if (t.id == _taskId) {
            task = &t;
            break;
        }
    }
    return task;
}
void ATParamWidget::init()
{
    //	ui.comboBox_keyMode->setCurrentIndex(_mode);
    ui.widget->setTaskId(_taskId);
    ui.widget->readCameras();
    ATTask* task = getTask();
    if (task == nullptr) {
        return;
    }
    ui.doubleSpinBox_location->setValue(task->info.gpsPrecision);
    ui.doubleSpinBox_maxError->setValue(task->info.gpsMaxError);
    ui.checkBox->setChecked(task->info.enableGNSSBA);
    ui.spinBox->setValue(task->info.maxLinkFeatures);
    ui.doubleSpinBox_location_2->setValue(task->info.maxReprojectError);
}

void ATParamWidget::saveData()
{
    //	_mode = ui.comboBox_keyMode->currentIndex();
    ui.widget->saveCameras();
    ATTask* task = getTask();
    if (task == nullptr) {
        return;
    }
    task->info.gpsPrecision = ui.doubleSpinBox_location->value();
    task->info.gpsMaxError = ui.doubleSpinBox_maxError->value();
    task->info.enableGNSSBA = ui.checkBox->isChecked();
    task->info.maxLinkFeatures = ui.spinBox->value();
    task->info.maxReprojectError = ui.doubleSpinBox_location_2->value();
    task->writeInfos();
}

}//name space insight
