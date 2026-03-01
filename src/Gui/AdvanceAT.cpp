#include "AdvanceAT.h"

#include <QString>
#include <QWidget>
#include <cassert>

namespace insight{

AdvanceAT::AdvanceAT(QWidget* parent)
    : QWidget(parent)
{
    ui.setupUi(this);
}

AdvanceAT::~AdvanceAT()
{
}

void AdvanceAT::on_pushButton_process_clicked()
{
    bool doFeat = ui.checkBox_featureDetect->isChecked();
    bool doMatch = ui.checkBox_matching->isChecked();
    bool doAT = ui.checkBox_at->isChecked();
    if(!doFeat && !doMatch && !doAT){
        
    }
}

}//name space insight
