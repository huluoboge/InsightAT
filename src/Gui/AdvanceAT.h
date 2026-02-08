#ifndef ADVANCEAT_H
#define ADVANCEAT_H

#include <QObject>

#include <QWidget>
#include "ui_AdvanceAT.h"
#include "Common/string_utils.h"
#include "Document.h"
#include "Utils.h"

namespace insight{

class AdvanceAT : public QWidget
{
    Q_OBJECT
signals:

public:
    AdvanceAT(QWidget *parent = nullptr);
    ~AdvanceAT();
public slots:
    void on_pushButton_process_clicked();
private:
    Ui::AdvanceAT ui;
};

}//name space insight
#endif // ADVANCEAT_H
