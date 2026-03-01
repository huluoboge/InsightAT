#ifndef INSIGHTAT_ABOUT_H
#define INSIGHTAT_ABOUT_H

#include <QDialog>
#include "InsightATGlobal.h"

#include "ui_About.h"

namespace insight{
class About : public QDialog
{
    Q_OBJECT
public:
    About(QWidget *parent);
    ~About();
    Ui::About ui;
};
}//name space insight
#endif // INSIGHTAT_ABOUT_H
