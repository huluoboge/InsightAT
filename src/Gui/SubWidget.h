

#ifndef SubWidget_h_
#define SubWidget_h_

#include <QWidget>
#include <QDialog>
#include <QVBoxLayout>
#include <QMdiSubWindow>
#include <QEvent>
#include <functional>

#include "InsightATGlobal.h"

namespace insight{
class SubWidget : public QWidget
{
    Q_OBJECT
signals:
    void closeWindow();
public:
    SubWidget(QWidget *parent = nullptr)
        :QWidget(parent){
    }
    virtual void init() = 0;
    virtual void refreshDatas() = 0;
    virtual void enable() = 0;
    virtual void disable() = 0;

};

inline void showSubWidget(QWidget *parent, SubWidget *widget)
{
    QDialog dlg(parent);
    QVBoxLayout *layout = new QVBoxLayout;
    widget->init();
    layout->addWidget(widget);
    dlg.setLayout(layout);
    QObject::connect(widget, SIGNAL(closeWindow()), &dlg, SLOT(accept()));
    dlg.exec();
}


class MdiSubWindow: public QMdiSubWindow
{
public:
    MdiSubWindow(QWidget *parent = nullptr)
        :QMdiSubWindow(parent)
    {
    }
    ~MdiSubWindow(){}

    void setCloseFunction(std::function<bool()> fuc)
    {
        _function = fuc;
    }
    virtual bool showClose(){
        if(_function == nullptr)
            return true;
        else{
            return _function();
        }
    }
protected:
    virtual bool event(QEvent *e)
    {
      if(e->type()==QEvent::Close){
//              qDebug()<<"Close\n";//打印一个关闭的信息，可以看到SubWindow并没有关闭
          if(showClose()){
              return QMdiSubWindow::event(e);
          }else{
            e->ignore ();//忽略关闭用于测试
             return true;//返回true表示 阻止event
          }
        }
      return QMdiSubWindow::event(e);
    }

    std::function<bool()> _function;
};

}//name space insight
#endif // SubWidget_h_
