#ifndef PROJECTEDITWIDGET_H
#define PROJECTEDITWIDGET_H

#include <QWidget>
#include <QSplitter>

#include "SubWidget.h"
#include "cameraedit.h"
#include "ImageAttributes.h"

namespace insight{
class ProjectEditWidget : public SubWidget
{
	Q_OBJECT

public:
	ProjectEditWidget(QWidget *parent = nullptr);
	~ProjectEditWidget();

	void init();
	void refreshDatas();

    void setShowTask(const std::string taskId, bool isOrigin);
    void enable(){}
    void disable(){}
    void disableCameraEdit(){
        _camEdit->setEditable(false);
    }

private:
	QSplitter *_splitter;
	CameraEdit *_camEdit;
	ImageAttributes *_imageAttribute;
};

}//name space insight
#endif // PROJECTEDITWIDGET_H
