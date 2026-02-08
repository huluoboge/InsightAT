#include "ProgressDialog.h"

#include <QMainWindow>
#include <QMessageBox>
#include <iostream>
#include <ctime>
#include "Common/string_utils.h"
#include <functional>

namespace insight{

ProgressDialog::ProgressDialog(QWidget *parent)
: QDialog(parent)
, _thread(nullptr)
, _finished(false)
{
	setWindowFlags(windowFlags() & ~Qt::WindowCloseButtonHint);
	ui.setupUi(this);
	_msgWidget = new QP_inner::MessageWidget(this);
	_msgWidget->resize(10, 10);
	_msgWidget->hide();
	_waitExitThread = new QP_inner::WaitExitThread(this);
	
	connect(this, SIGNAL(processFinished()), this, SLOT(onFinished()));
	totalBar()->setMinimum(0);
	totalBar()->setMaximum(0);
	connect(this, SIGNAL(exitDlg()), this, SLOT(accept()), Qt::QueuedConnection);
    connect(this, SIGNAL(updateSubBar(float)), this, SLOT(onSetPercent(float)), Qt::QueuedConnection);
    globalDlg = this;
}

ProgressDialog::~ProgressDialog()
{
	delete _waitExitThread;
}

ProgressDialog* ProgressDialog::globalDlg = nullptr;

int ProgressDialog::progress(float percent, const char *msg)
{
    Q_UNUSED(msg)
    emit globalDlg->updateSubBar(percent);
    return 0;
}

void ProgressDialog::onMinimize()
{
	QWidget *w = (QWidget*)parent();
    if (w != nullptr)
	{
		w->showMinimized();
	}
}

int ProgressDialog::exec()
{
	if (_thread){
		std::function<void()> func = [this](){
			emit processFinished();
		};
		_thread->AddCallback(Thread::FINISHED_CALLBACK, func);
		_thread->Start();
	}
	int result = Base::exec();
	return result;
}

void ProgressDialog::onFinished()
{
	accept();
}

void ProgressDialog::onSetPercent(float percent)
{
    int v = int(100 *percent);
    totalBar()->setValue(v);
   // printf("%f-", double(percent));
}
bool ProgressDialog::nativeEvent(const QByteArray &eventType, void *message, long *result)
{
	Q_UNUSED(eventType);
	return QDialog::nativeEvent(eventType, message, result);
}

QProgressBar * ProgressDialog::totalBar()
{
	return ui.progressBar_total;
}

//////////////////////////////////////////////////////////////////////////
QP_inner::MessageWidget::MessageWidget(ProgressDialog *parent)
:QWidget(parent)
{
}
QP_inner::MessageWidget::~MessageWidget()
{
}

bool QP_inner::MessageWidget::nativeEvent(const QByteArray &eventType, void *message, long *result)
{
#ifdef WIN32
	const MSG *msg = static_cast<MSG*>(message);
 	return QWidget::nativeEvent(eventType, message, result);
#else
    return QWidget::nativeEvent(eventType, message, result);
#endif
}

}//name space insight

