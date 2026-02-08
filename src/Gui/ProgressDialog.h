#ifndef WPOPROGRESSDIALOG_H
#define WPOPROGRESSDIALOG_H
#include "ui_ProgressDialog.h"

#include <QProgressDialog>

#include <QDialog>
#include <QThread>
#include <QTimer>
#include <QThread>
#include <QMainWindow>
#include <fstream>

#include <functional>
#include <QTime>
#include "Common/threading.h"
#include "InsightATGlobal.h"

namespace insight{


typedef std::function<void()> ProcessFunction;



namespace QP_inner
{
	class WaitExitThread;
	class MessageWidget;
}

class ProgressDialog : public QDialog
{
	Q_OBJECT
	typedef QDialog Base;
signals :
	void updateSubBar(float);
	void processFinished();
	void exitDlg();
public:
	
    static ProgressDialog *globalDlg;
    static int progress(float percent, const char *msg);
	ProgressDialog(QWidget *parent);
	~ProgressDialog();


	void setProcessThread(Thread * thread){
		_thread = thread;
	}

	int exec();

	void wait()	{
		_thread->Wait();
	}
	void exit(){
		emit exitDlg();
	}
	QProgressBar *totalBar();
	public slots:
	void onFinished();
	void onMinimize();
    void onSetPercent(float);
private slots:
private:
	bool nativeEvent(const QByteArray &eventType, void *message, long *result);
	
    Ui::ProgressDialog ui;
	Thread * _thread;
	bool _finished;
	QP_inner::WaitExitThread *_waitExitThread;
	QP_inner::MessageWidget *_msgWidget;
};

namespace QP_inner
{
	class WaitExitThread : public QThread
	{
		Q_OBJECT
	signals:
		void rejected();
	public:
		WaitExitThread(ProgressDialog *parent)
			:QThread(parent)
		{

		}
	protected:
		virtual void run()
		{
			ProgressDialog *dialog = (ProgressDialog*)parent();
			dialog->wait();
			emit rejected();
		}
	};

	class MessageWidget : public QWidget{
		Q_OBJECT
	public:
        MessageWidget(ProgressDialog *parent);
		~MessageWidget();
	private:
		bool nativeEvent(const QByteArray &eventType, void *message, long *result);
		//AlgWindowsWStream *m_windowLoggerReadStream;									//�㷨��־��Ϣ��
		
	};
}

}//name space insight

#endif // WPOPROGRESSDIALOG_H
