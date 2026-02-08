#ifndef WPONEWPROJECTDIALOG_H
#define WPONEWPROJECTDIALOG_H

#include <QDialog>
#include "ui_NewProjectDialog.h"
#include "InsightATGlobal.h"
namespace insight{
class NewProjectDialog : public QDialog
{
	Q_OBJECT

public:
	NewProjectDialog(QWidget *parent = 0);
	~NewProjectDialog();

	QString name() const;
	QString location() const;
	QString description() const;
	QString author() const;
private slots:
	bool onValid();
	void onOK();
	void onSelectLocation();
private:
	Ui::NewProjectDialog ui;
};
}//name space insight
#endif // WPONEWPROJECTDIALOG_H
