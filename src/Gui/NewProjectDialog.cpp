#include "NewProjectDialog.h"

#include <QFileInfo>
#include <QPushButton>
#include <QFileDialog>
#include <QMessageBox>

#include "Settings.h"

namespace insight{
NewProjectDialog::NewProjectDialog(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
    ui.lineEdit_projectLocation->setText(settings().recentProjectPath());
}

NewProjectDialog::~NewProjectDialog()
{

}

bool NewProjectDialog::onValid()
{
	QFileInfo fileInfo(ui.lineEdit_projectLocation->text());
	
	if (ui.lineEdit_projectName->text().isEmpty() || !fileInfo.isDir())
	{
		ui.buttonBox->button(QDialogButtonBox::Ok)->setEnabled(false);
		return false;
	}
	ui.buttonBox->button(QDialogButtonBox::Ok)->setEnabled(true);
	return true;
}

void NewProjectDialog::onOK()
{
	if (onValid())
	{
		QDir dir(location());
		if (!dir.exists())
		{
			if (!dir.mkpath(location())){
				QMessageBox::critical(this, tr("Error"), tr("Can't create folder!"));
				return;
			}
		}
		ui.lineEdit_projectLocation->setText(dir.absolutePath());
		accept();
		return;
	}
}

void NewProjectDialog::onSelectLocation()
{
	QString dirPath = QFileDialog::getExistingDirectory(this, tr("Select project location..."), 
		settings().recentProjectPath());
	if (!dirPath.isEmpty())
		ui.lineEdit_projectLocation->setText(dirPath);
}

QString NewProjectDialog::name() const
{
	return ui.lineEdit_projectName->text();
}

QString NewProjectDialog::location() const
{
	return ui.lineEdit_projectLocation->text();
}

QString NewProjectDialog::description() const
{
	return ui.plainTextEdit_description->toPlainText();
}

QString NewProjectDialog::author() const
{
	return ui.lineEdit_projectLocation_author->text();
}
}//name space insight
