#ifndef WATCONTROLPOINTSWIZARDDIALOG_H
#define WATCONTROLPOINTSWIZARDDIALOG_H

#include <QDialog>
#include "ui_GPSPointsWizardDialog.h"

#include "GPSPointsWizardModel.h"

#include <Eigen/Core>

#include "GPSPointsWizardDelegate.h"
#include "Common/Coordinates.h"

namespace insight{


class GPSPointsWizardDialog : public QDialog, ImportDataBaseDocument
{
	Q_OBJECT

public:
	
    GPSPointsWizardDialog(QWidget *parent = 0);
    ~GPSPointsWizardDialog();
	void setFile(const QString &fileFullPath);
	
    GPSPointsDocument *getDoc() { return &m_document; }
	bool isImportByName()const;
public slots:
	void updateModel();
	void preview();//Ԥ��
	void checkEnablePreview();
	void validImport();
	void enableSelectImportOption(bool enable);
	bool hasOmegaPhiKappa() const;

	int angleUnit() const ;
	int coordinateSystem() const;
	int eulerAngleSystem() const;
protected:
	bool valid();
	//bool checkFields(int &rowFrom,std::vector<int> &fieldIndex);
	void getFieldIndex(int &rowFrom, std::vector<int> &fieldIndex);
	virtual bool showCustemDialog() { return true; }
	
    Ui::GPSPointsWizardDialog ui;
    GPSPointsDocument m_document;
    GPSPointsWizardModel *m_model;
	//Coordinate m_coord;
};

}//name space insight

#endif // WATCONTROLPOINTSWIZARDDIALOG_H
