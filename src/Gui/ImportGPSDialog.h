#ifndef ImportGPSDialog_h
#define ImportGPSDialog_h

#include "GPSPointsWizardDialog.h"
#include "InsightATGlobal.h"
#include <QList>
#include <QString>
#include <vector>
#include <string>


namespace insight{

class ImportGPSDialog : public GPSPointsWizardDialog
{
	Q_OBJECT

public:
	struct Pose{
		std::string name;
		double x = 0;
		double y = 0;
		double z = 0;
		double omega = 0;
		double phi = 0;
		double kappa = 0;
	};
	typedef std::vector<Pose> Vec_Pose;
    ImportGPSDialog(QWidget *parent = nullptr);
    virtual ~ImportGPSDialog();
	const Vec_Pose &Points() const{
		return m_vecPoints;
	}

	virtual bool checkFieldData(int rowFrom,const std::vector<int> &fieldIndex);
	virtual QList<QString> fieldNames() const;

protected:
	virtual bool showCustemDialog();
private:
	Vec_Pose m_vecPoints;
};


}//name space insight
#endif // ImportGPSDialog
