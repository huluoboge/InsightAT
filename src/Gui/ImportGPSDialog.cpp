#include "ImportGPSDialog.h"
#include <QMessageBox>
#include <QFileInfo>
#include "Common/string_utils.h"
#include "Document.h"

namespace insight{

ImportGPSDialog::ImportGPSDialog(QWidget *parent /*= 0*/)
	: GPSPointsWizardDialog(parent)
{
}

ImportGPSDialog::~ImportGPSDialog()
{
}

bool ImportGPSDialog::checkFieldData(int rowFrom, const std::vector<int> &fieldIndex)
{
	//�������
	bool hasIMU = hasOmegaPhiKappa();

	const int from = rowFrom;
	{
		int namdIdx = fieldIndex[0];
		int xIdx = fieldIndex[1];
		int yIdx = fieldIndex[2];
		int zIdx = fieldIndex[3];
		int omegaIdx = fieldIndex[4];
		int phiIdx = fieldIndex[5];
		int kappaIdx = fieldIndex[6];
		if (namdIdx == -1)
		{
			QMessageBox::information(this, tr("Error"), tr("Missing name"));
			return false;
		}
		if (xIdx == -1 || yIdx == -1 || zIdx == -1)
		{
			QMessageBox::information(this, tr("Error"), tr("Missing X,Y,Z"));
			return false;
		}

		if (hasOmegaPhiKappa())
		{
			if (phiIdx == -1 || omegaIdx == -1 || kappaIdx == -1)
			{
				QMessageBox::information(this, tr("Error"), tr("Missing Omega, Phi, Kappa"));
				return false;
			}
		}
		int maxCol = std::max(xIdx, yIdx);
		maxCol = std::max(zIdx, maxCol);

		if (hasOmegaPhiKappa())
		{
			maxCol = std::max(omegaIdx, maxCol);
			maxCol = std::max(phiIdx, maxCol);
			maxCol = std::max(kappaIdx, maxCol);
		}
		for (int i = from; i < m_document.m_tableData.count(); ++i)
		{
			bool ok(false);
			if (m_document.m_tableData[i].size() < maxCol + 1)
			{
				QMessageBox::information(this, tr("Error"), tr("Missing data in row %1").arg(i + 1));
				return false;
			}
			m_document.m_tableData[i][xIdx].trimmed().toDouble(&ok);
			if (!ok)
			{
				QMessageBox::information(this, tr("Error"), tr("X value is not a number in row: %1").arg(i + 1));
				return false;
			}
			m_document.m_tableData[i][yIdx].trimmed().toDouble(&ok);
			if (!ok)
			{
				QMessageBox::information(this, tr("Error"), tr("Y value is not a number in row: %1").arg(i + 1));
				return false;
			}
			m_document.m_tableData[i][zIdx].trimmed().toDouble(&ok);
			if (!ok)
			{
				QMessageBox::information(this, tr("Error"), tr("Z value is not a number in row: %1").arg(i + 1));
				return false;
			}
			//////////////////////////////////////////////////////////////////////////
			if (hasIMU)
			{
				m_document.m_tableData[i][omegaIdx].trimmed().toDouble(&ok);
				if (!ok)
				{
					QMessageBox::information(this, tr("Error"), tr("Omega value is not a number in row: %1").arg(i + 1));
					return false;
				}
				m_document.m_tableData[i][phiIdx].trimmed().toDouble(&ok);
				if (!ok)
				{
					QMessageBox::information(this, tr("Error"), tr("Phi value is not a number in row: %1").arg(i + 1));
					return false;
				}
				m_document.m_tableData[i][kappaIdx].trimmed().toDouble(&ok);
				if (!ok)
				{
					QMessageBox::information(this, tr("Error"), tr("Kappa value is not a number in row: %1").arg(i + 1));
					return false;
				}
			}
		}
	}

	m_vecPoints.clear();

	for (int i = from; i < m_document.m_tableData.count(); ++i)
	{
		Pose p;
		int nameIdx = fieldIndex[0];
		int xIdx = fieldIndex[1];
		int yIdx = fieldIndex[2];
		int zIdx = fieldIndex[3];
		int omegaIdx = fieldIndex[4];
		int phiIdx = fieldIndex[5];
		int kappaIdx = fieldIndex[6];
		QString name = m_document.m_tableData[i][nameIdx];
		name = QFileInfo(name).baseName().toLower();
		p.x = m_document.m_tableData[i][xIdx].toDouble();
		p.y = m_document.m_tableData[i][yIdx].toDouble();
		p.z = m_document.m_tableData[i][zIdx].toDouble();
		if (hasIMU)
		{
			p.omega = m_document.m_tableData[i][omegaIdx].toDouble();
			p.phi = m_document.m_tableData[i][phiIdx].toDouble();
			p.kappa = m_document.m_tableData[i][kappaIdx].toDouble();
		}
		p.name = tos(name);
		m_vecPoints.push_back(p);
	}
	return true;
}

QList<QString> ImportGPSDialog::fieldNames() const
{
	QList<QString> fields;
	fields
		<< tr("Key")
		<< tr("X/Lon")
		<< tr("Y/Lat")
		<< tr("Z/Alt")
		<< tr("Omega")
		<< tr("Phi")
		<< tr("Kappa");
	bool hasIMU = hasOmegaPhiKappa();
	if (!hasIMU)
	{
		fields.clear();
		fields << tr("Key")
			   << tr("X/Lon")
			   << tr("Y/Lat")
			   << tr("Z/Alt");
	}
	return fields;
}

bool ImportGPSDialog::showCustemDialog()
{
	return true;
}

}//name space insight
