#include "GPSPointsWizardModel.h"

#include <QFont>

namespace insight{
void GPSPointsWizardModelFilter::init()
{
}

//////////////////////////////////////////////////////////////////////////

GPSPointsWizardModel::GPSPointsWizardModel(QObject *parent /*= NULL*/)
:m_data(0)
{
}

GPSPointsWizardModel::~GPSPointsWizardModel()
{

}


int GPSPointsWizardModel::rowCount(const QModelIndex &parent /*= QModelIndex()*/) const
{
	int count = m_data->m_tableData.count() - m_data->m_rowFrom;
	if (count < 0)
	{
		return 1;
	}
	return count + 1;
}

int GPSPointsWizardModel::columnCount(const QModelIndex &parent /*= QModelIndex()*/) const
{
	return m_data->m_columns;
}

QVariant GPSPointsWizardModel::data(const QModelIndex &index, int role /*= Qt::DisplayRole*/) const
{
	int col = index.column();
	int row = index.row();

    if (row == 0) //
	{
		if (role == Qt::DisplayRole || role == Qt::EditRole)
		{
			return m_data->m_fields[col];
		}
		else if (role == Qt::FontRole)
		{
			QFont boldFont;
			boldFont.setBold(true);
			return boldFont;
		}
		else
		{
			return QVariant();
		}
	}
	else
	{
		row += m_data->m_rowFrom;
		--row;//��һ����ѡ�����������ʲô�ֶ�

		if (role == Qt::DisplayRole)
		{
			if (row >= m_data->m_tableData.count())
			{
				return QVariant();
			}
			if (col >= m_data->m_tableData[row].count())
			{
				return QString("null");
			}
			return m_data->m_tableData[row][col];
		}
	}
	
	
	return QVariant();
}

QVariant GPSPointsWizardModel::headerData(int section, Qt::Orientation orientation, int role /*= Qt::DisplayRole */) const
{
	if (role == Qt::DisplayRole)
	{
		if (orientation == Qt::Horizontal)
		{
			return section + 1;
		}
		else
		{
			if (section == 0)
			{
				return tr("Data type");
			}
			else
			{
				return section;
			}
		}
	}
	return QAbstractTableModel::headerData(section, orientation, role);
}

bool GPSPointsWizardModel::setData(const QModelIndex &index, const QVariant &value, int role /*= Qt::EditRole */)
{

	if (role == Qt::EditRole)
	{
		int row = index.row();

		if (row != 0)
		{
			return false;
		}

		int col = index.column();
		m_data->m_fields[col] = value.toString();
		return true;
	}
	else
	{
		return false;
	}
}

Qt::ItemFlags GPSPointsWizardModel::flags(const QModelIndex & index) const
{
	Qt::ItemFlags flags;
	flags |= Qt::ItemIsSelectable;
	flags |= Qt::ItemIsEnabled;
	
	if (index.row() == 0) //�༭����
	{
		flags |= Qt::ItemIsEditable;
	}
	return flags;
}

}//name space insight
