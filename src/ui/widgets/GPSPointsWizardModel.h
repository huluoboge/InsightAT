#ifndef GPSPointsWizardModel_h
#define GPSPointsWizardModel_h

#include <QAbstractTableModel>
#include <QVariant>
#include <QStringList>
#include <QList>
#include <cassert>

namespace insight{

struct GPSPointsWizardModelFilter
{
    GPSPointsWizardModelFilter()
		//:m_useIndices(false)
	{}
	
	std::vector<int> m_sortedIndices;
	void init();
};

struct GPSPointsDocument
{
	typedef QStringList  RowData;
	QList<RowData> m_tableData;
	QList<QString> m_fields;
    int m_rowFrom = 0;//row form 0
	QString m_txt;

    // semicolon  ;
    // comma
    // tab

	int m_columns;
	void updateColumnCount()
	{
		int maxCol = 0;
		for (int i = 0; i < m_tableData.count(); ++i)
		{
			if (m_tableData[i].count() > maxCol)
			{
				maxCol = m_tableData[i].count();
			}
		}
		m_columns = maxCol;
		m_fields.clear();

		for (int i = 0; i < m_columns; ++i)
		{
			m_fields.append(QObject::tr("Undefined"));
		}
	}

	void parse(bool tab, bool semicolon, bool comma ,bool space, QString other, bool multiAsSingle)
	{
		m_tableData.clear();
		m_fields.clear();

		QString left("[");
		QString right("]");
		if (multiAsSingle)
		{
			right += "+";
		}
		QString regString;
		
		if (tab)
		{
			regString.append("\\t");
		}

		if (semicolon)
		{
			regString.append(";");
		}

		if (comma)
		{
			regString.append(",");
		}

		if (space)
		{
			regString.append("\\s");
		}

		regString.append(other);
		QString regStr = left + regString + right;

		QRegExp reg(regStr);
		if (!reg.isValid())
		{
			return;
		}
		QStringList lines = m_txt.split('\n');

		for (int i = 0; i < lines.count(); ++i)
		{
			if (lines[i].isEmpty())continue;
			QString line = lines[i].trimmed();
			QStringList aLine = line.split(reg);
			if (!aLine.isEmpty())
			{
				m_tableData.append(aLine);
			}
		}
		updateColumnCount();
	}
    GPSPointsDocument() :m_columns(0)
	{

	}

};

class GPSPointsWizardModel : public QAbstractTableModel
{
	Q_OBJECT
public:
    explicit GPSPointsWizardModel(QObject *parent = NULL);
    ~GPSPointsWizardModel();

public:

	virtual int rowCount(const QModelIndex &parent = QModelIndex()) const;
	virtual int columnCount(const QModelIndex &parent = QModelIndex()) const;

	virtual QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;
	virtual bool setData(const QModelIndex &index, const QVariant &value, int role  = Qt::EditRole );
	virtual QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole ) const;

	Qt::ItemFlags flags(const QModelIndex & index) const;

	//virtual void sort(int column, Qt::SortOrder order  = Qt::AscendingOrder );

    GPSPointsWizardModelFilter filter() const { return m_filter; }
    void setFilter(const GPSPointsWizardModelFilter &filter) { m_filter = filter; }

	void filterInit() { m_filter.init(); }

    void setDataSource(GPSPointsDocument *data) { m_data = data; }

	void updateDatas(){ beginResetModel(); endResetModel(); }
private:
    GPSPointsDocument *m_data;
	int m_columns;
    GPSPointsWizardModelFilter m_filter;
};

}//name space insight
#endif // WATGPSPointsModel_h__
