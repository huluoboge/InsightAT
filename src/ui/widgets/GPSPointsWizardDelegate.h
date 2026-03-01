#ifndef WATGPSPointsDelegate_h
#define WATGPSPointsDelegate_h

#include <QStyledItemDelegate>
#include <QList>
#include <QStringList>

namespace insight{
struct ImportDataBaseDocument
{
	virtual bool checkFieldData(int rowFrom, const std::vector<int> &fieldIndex) = 0;
	virtual QList<QString> fieldNames() const = 0;
};


class GPSPointsWizardDelegate : public QStyledItemDelegate
{
	Q_OBJECT
public:
    explicit GPSPointsWizardDelegate(QObject *parent = 0);

    void setDataBseDocument(ImportDataBaseDocument *doc){ _doc = doc; }

	QWidget *createEditor(QWidget *parent, const QStyleOptionViewItem &option,
		const QModelIndex &index) const;

	void setEditorData(QWidget *editor, const QModelIndex &index) const;
	void setModelData(QWidget *editor, QAbstractItemModel *model,
		const QModelIndex &index) const;

	void updateEditorGeometry(QWidget *editor,
		const QStyleOptionViewItem &option, const QModelIndex &index) const;

private:
    ImportDataBaseDocument *_doc = nullptr;
};

}//name space insight
#endif
