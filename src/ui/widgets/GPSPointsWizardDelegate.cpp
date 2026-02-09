#include "GPSPointsWizardDelegate.h"
#include <QModelIndex>
#include <QLineEdit>
#include <QComboBox>



namespace insight{
GPSPointsWizardDelegate::GPSPointsWizardDelegate(QObject *parent /*= 0*/)
	:QStyledItemDelegate(parent)
{

}

//! [1]
QWidget *GPSPointsWizardDelegate::createEditor(QWidget *parent,
	const QStyleOptionViewItem &option,
	const QModelIndex & index ) const
{
	//QWidget *editor = NULL;
	if (index.row() == 0)
	{
		QComboBox *box = new QComboBox(parent);
		QList<QString> fields = _doc->fieldNames();
		box->insertItem(0, tr("Undefined"));
		for (int i = 0;  i < fields.size(); ++i)
		{
			box->insertItem(i + 1, fields[i]);
		}
		box->setCurrentIndex(0);
		return box;
	}
    return nullptr;
}
//! [1]

//! [2]
void GPSPointsWizardDelegate::setEditorData(QWidget *editor,
	const QModelIndex &index) const
{
	if (index.row() == 0)
	{
		QString value = index.model()->data(index, Qt::EditRole).toString();
		QComboBox *box = static_cast<QComboBox *>(editor);
		box->setCurrentText(value);
	}
}
//! [2]

//! [3]
void GPSPointsWizardDelegate::setModelData(QWidget *editor, QAbstractItemModel *model,
	const QModelIndex &index) const
{
	if (index.row() == 0)
	{
		QComboBox *box = static_cast<QComboBox*>(editor);
		QString value = box->currentText();
		model->setData(index, value, Qt::EditRole);
	}

}
//! [3]

//! [4]
void GPSPointsWizardDelegate::updateEditorGeometry(QWidget *editor,
	const QStyleOptionViewItem &option, const QModelIndex &/* index */) const
{
	editor->setGeometry(option.rect);
}
//! [4]

}//name space insight

