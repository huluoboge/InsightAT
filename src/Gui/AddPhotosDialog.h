#ifndef WPOADDPHOTOSDIALOG_H
#define WPOADDPHOTOSDIALOG_H

#include <QDialog>
#include "ui_AddPhotosDialog.h"
#include "Common/string_utils.h"
#include "Document.h"
#include "Utils.h"

namespace insight{
class AddPhotosDialog : public QDialog
{
    Q_OBJECT

public:
    enum AddCameraType
    {
        eNew,
        eSelect,
        eByExif,
    };

    AddPhotosDialog(QWidget *parent = nullptr);
    ~AddPhotosDialog();

    void initGroups(const QStringList &groupNames, const QList<int> &groupIds);

    AddCameraType addCameraType() const;

    QString newCameraName() const { return (ui.lineEdit_newGroupName->text()); }

    int selectCameraId() const;

    void setNextCameraId(int groupId);

private slots:
    void selectItem(int idx){ m_selectIndex = idx; }
private:
    void enableSetGroup(bool enable);

    int m_selectIndex;
    QStringList m_groupNames;
    QList<int> m_groupIds;
    Ui::AddPhotosDialog ui;
};
}//name space insight
#endif // WPOADDPHOTOSDIALOG_H
