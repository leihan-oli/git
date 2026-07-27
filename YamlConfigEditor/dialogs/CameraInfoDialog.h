#ifndef CAMERAINFODIALOG_H
#define CAMERAINFODIALOG_H

#include "FullScreenDialog.h"
#include "AllConfig.h"

class QTableWidget;
class QPushButton;

class CameraInfoDialog : public FullScreenDialog
{
    Q_OBJECT
public:
    explicit CameraInfoDialog(std::vector<CameraInfo>& cameraList, QWidget *parent = nullptr);

private slots:
    void onAdd();
    void onEdit();
    void onDelete();
    void onSave();
    void onRestoreDefault() override;

private:
    void setupContent();
    void refreshTable();
    int getCurrentRow() const;

    std::vector<CameraInfo>& m_cameraList;
    QTableWidget* m_table;
};

#endif // CAMERAINFODIALOG_H
