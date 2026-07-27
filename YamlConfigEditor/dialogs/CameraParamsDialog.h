#ifndef CAMERAPARAMSDIALOG_H
#define CAMERAPARAMSDIALOG_H

#include "FullScreenDialog.h"
#include "AllConfig.h"
#include <QLineEdit>
#include <QDoubleSpinBox>

class QTableWidget;
class QPushButton;

// 编辑单个相机参数的子对话框（无需全屏，使用普通对话框）
class CameraParamEditDialog : public QDialog
{
    Q_OBJECT
public:
    explicit CameraParamEditDialog(CameraParams& params, QWidget *parent = nullptr);

private slots:
    void onSave();

private:
    CameraParams& m_params;
    QLineEdit* m_kEdit[9];
    QLineEdit* m_rEdit[9];
    QLineEdit* m_tEdit[3];
    QLineEdit* m_dEdit[5];
    QDoubleSpinBox* m_ppxSpin;
    QDoubleSpinBox* m_ppySpin;
    QDoubleSpinBox* m_focalSpin;
    QDoubleSpinBox* m_aspectSpin;
};

// 相机参数列表管理对话框（全屏）
class CameraParamsDialog : public FullScreenDialog
{
    Q_OBJECT
public:
    explicit CameraParamsDialog(std::vector<CameraParams>& paramsList, QWidget *parent = nullptr);

private slots:
    void onAdd();
    void onEdit();
    void onDelete();
    void onSave();
    void onRestoreDefault() override;

private:
    void setupContent();
    void updateTable();

    std::vector<CameraParams>& m_paramsList;
    QTableWidget* m_table;
};

#endif // CAMERAPARAMSDIALOG_H
