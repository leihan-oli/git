#ifndef CAMERAADJUSTDIALOG_H
#define CAMERAADJUSTDIALOG_H

#include "FullScreenDialog.h"
#include "AllConfig.h"

class QComboBox;
class QSpinBox;

class CameraAdjustDialog : public FullScreenDialog
{
    Q_OBJECT
public:
    explicit CameraAdjustDialog(AllConfig::CameraAdjust& adjustRef, QWidget *parent = nullptr);

private slots:
    void onSave();
    void onRestoreDefault() override;

private:
    void setupContent();

    AllConfig::CameraAdjust& m_adjustRef;
    QComboBox* m_adjusterTypeCombo;
    QComboBox* m_waveCorrectionCombo;
    QComboBox* m_warperTypeCombo;
    QComboBox* m_estimatorTypeCombo;
    QSpinBox* m_alignDepthSpin;
};

#endif // CAMERAADJUSTDIALOG_H
