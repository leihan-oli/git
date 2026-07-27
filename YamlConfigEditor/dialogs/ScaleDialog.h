#ifndef SCALEDIALOG_H
#define SCALEDIALOG_H

#include "FullScreenDialog.h"
#include "AllConfig.h"

class QDoubleSpinBox;

class ScaleDialog : public FullScreenDialog
{
    Q_OBJECT
public:
    explicit ScaleDialog(AllConfig::Scale& scaleRef, QWidget *parent = nullptr);

private slots:
    void onSave();
    void onRestoreDefault() override;

private:
    void setupContent();

    AllConfig::Scale& m_scaleRef;
    QDoubleSpinBox* m_inputScaleSpin;
    QDoubleSpinBox* m_homoEstScaleSpin;
    QDoubleSpinBox* m_expEstScaleSpin;
    QDoubleSpinBox* m_seamFinderScaleSpin;
    QDoubleSpinBox* m_blenderScaleSpin;
};

#endif // SCALEDIALOG_H
