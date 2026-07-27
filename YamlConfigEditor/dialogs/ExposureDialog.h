#ifndef EXPOSUREDIALOG_H
#define EXPOSUREDIALOG_H

#include "FullScreenDialog.h"
#include "AllConfig.h"

class QComboBox;
class QSpinBox;

class ExposureDialog : public FullScreenDialog
{
    Q_OBJECT
public:
    explicit ExposureDialog(AllConfig::Exposure& exposureRef, QWidget *parent = nullptr);

private slots:
    void onSave();
    void onRestoreDefault() override;

private:
    void setupContent();

    AllConfig::Exposure& m_exposureRef;
    QComboBox* m_expTypeCombo;
    QSpinBox* m_expNrFeedsSpin;
    QSpinBox* m_expNrFilteringSpin;
    QSpinBox* m_expBlockSizeSpin;
};

#endif // EXPOSUREDIALOG_H
