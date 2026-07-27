#ifndef SEAMDIALOG_H
#define SEAMDIALOG_H

#include "FullScreenDialog.h"
#include "AllConfig.h"

class QComboBox;
class QCheckBox;
class QSpinBox;

class SeamDialog : public FullScreenDialog
{
    Q_OBJECT
public:
    explicit SeamDialog(AllConfig::Seam& seamRef, QWidget *parent = nullptr);

private slots:
    void onSave();
    void onRestoreDefault() override;

private:
    void setupContent();

    AllConfig::Seam& m_seamRef;
    QComboBox* m_seamFindTypeCombo;
    QCheckBox* m_cropCheck;
    QComboBox* m_blenderTypeCombo;
    QSpinBox* m_blendStrengthSpin;
};

#endif // SEAMDIALOG_H
