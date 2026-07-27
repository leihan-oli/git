#ifndef BASICSETTINGSDIALOG_H
#define BASICSETTINGSDIALOG_H

#include "FullScreenDialog.h"
#include "AllConfig.h"

class QCheckBox;
class QSpinBox;

class BasicSettingsDialog : public FullScreenDialog
{
    Q_OBJECT

public:
    explicit BasicSettingsDialog(AllConfig::Basic& basicRef, QWidget *parent = nullptr);
    ~BasicSettingsDialog();

private slots:
    void onSave();
    void onRestoreDefault() override;

private:
    void setupContent();   // 初始化内容区域控件

    AllConfig::Basic& m_basicRef;

    QCheckBox* m_previewCheck;
    QCheckBox* m_cudaCheck;
    QSpinBox*  m_logLevelSpin;
    QCheckBox* m_verboseCheck;
    QSpinBox*  m_cameraCountSpin;
    QSpinBox*  m_outputWidthSpin;
    QSpinBox*  m_outputHeightSpin;
};

#endif // BASICSETTINGSDIALOG_H
