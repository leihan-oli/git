#ifndef GAZEDIALOG_H
#define GAZEDIALOG_H

#include "FullScreenDialog.h"
#include "AllConfig.h"

class QDoubleSpinBox;
class QCheckBox;
class QComboBox;
class QLineEdit;

class GazeDialog : public FullScreenDialog
{
    Q_OBJECT
public:
    // 同时传入 Gaze 和 Saliency 引用
    explicit GazeDialog(AllConfig::Gaze& gazeRef, AllConfig::Saliency& saliencyRef, QWidget *parent = nullptr);

private slots:
    void onSave();
    void onRestoreDefault() override;

private:
    void setupContent();

    AllConfig::Gaze&      m_gazeRef;
    AllConfig::Saliency&  m_saliencyRef;

    // 注视参数控件
    QDoubleSpinBox* m_sigmaRatioSpin;
    QDoubleSpinBox* m_alphaSpin;

    // 显著性参数控件
    QCheckBox*      m_useGazeCheck;
    QCheckBox*      m_useU2NetCheck;
    QComboBox*      m_fusionModeCombo;
    QDoubleSpinBox* m_gazeWeightSpin;
    QDoubleSpinBox* m_u2netWeightSpin;
    QLineEdit*      m_u2netDirEdit;
    QComboBox*      m_displayModeCombo;
};

#endif // GAZEDIALOG_H
