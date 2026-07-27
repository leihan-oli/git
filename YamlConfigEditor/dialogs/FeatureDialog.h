#ifndef FEATUREDIALOG_H
#define FEATUREDIALOG_H

#include "FullScreenDialog.h"
#include "AllConfig.h"

class QComboBox;
class QDoubleSpinBox;
class QSpinBox;

class FeatureDialog : public FullScreenDialog
{
    Q_OBJECT
public:
    explicit FeatureDialog(AllConfig::Feature& featureRef, QWidget *parent = nullptr);

private slots:
    void onSave();
    void onRestoreDefault() override;

private:
    void setupContent();

    AllConfig::Feature& m_featureRef;
    QComboBox* m_featuresTypeCombo;
    QComboBox* m_matcherTypeCombo;
    QDoubleSpinBox* m_matchConfSpin;
    QSpinBox* m_maxFeaturesSpin;
    QDoubleSpinBox* m_matchingThreshSpin;
    QSpinBox* m_matchRWSpin;
};

#endif // FEATUREDIALOG_H
