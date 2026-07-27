#include "FeatureDialog.h"
#include <QVBoxLayout>
#include <QFormLayout>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QPushButton>

FeatureDialog::FeatureDialog(AllConfig::Feature& featureRef, QWidget *parent)
    : FullScreenDialog(parent, "特征提取与匹配设置")
    , m_featureRef(featureRef)
{
    setupContent();
}

void FeatureDialog::setupContent()
{
    QWidget *contentArea = findChild<QWidget*>("contentArea");
    if (!contentArea) return;

    QVBoxLayout *layout = new QVBoxLayout(contentArea);
    layout->setContentsMargins(30, 12, 30, 12);

    QFormLayout *formLayout = new QFormLayout;
    formLayout->setLabelAlignment(Qt::AlignRight);
    formLayout->setHorizontalSpacing(28);
    formLayout->setVerticalSpacing(10);
    formLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);

    m_featuresTypeCombo = new QComboBox;
    m_featuresTypeCombo->addItems({"surf", "orb", "sift", "akaze", "brisk"});
    int idx = m_featuresTypeCombo->findText(QString::fromStdString(m_featureRef.features_type));
    if (idx >= 0) m_featuresTypeCombo->setCurrentIndex(idx);
    formLayout->addRow("特征类型:", m_featuresTypeCombo);

    m_matcherTypeCombo = new QComboBox;
    m_matcherTypeCombo->addItems({"homography", "affine", "fundamental"});
    idx = m_matcherTypeCombo->findText(QString::fromStdString(m_featureRef.matcher_type));
    if (idx >= 0) m_matcherTypeCombo->setCurrentIndex(idx);
    formLayout->addRow("匹配器类型:", m_matcherTypeCombo);

    m_matchConfSpin = new QDoubleSpinBox;
    m_matchConfSpin->setRange(0.0, 1.0);
    m_matchConfSpin->setSingleStep(0.05);
    m_matchConfSpin->setDecimals(2);
    m_matchConfSpin->setValue(m_featureRef.match_conf);
    formLayout->addRow("匹配置信度:", m_matchConfSpin);

    m_maxFeaturesSpin = new QSpinBox;
    m_maxFeaturesSpin->setRange(1, 10000);
    m_maxFeaturesSpin->setSingleStep(100);
    m_maxFeaturesSpin->setValue(m_featureRef.max_features);
    formLayout->addRow("最大特征数:", m_maxFeaturesSpin);

    m_matchingThreshSpin = new QDoubleSpinBox;
    m_matchingThreshSpin->setRange(0.0, 5.0);
    m_matchingThreshSpin->setSingleStep(0.1);
    m_matchingThreshSpin->setDecimals(2);
    m_matchingThreshSpin->setValue(m_featureRef.matching_thresh);
    formLayout->addRow("匹配阈值:", m_matchingThreshSpin);

    m_matchRWSpin = new QSpinBox;
    m_matchRWSpin->setRange(-1, 10);
    m_matchRWSpin->setSpecialValueText("自动(-1)");
    m_matchRWSpin->setValue(m_featureRef.match_rw);
    formLayout->addRow("匹配重加权:", m_matchRWSpin);

    layout->addStretch(1);
    layout->addLayout(formLayout);
    layout->addStretch(1);

    QList<QPushButton*> btns = findChildren<QPushButton*>();
    for (auto btn : btns) {
        if (btn->text() == "✓ 确定") connect(btn, &QPushButton::clicked, this, &FeatureDialog::onSave);
        else if (btn->text() == "✗ 取消") connect(btn, &QPushButton::clicked, this, &QDialog::reject);
        else if (btn->text() == "⟳ 恢复默认") connect(btn, &QPushButton::clicked, this, &FeatureDialog::onRestoreDefault);
    }
}

void FeatureDialog::onSave()
{
    m_featureRef.features_type = m_featuresTypeCombo->currentText().toStdString();
    m_featureRef.matcher_type = m_matcherTypeCombo->currentText().toStdString();
    m_featureRef.match_conf = m_matchConfSpin->value();
    m_featureRef.max_features = m_maxFeaturesSpin->value();
    m_featureRef.matching_thresh = m_matchingThreshSpin->value();
    m_featureRef.match_rw = m_matchRWSpin->value();
    accept();
}

void FeatureDialog::onRestoreDefault()
{
    AllConfig::Feature defaults;
    int idx = m_featuresTypeCombo->findText(QString::fromStdString(defaults.features_type));
    if (idx >= 0) m_featuresTypeCombo->setCurrentIndex(idx);
    idx = m_matcherTypeCombo->findText(QString::fromStdString(defaults.matcher_type));
    if (idx >= 0) m_matcherTypeCombo->setCurrentIndex(idx);
    m_matchConfSpin->setValue(defaults.match_conf);
    m_maxFeaturesSpin->setValue(defaults.max_features);
    m_matchingThreshSpin->setValue(defaults.matching_thresh);
    m_matchRWSpin->setValue(defaults.match_rw);
}
