#include "ExposureDialog.h"
#include <QVBoxLayout>
#include <QFormLayout>
#include <QComboBox>
#include <QSpinBox>
#include <QPushButton>

ExposureDialog::ExposureDialog(AllConfig::Exposure& exposureRef, QWidget *parent)
    : FullScreenDialog(parent, "曝光补偿设置")
    , m_exposureRef(exposureRef)
{
    setupContent();
}

void ExposureDialog::setupContent()
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

    m_expTypeCombo = new QComboBox;
    m_expTypeCombo->addItems({"gain_blocks", "gain", "channels", "no", "GAIN_BLOCKS", "GAIN", "CHANNELS", "NO"});
    int idx = m_expTypeCombo->findText(QString::fromStdString(m_exposureRef.exp_type));
    if (idx >= 0) m_expTypeCombo->setCurrentIndex(idx);
    formLayout->addRow("曝光补偿类型:", m_expTypeCombo);

    m_expNrFeedsSpin = new QSpinBox;
    m_expNrFeedsSpin->setRange(1, 10);
    m_expNrFeedsSpin->setValue(m_exposureRef.exp_nr_feeds);
    formLayout->addRow("馈送数量 (nr_feeds):", m_expNrFeedsSpin);

    m_expNrFilteringSpin = new QSpinBox;
    m_expNrFilteringSpin->setRange(0, 10);
    m_expNrFilteringSpin->setValue(m_exposureRef.exp_nr_filtering);
    formLayout->addRow("滤波次数 (nr_filtering):", m_expNrFilteringSpin);

    m_expBlockSizeSpin = new QSpinBox;
    m_expBlockSizeSpin->setRange(8, 256);
    m_expBlockSizeSpin->setSingleStep(8);
    m_expBlockSizeSpin->setValue(m_exposureRef.exp_block_size);
    formLayout->addRow("块大小 (block_size):", m_expBlockSizeSpin);

    layout->addStretch(1);
    layout->addLayout(formLayout);
    layout->addStretch(1);

    QList<QPushButton*> btns = findChildren<QPushButton*>();
    for (auto btn : btns) {
        if (btn->text() == "✓ 确定") connect(btn, &QPushButton::clicked, this, &ExposureDialog::onSave);
        else if (btn->text() == "✗ 取消") connect(btn, &QPushButton::clicked, this, &QDialog::reject);
        else if (btn->text() == "⟳ 恢复默认") connect(btn, &QPushButton::clicked, this, &ExposureDialog::onRestoreDefault);
    }
}

void ExposureDialog::onSave()
{
    m_exposureRef.exp_type = m_expTypeCombo->currentText().toStdString();
    m_exposureRef.exp_nr_feeds = m_expNrFeedsSpin->value();
    m_exposureRef.exp_nr_filtering = m_expNrFilteringSpin->value();
    m_exposureRef.exp_block_size = m_expBlockSizeSpin->value();
    accept();
}

void ExposureDialog::onRestoreDefault()
{
    AllConfig::Exposure defaults;
    int idx = m_expTypeCombo->findText(QString::fromStdString(defaults.exp_type));
    if (idx >= 0) m_expTypeCombo->setCurrentIndex(idx);
    m_expNrFeedsSpin->setValue(defaults.exp_nr_feeds);
    m_expNrFilteringSpin->setValue(defaults.exp_nr_filtering);
    m_expBlockSizeSpin->setValue(defaults.exp_block_size);
}
