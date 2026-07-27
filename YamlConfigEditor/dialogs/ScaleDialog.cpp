#include "ScaleDialog.h"
#include <QVBoxLayout>
#include <QFormLayout>
#include <QDoubleSpinBox>
#include <QPushButton>

ScaleDialog::ScaleDialog(AllConfig::Scale& scaleRef, QWidget *parent)
    : FullScreenDialog(parent, "图像分辨率缩放参数")
    , m_scaleRef(scaleRef)
{
    setupContent();
}

void ScaleDialog::setupContent()
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

    m_inputScaleSpin = new QDoubleSpinBox;
    m_inputScaleSpin->setRange(-1.0, 10.0);
    m_inputScaleSpin->setSingleStep(0.1);
    m_inputScaleSpin->setDecimals(2);
    m_inputScaleSpin->setValue(m_scaleRef.input_scale_megapix);
    formLayout->addRow("输入缩放 (input_scale_megapix):", m_inputScaleSpin);

    m_homoEstScaleSpin = new QDoubleSpinBox;
    m_homoEstScaleSpin->setRange(0.1, 10.0);
    m_homoEstScaleSpin->setSingleStep(0.1);
    m_homoEstScaleSpin->setDecimals(2);
    m_homoEstScaleSpin->setValue(m_scaleRef.homo_est_scale_megapix);
    formLayout->addRow("单应性估计缩放 (homo_est_scale_megapix):", m_homoEstScaleSpin);

    m_expEstScaleSpin = new QDoubleSpinBox;
    m_expEstScaleSpin->setRange(0.1, 10.0);
    m_expEstScaleSpin->setSingleStep(0.1);
    m_expEstScaleSpin->setDecimals(2);
    m_expEstScaleSpin->setValue(m_scaleRef.exp_est_scale_megapix);
    formLayout->addRow("曝光估计缩放 (exp_est_scale_megapix):", m_expEstScaleSpin);

    m_seamFinderScaleSpin = new QDoubleSpinBox;
    m_seamFinderScaleSpin->setRange(0.05, 1.0);
    m_seamFinderScaleSpin->setSingleStep(0.05);
    m_seamFinderScaleSpin->setDecimals(2);
    m_seamFinderScaleSpin->setValue(m_scaleRef.seam_finder_scale_megapix);
    formLayout->addRow("接缝查找缩放 (seam_finder_scale_megapix):", m_seamFinderScaleSpin);

    m_blenderScaleSpin = new QDoubleSpinBox;
    m_blenderScaleSpin->setRange(-1.0, 10.0);
    m_blenderScaleSpin->setSingleStep(0.1);
    m_blenderScaleSpin->setDecimals(2);
    m_blenderScaleSpin->setValue(m_scaleRef.blender_scale_megapix);
    formLayout->addRow("融合缩放 (blender_scale_megapix):", m_blenderScaleSpin);

    layout->addStretch(1);
    layout->addLayout(formLayout);
    layout->addStretch(1);

    // 连接基类中的按钮
    QList<QPushButton*> btns = findChildren<QPushButton*>();
    for (auto btn : btns) {
        if (btn->text() == "✓ 确定") connect(btn, &QPushButton::clicked, this, &ScaleDialog::onSave);
        else if (btn->text() == "✗ 取消") connect(btn, &QPushButton::clicked, this, &QDialog::reject);
        else if (btn->text() == "⟳ 恢复默认") connect(btn, &QPushButton::clicked, this, &ScaleDialog::onRestoreDefault);
    }
}

void ScaleDialog::onSave()
{
    m_scaleRef.input_scale_megapix = m_inputScaleSpin->value();
    m_scaleRef.homo_est_scale_megapix = m_homoEstScaleSpin->value();
    m_scaleRef.exp_est_scale_megapix = m_expEstScaleSpin->value();
    m_scaleRef.seam_finder_scale_megapix = m_seamFinderScaleSpin->value();
    m_scaleRef.blender_scale_megapix = m_blenderScaleSpin->value();
    accept();
}

void ScaleDialog::onRestoreDefault()
{
    AllConfig::Scale defaults;
    m_inputScaleSpin->setValue(defaults.input_scale_megapix);
    m_homoEstScaleSpin->setValue(defaults.homo_est_scale_megapix);
    m_expEstScaleSpin->setValue(defaults.exp_est_scale_megapix);
    m_seamFinderScaleSpin->setValue(defaults.seam_finder_scale_megapix);
    m_blenderScaleSpin->setValue(defaults.blender_scale_megapix);
}
