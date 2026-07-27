#include "CameraAdjustDialog.h"
#include <QVBoxLayout>
#include <QFormLayout>
#include <QComboBox>
#include <QPushButton>
#include <QSpinBox>

CameraAdjustDialog::CameraAdjustDialog(AllConfig::CameraAdjust& adjustRef, QWidget *parent)
    : FullScreenDialog(parent, "相机位姿估计与校正")
    , m_adjustRef(adjustRef)
{
    setupContent();
}

void CameraAdjustDialog::setupContent()
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

    m_adjusterTypeCombo = new QComboBox;
    m_adjusterTypeCombo->addItems({"ray", "bundle", "no"});
    m_adjusterTypeCombo->setCurrentText(QString::fromStdString(m_adjustRef.adjuster_type));
    formLayout->addRow("调整器类型 (Adjuster Type):", m_adjusterTypeCombo);

    m_waveCorrectionCombo = new QComboBox;
    m_waveCorrectionCombo->addItems({"horiz", "vert", "no"});
    m_waveCorrectionCombo->setCurrentText(QString::fromStdString(m_adjustRef.wave_correction));
    formLayout->addRow("波浪校正 (Wave Correction):", m_waveCorrectionCombo);

    m_warperTypeCombo = new QComboBox;
    m_warperTypeCombo->addItems({"spherical", "plane", "cylindrical", "fisheye", "stereographic", "compressedPlaneA2B1"});
    m_warperTypeCombo->setCurrentText(QString::fromStdString(m_adjustRef.warper_type));
    formLayout->addRow("投影类型 (Warper Type):", m_warperTypeCombo);

    m_estimatorTypeCombo = new QComboBox;
    m_estimatorTypeCombo->addItems({"homography", "affine", "no"});
    m_estimatorTypeCombo->setCurrentText(QString::fromStdString(m_adjustRef.estimator_type));
    formLayout->addRow("估计器类型 (Estimator Type):", m_estimatorTypeCombo);

    m_alignDepthSpin = new QSpinBox;
    m_alignDepthSpin->setRange(-1, 9999);
    m_alignDepthSpin->setSpecialValueText("自动(-1)");
    m_alignDepthSpin->setValue(m_adjustRef.align_depth);
    formLayout->addRow("对齐深度 (align_depth):", m_alignDepthSpin);

    layout->addStretch(1);
    layout->addLayout(formLayout);
    layout->addStretch(1);

    QList<QPushButton*> btns = findChildren<QPushButton*>();
    for (auto btn : btns) {
        if (btn->text() == "✓ 确定") connect(btn, &QPushButton::clicked, this, &CameraAdjustDialog::onSave);
        else if (btn->text() == "✗ 取消") connect(btn, &QPushButton::clicked, this, &QDialog::reject);
        else if (btn->text() == "⟳ 恢复默认") connect(btn, &QPushButton::clicked, this, &CameraAdjustDialog::onRestoreDefault);
    }
}

void CameraAdjustDialog::onSave()
{
    m_adjustRef.adjuster_type = m_adjusterTypeCombo->currentText().toStdString();
    m_adjustRef.wave_correction = m_waveCorrectionCombo->currentText().toStdString();
    m_adjustRef.warper_type = m_warperTypeCombo->currentText().toStdString();
    m_adjustRef.estimator_type = m_estimatorTypeCombo->currentText().toStdString();
    m_adjustRef.align_depth = m_alignDepthSpin->value();
    accept();
}

void CameraAdjustDialog::onRestoreDefault()
{
    AllConfig::CameraAdjust defaults;
    m_adjusterTypeCombo->setCurrentText(QString::fromStdString(defaults.adjuster_type));
    m_waveCorrectionCombo->setCurrentText(QString::fromStdString(defaults.wave_correction));
    m_warperTypeCombo->setCurrentText(QString::fromStdString(defaults.warper_type));
    m_estimatorTypeCombo->setCurrentText(QString::fromStdString(defaults.estimator_type));
    m_alignDepthSpin->setValue(defaults.align_depth);
}
