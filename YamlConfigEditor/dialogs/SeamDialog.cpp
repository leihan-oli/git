#include "SeamDialog.h"
#include <QVBoxLayout>
#include <QFormLayout>
#include <QComboBox>
#include <QCheckBox>
#include <QSpinBox>
#include <QPushButton>

SeamDialog::SeamDialog(AllConfig::Seam& seamRef, QWidget *parent)
    : FullScreenDialog(parent, "接缝查找与融合设置")
    , m_seamRef(seamRef)
{
    setupContent();
}

void SeamDialog::setupContent()
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

    m_seamFindTypeCombo = new QComboBox;
    m_seamFindTypeCombo->addItems({"gc_color", "gc_graph", "dp_color", "dp_grad", "dp_colorgrad", "voronoi"});
    int idx = m_seamFindTypeCombo->findText(QString::fromStdString(m_seamRef.seam_find_type));
    if (idx >= 0) m_seamFindTypeCombo->setCurrentIndex(idx);
    formLayout->addRow("接缝查找类型:", m_seamFindTypeCombo);

    m_cropCheck = new QCheckBox;
    m_cropCheck->setChecked(m_seamRef.crop);
    formLayout->addRow("裁剪图像 (crop):", m_cropCheck);

    m_blenderTypeCombo = new QComboBox;
    m_blenderTypeCombo->addItems({"Blender::FEATHER", "Blender::MULTI_BAND", "Blender::NO"});
    idx = m_blenderTypeCombo->findText(QString::fromStdString(m_seamRef.blender_type));
    if (idx >= 0) m_blenderTypeCombo->setCurrentIndex(idx);
    formLayout->addRow("融合器类型:", m_blenderTypeCombo);

    m_blendStrengthSpin = new QSpinBox;
    m_blendStrengthSpin->setRange(0, 100);
    m_blendStrengthSpin->setValue(m_seamRef.blend_strength);
    formLayout->addRow("融合强度:", m_blendStrengthSpin);

    layout->addStretch(1);
    layout->addLayout(formLayout);
    layout->addStretch(1);

    QList<QPushButton*> btns = findChildren<QPushButton*>();
    for (auto btn : btns) {
        if (btn->text() == "✓ 确定") connect(btn, &QPushButton::clicked, this, &SeamDialog::onSave);
        else if (btn->text() == "✗ 取消") connect(btn, &QPushButton::clicked, this, &QDialog::reject);
        else if (btn->text() == "⟳ 恢复默认") connect(btn, &QPushButton::clicked, this, &SeamDialog::onRestoreDefault);
    }
}

void SeamDialog::onSave()
{
    m_seamRef.seam_find_type = m_seamFindTypeCombo->currentText().toStdString();
    m_seamRef.crop = m_cropCheck->isChecked();
    m_seamRef.blender_type = m_blenderTypeCombo->currentText().toStdString();
    m_seamRef.blend_strength = m_blendStrengthSpin->value();
    accept();
}

void SeamDialog::onRestoreDefault()
{
    AllConfig::Seam defaults;
    int idx = m_seamFindTypeCombo->findText(QString::fromStdString(defaults.seam_find_type));
    if (idx >= 0) m_seamFindTypeCombo->setCurrentIndex(idx);
    m_cropCheck->setChecked(defaults.crop);
    idx = m_blenderTypeCombo->findText(QString::fromStdString(defaults.blender_type));
    if (idx >= 0) m_blenderTypeCombo->setCurrentIndex(idx);
    m_blendStrengthSpin->setValue(defaults.blend_strength);
}
