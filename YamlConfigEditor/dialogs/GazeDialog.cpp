#include "GazeDialog.h"
#include <QVBoxLayout>
#include <QFormLayout>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QComboBox>
#include <QLineEdit>
#include <QGroupBox>
#include <QPushButton>

GazeDialog::GazeDialog(AllConfig::Gaze& gazeRef, AllConfig::Saliency& saliencyRef, QWidget *parent)
    : FullScreenDialog(parent, "注视感知与显著性融合设置")
    , m_gazeRef(gazeRef)
    , m_saliencyRef(saliencyRef)
{
    setupContent();
}

void GazeDialog::setupContent()
{
    QWidget *contentArea = findChild<QWidget*>("contentArea");
    if (!contentArea) return;

    QVBoxLayout *layout = new QVBoxLayout(contentArea);
    layout->setContentsMargins(24, 8, 24, 8);
    layout->setSpacing(8);

    // ---- 注视参数分组 ----
    QGroupBox *gazeGroup = new QGroupBox("注视感知避让参数");
    QFormLayout *gazeForm = new QFormLayout(gazeGroup);
    gazeForm->setLabelAlignment(Qt::AlignRight);
    gazeForm->setHorizontalSpacing(28);
    gazeForm->setVerticalSpacing(8);
    gazeForm->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);

    m_sigmaRatioSpin = new QDoubleSpinBox;
    m_sigmaRatioSpin->setRange(0.0, 1.0);
    m_sigmaRatioSpin->setSingleStep(0.01);
    m_sigmaRatioSpin->setDecimals(3);
    m_sigmaRatioSpin->setValue(m_gazeRef.gaze_sigma_ratio);
    gazeForm->addRow("Sigma Ratio (gaze_sigma_ratio):", m_sigmaRatioSpin);

    m_alphaSpin = new QDoubleSpinBox;
    m_alphaSpin->setRange(0.0, 10.0);
    m_alphaSpin->setSingleStep(0.1);
    m_alphaSpin->setDecimals(2);
    m_alphaSpin->setValue(m_gazeRef.gaze_alpha);
    gazeForm->addRow("Alpha (gaze_alpha):", m_alphaSpin);

    layout->addWidget(gazeGroup);

    // ---- 显著性融合参数分组 ----
    QGroupBox *saliencyGroup = new QGroupBox("显著性融合参数");
    QFormLayout *saliencyForm = new QFormLayout(saliencyGroup);
    saliencyForm->setLabelAlignment(Qt::AlignRight);
    saliencyForm->setHorizontalSpacing(28);
    saliencyForm->setVerticalSpacing(8);
    saliencyForm->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);

    m_useGazeCheck = new QCheckBox;
    m_useGazeCheck->setChecked(m_saliencyRef.use_gaze);
    saliencyForm->addRow("启用眼动注视 (use_gaze):", m_useGazeCheck);

    m_useU2NetCheck = new QCheckBox;
    m_useU2NetCheck->setChecked(m_saliencyRef.use_u2net);
    saliencyForm->addRow("启用 U2-Net (use_u2net):", m_useU2NetCheck);

    m_fusionModeCombo = new QComboBox;
    m_fusionModeCombo->addItems({"max", "weighted", "mul"});
    m_fusionModeCombo->setCurrentText(QString::fromStdString(m_saliencyRef.fusion_mode));
    saliencyForm->addRow("融合模式 (fusion_mode):", m_fusionModeCombo);

    m_gazeWeightSpin = new QDoubleSpinBox;
    m_gazeWeightSpin->setRange(0.0, 1.0);
    m_gazeWeightSpin->setSingleStep(0.05);
    m_gazeWeightSpin->setDecimals(2);
    m_gazeWeightSpin->setValue(m_saliencyRef.gaze_weight);
    saliencyForm->addRow("眼动权重 (gaze_weight):", m_gazeWeightSpin);

    m_u2netWeightSpin = new QDoubleSpinBox;
    m_u2netWeightSpin->setRange(0.0, 1.0);
    m_u2netWeightSpin->setSingleStep(0.05);
    m_u2netWeightSpin->setDecimals(2);
    m_u2netWeightSpin->setValue(m_saliencyRef.u2net_weight);
    saliencyForm->addRow("U2-Net 权重 (u2net_weight):", m_u2netWeightSpin);

    m_u2netDirEdit = new QLineEdit(QString::fromStdString(m_saliencyRef.u2net_dir));
    saliencyForm->addRow("U2-Net 输出目录 (u2net_dir):", m_u2netDirEdit);

    m_displayModeCombo = new QComboBox;
    m_displayModeCombo->addItems({"combined", "separate"});
    m_displayModeCombo->setCurrentText(QString::fromStdString(m_saliencyRef.display_mode));
    saliencyForm->addRow("显示模式 (display_mode):", m_displayModeCombo);

    layout->addWidget(saliencyGroup);
    layout->addStretch();

    // 连接底部按钮
    QList<QPushButton*> btns = findChildren<QPushButton*>();
    for (auto btn : btns) {
        if (btn->text() == "✓ 确定") connect(btn, &QPushButton::clicked, this, &GazeDialog::onSave);
        else if (btn->text() == "✗ 取消") connect(btn, &QPushButton::clicked, this, &QDialog::reject);
        else if (btn->text() == "⟳ 恢复默认") connect(btn, &QPushButton::clicked, this, &GazeDialog::onRestoreDefault);
    }
}

void GazeDialog::onSave()
{
    // 保存注视参数
    m_gazeRef.gaze_sigma_ratio = m_sigmaRatioSpin->value();
    m_gazeRef.gaze_alpha = m_alphaSpin->value();

    // 保存显著性参数
    m_saliencyRef.use_gaze = m_useGazeCheck->isChecked();
    m_saliencyRef.use_u2net = m_useU2NetCheck->isChecked();
    m_saliencyRef.fusion_mode = m_fusionModeCombo->currentText().toStdString();
    m_saliencyRef.gaze_weight = m_gazeWeightSpin->value();
    m_saliencyRef.u2net_weight = m_u2netWeightSpin->value();
    m_saliencyRef.u2net_dir = m_u2netDirEdit->text().toStdString();
    m_saliencyRef.display_mode = m_displayModeCombo->currentText().toStdString();

    accept();
}

void GazeDialog::onRestoreDefault()
{
    // 注视默认值
    AllConfig::Gaze gazeDefaults;
    m_sigmaRatioSpin->setValue(gazeDefaults.gaze_sigma_ratio);
    m_alphaSpin->setValue(gazeDefaults.gaze_alpha);

    // 显著性默认值
    AllConfig::Saliency saliencyDefaults;
    m_useGazeCheck->setChecked(saliencyDefaults.use_gaze);
    m_useU2NetCheck->setChecked(saliencyDefaults.use_u2net);
    m_fusionModeCombo->setCurrentText(QString::fromStdString(saliencyDefaults.fusion_mode));
    m_gazeWeightSpin->setValue(saliencyDefaults.gaze_weight);
    m_u2netWeightSpin->setValue(saliencyDefaults.u2net_weight);
    m_u2netDirEdit->setText(QString::fromStdString(saliencyDefaults.u2net_dir));
    m_displayModeCombo->setCurrentText(QString::fromStdString(saliencyDefaults.display_mode));
}
