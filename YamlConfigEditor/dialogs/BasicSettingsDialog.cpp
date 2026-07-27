#include "BasicSettingsDialog.h"
#include <QVBoxLayout>
#include <QFormLayout>
#include <QCheckBox>
#include <QSpinBox>
#include <QDialogButtonBox>
#include <QPushButton>
#include <QScrollArea>

BasicSettingsDialog::BasicSettingsDialog(AllConfig::Basic& basicRef, QWidget *parent)
    : FullScreenDialog(parent, "基础设置")
    , m_basicRef(basicRef)
{
    setupContent();
}

BasicSettingsDialog::~BasicSettingsDialog() = default;

void BasicSettingsDialog::setupContent()
{
    // 获取内容区域（基类中已创建）
    QWidget *contentArea = findChild<QWidget*>("contentArea");
    if (!contentArea) return;

    QVBoxLayout *layout = new QVBoxLayout(contentArea);
    layout->setContentsMargins(30, 12, 30, 12);

    QFormLayout *formLayout = new QFormLayout;
    formLayout->setLabelAlignment(Qt::AlignRight);
    formLayout->setHorizontalSpacing(28);
    formLayout->setVerticalSpacing(10);
    formLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);

    m_previewCheck = new QCheckBox;
    m_cudaCheck = new QCheckBox;
    m_logLevelSpin = new QSpinBox;
    m_logLevelSpin->setRange(0, 5);
    m_verboseCheck = new QCheckBox;
    m_cameraCountSpin = new QSpinBox;
    m_cameraCountSpin->setRange(1, 10);
    m_outputWidthSpin = new QSpinBox;
    m_outputWidthSpin->setRange(1, 4000);
    m_outputHeightSpin = new QSpinBox;
    m_outputHeightSpin->setRange(1, 4000);

    formLayout->addRow("预览 (Preview):", m_previewCheck);
    formLayout->addRow("尝试 CUDA:", m_cudaCheck);
    formLayout->addRow("日志级别 (0~5):", m_logLevelSpin);
    formLayout->addRow("详细输出:", m_verboseCheck);
    formLayout->addRow("相机数量:", m_cameraCountSpin);
    formLayout->addRow("输出宽度:", m_outputWidthSpin);
    formLayout->addRow("输出高度:", m_outputHeightSpin);

    layout->addStretch(1);
    layout->addLayout(formLayout);
    layout->addStretch(1);

    // 加载当前值
    m_previewCheck->setChecked(m_basicRef.preview);
    m_cudaCheck->setChecked(m_basicRef.try_cuda);
    m_logLevelSpin->setValue(m_basicRef.log_level);
    m_verboseCheck->setChecked(m_basicRef.verbose_output);
    m_cameraCountSpin->setValue(m_basicRef.camera_count);
    m_outputWidthSpin->setValue(m_basicRef.output_width);
    m_outputHeightSpin->setValue(m_basicRef.output_height);

    // 连接确定、取消、恢复默认按钮
    QPushButton *okBtn = findChild<QPushButton*>(QString(), Qt::FindDirectChildrenOnly);
    // 通过文本查找：基类中确定按钮文本为 "✓ 确定"
    QList<QPushButton*> btns = findChildren<QPushButton*>();
    for (auto btn : btns) {
        if (btn->text() == "✓ 确定") connect(btn, &QPushButton::clicked, this, &BasicSettingsDialog::onSave);
        else if (btn->text() == "✗ 取消") connect(btn, &QPushButton::clicked, this, &QDialog::reject);
        else if (btn->text() == "⟳ 恢复默认") connect(btn, &QPushButton::clicked, this, &BasicSettingsDialog::onRestoreDefault);
    }
}

void BasicSettingsDialog::onSave()
{
    m_basicRef.preview = m_previewCheck->isChecked();
    m_basicRef.try_cuda = m_cudaCheck->isChecked();
    m_basicRef.log_level = m_logLevelSpin->value();
    m_basicRef.verbose_output = m_verboseCheck->isChecked();
    m_basicRef.camera_count = m_cameraCountSpin->value();
    m_basicRef.output_width = m_outputWidthSpin->value();
    m_basicRef.output_height = m_outputHeightSpin->value();
    accept();
}

void BasicSettingsDialog::onRestoreDefault()
{
    AllConfig::Basic defaults;
    m_previewCheck->setChecked(defaults.preview);
    m_cudaCheck->setChecked(defaults.try_cuda);
    m_logLevelSpin->setValue(defaults.log_level);
    m_verboseCheck->setChecked(defaults.verbose_output);
    m_cameraCountSpin->setValue(defaults.camera_count);
    m_outputWidthSpin->setValue(defaults.output_width);
    m_outputHeightSpin->setValue(defaults.output_height);
}
