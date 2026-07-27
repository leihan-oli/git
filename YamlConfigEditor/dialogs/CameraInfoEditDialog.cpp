#include "CameraInfoEditDialog.h"
#include <QVBoxLayout>
#include <QFormLayout>
#include <QDialogButtonBox>
#include <QLineEdit>
#include <QSpinBox>
#include <QCheckBox>
#include <QComboBox>
#include <QLabel>
#include <QSizePolicy>

CameraInfoEditDialog::CameraInfoEditDialog(const CameraInfo& info, bool isNew, QWidget* parent)
    : QDialog(parent), m_info(info), m_isNew(isNew) {
    setWindowTitle(isNew ? "添加相机" : "编辑相机");
    setMinimumSize(760, 540);
    resize(860, 560);
    setStyleSheet(R"(
        QDialog { background: #f3f6f8; font-size: 24px; }
        QLabel { font-size: 24px; color: #1f2d3d; }
        QLineEdit, QComboBox, QSpinBox { font-size: 24px; min-height: 48px; padding: 5px 10px; border: 2px solid #aebdcc; border-radius: 10px; background: white; }
        QCheckBox { font-size: 24px; }
        QPushButton { font-size: 24px; min-width: 140px; min-height: 54px; }
    )");

    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(28, 22, 28, 22);
    mainLayout->setSpacing(16);
    QFormLayout* form = new QFormLayout;
    form->setHorizontalSpacing(28);
    form->setVerticalSpacing(10);
    form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);

    m_indexSpin = new QSpinBox;
    m_indexSpin->setRange(0, 999);
    m_indexSpin->setValue(m_info.video_index);
    form->addRow("视频索引:", m_indexSpin);

    m_descEdit = new QLineEdit(QString::fromStdString(m_info.description));
    form->addRow("描述:", m_descEdit);

    m_typeCombo = new QComboBox;
    m_typeCombo->addItems({"camera", "video", "rtsp"});
    m_typeCombo->setCurrentText(QString::fromStdString(m_info.type));
    form->addRow("类型:", m_typeCombo);

    m_urlEdit = new QLineEdit(QString::fromStdString(m_info.url));
    form->addRow("URL:", m_urlEdit);

    m_widthSpin = new QSpinBox;
    m_widthSpin->setRange(1, 4096);
    m_widthSpin->setValue(m_info.width);
    form->addRow("宽度:", m_widthSpin);

    m_heightSpin = new QSpinBox;
    m_heightSpin->setRange(1, 4096);
    m_heightSpin->setValue(m_info.height);
    form->addRow("高度:", m_heightSpin);

    m_undistortCheck = new QCheckBox;
    m_undistortCheck->setChecked(m_info.undistort);
    form->addRow("去畸变:", m_undistortCheck);

    m_modelCombo = new QComboBox;
    m_modelCombo->addItems({"pinhole", "fisheye"});
    m_modelCombo->setCurrentText(QString::fromStdString(m_info.model));
    form->addRow("相机模型:", m_modelCombo);

    m_fpsSpin = new QSpinBox;
    m_fpsSpin->setRange(1, 240);
    m_fpsSpin->setValue(m_info.fps);
    form->addRow("帧率:", m_fpsSpin);

    mainLayout->addLayout(form);

    QDialogButtonBox* buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    mainLayout->addWidget(buttonBox);
}

CameraInfo CameraInfoEditDialog::getCameraInfo() const {
    CameraInfo info;
    info.video_index = m_indexSpin->value();
    info.description = m_descEdit->text().toStdString();
    info.type = m_typeCombo->currentText().toStdString();
    info.url = m_urlEdit->text().toStdString();
    info.width = m_widthSpin->value();
    info.height = m_heightSpin->value();
    info.undistort = m_undistortCheck->isChecked();
    info.model = m_modelCombo->currentText().toStdString();
    info.fps = m_fpsSpin->value();
    return info;
}
