#include "CameraParamsDialog.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QHeaderView>
#include <QPushButton>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QDialogButtonBox>
#include <QLabel>
#include <QMessageBox>

// ---------------------- CameraParamEditDialog ----------------------
CameraParamEditDialog::CameraParamEditDialog(CameraParams& params, QWidget *parent)
    : QDialog(parent), m_params(params)
{
    setWindowTitle("编辑相机参数");
    setMinimumSize(1160, 560);
    resize(1180, 570);
    setStyleSheet(R"(
        QDialog { background: #f3f6f8; font-size: 22px; }
        QLabel { font-size: 22px; color: #1f2d3d; }
        QLineEdit, QDoubleSpinBox { font-size: 22px; min-height: 42px; padding: 4px 8px; border: 2px solid #aebdcc; border-radius: 8px; background: white; }
        QPushButton { font-size: 22px; min-width: 130px; min-height: 50px; }
    )");
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(18, 14, 18, 14);
    mainLayout->setSpacing(12);
    QFormLayout* form = new QFormLayout;
    form->setHorizontalSpacing(20);
    form->setVerticalSpacing(8);
    form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);

    // 内参矩阵 K (3x3)
    QHBoxLayout* kLayout = new QHBoxLayout;
    for (int i = 0; i < 9; ++i) {
        m_kEdit[i] = new QLineEdit;
        m_kEdit[i]->setText(QString::number(m_params.K[i]));
        kLayout->addWidget(m_kEdit[i]);
        if ((i+1) % 3 == 0 && i != 8) {
            kLayout->addWidget(new QLabel("<br>"));
        }
    }
    form->addRow("内参矩阵 K (3x3):", kLayout);

    // 旋转矩阵 R (3x3)
    QHBoxLayout* rLayout = new QHBoxLayout;
    for (int i = 0; i < 9; ++i) {
        m_rEdit[i] = new QLineEdit;
        m_rEdit[i]->setText(QString::number(m_params.R[i]));
        rLayout->addWidget(m_rEdit[i]);
        if ((i+1) % 3 == 0 && i != 8) {
            rLayout->addWidget(new QLabel("<br>"));
        }
    }
    form->addRow("旋转矩阵 R (3x3):", rLayout);

    // 平移向量 T (3)
    QHBoxLayout* tLayout = new QHBoxLayout;
    for (int i = 0; i < 3; ++i) {
        m_tEdit[i] = new QLineEdit;
        m_tEdit[i]->setText(QString::number(m_params.T[i]));
        tLayout->addWidget(m_tEdit[i]);
    }
    form->addRow("平移向量 T (x,y,z):", tLayout);

    // 畸变系数 D (5)
    QHBoxLayout* dLayout = new QHBoxLayout;
    for (int i = 0; i < 5; ++i) {
        m_dEdit[i] = new QLineEdit;
        m_dEdit[i]->setText(QString::number(m_params.D[i]));
        dLayout->addWidget(m_dEdit[i]);
    }
    form->addRow("畸变系数 D (k1,k2,p1,p2,k3):", dLayout);

    m_ppxSpin = new QDoubleSpinBox;
    m_ppxSpin->setRange(-10000, 10000);
    m_ppxSpin->setDecimals(3);
    m_ppxSpin->setValue(m_params.ppx);
    form->addRow("主点 x (ppx):", m_ppxSpin);

    m_ppySpin = new QDoubleSpinBox;
    m_ppySpin->setRange(-10000, 10000);
    m_ppySpin->setDecimals(3);
    m_ppySpin->setValue(m_params.ppy);
    form->addRow("主点 y (ppy):", m_ppySpin);

    m_focalSpin = new QDoubleSpinBox;
    m_focalSpin->setRange(0, 100000);
    m_focalSpin->setDecimals(3);
    m_focalSpin->setValue(m_params.focal);
    form->addRow("焦距 (focal):", m_focalSpin);

    m_aspectSpin = new QDoubleSpinBox;
    m_aspectSpin->setRange(0, 10);
    m_aspectSpin->setDecimals(3);
    m_aspectSpin->setValue(m_params.aspect);
    form->addRow("宽高比 (aspect):", m_aspectSpin);

    mainLayout->addLayout(form);

    QDialogButtonBox* buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &CameraParamEditDialog::onSave);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    mainLayout->addWidget(buttonBox);
}

void CameraParamEditDialog::onSave()
{
    for (int i = 0; i < 9; ++i) {
        bool ok;
        double val = m_kEdit[i]->text().toDouble(&ok);
        m_params.K[i] = ok ? val : 0.0;
    }
    for (int i = 0; i < 9; ++i) {
        bool ok;
        double val = m_rEdit[i]->text().toDouble(&ok);
        m_params.R[i] = ok ? val : 0.0;
    }
    for (int i = 0; i < 3; ++i) {
        bool ok;
        double val = m_tEdit[i]->text().toDouble(&ok);
        m_params.T[i] = ok ? val : 0.0;
    }
    for (int i = 0; i < 5; ++i) {
        bool ok;
        double val = m_dEdit[i]->text().toDouble(&ok);
        m_params.D[i] = ok ? val : 0.0;
    }
    m_params.ppx = m_ppxSpin->value();
    m_params.ppy = m_ppySpin->value();
    m_params.focal = m_focalSpin->value();
    m_params.aspect = m_aspectSpin->value();
    accept();
}

// ---------------------- CameraParamsDialog ----------------------
CameraParamsDialog::CameraParamsDialog(std::vector<CameraParams>& paramsList, QWidget *parent)
    : FullScreenDialog(parent, "相机内参参数")
    , m_paramsList(paramsList)
{
    setupContent();
}

void CameraParamsDialog::setupContent()
{
    QWidget *contentArea = findChild<QWidget*>("contentArea");
    if (!contentArea) return;

    QVBoxLayout *layout = new QVBoxLayout(contentArea);
    layout->setContentsMargins(22, 12, 22, 12);

    m_table = new QTableWidget;
    m_table->setColumnCount(3);
    m_table->setHorizontalHeaderLabels({"相机索引", "焦距", "主点 (x,y)"});
    m_table->horizontalHeader()->setStretchLastSection(true);
    m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_table->verticalHeader()->setDefaultSectionSize(58);
    m_table->horizontalHeader()->setMinimumHeight(54);
    layout->addWidget(m_table);

    QHBoxLayout *btnLayout = new QHBoxLayout;
    QPushButton *addBtn = new QPushButton("➕ 添加");
    QPushButton *editBtn = new QPushButton("✏️ 编辑");
    QPushButton *deleteBtn = new QPushButton("🗑️ 删除");
    btnLayout->addWidget(addBtn);
    btnLayout->addWidget(editBtn);
    btnLayout->addWidget(deleteBtn);
    btnLayout->addStretch();
    layout->addLayout(btnLayout);

    connect(addBtn, &QPushButton::clicked, this, &CameraParamsDialog::onAdd);
    connect(editBtn, &QPushButton::clicked, this, &CameraParamsDialog::onEdit);
    connect(deleteBtn, &QPushButton::clicked, this, &CameraParamsDialog::onDelete);

    updateTable();

    QList<QPushButton*> btns = findChildren<QPushButton*>();
    for (auto btn : btns) {
        if (btn->text() == "✓ 确定") connect(btn, &QPushButton::clicked, this, &CameraParamsDialog::onSave);
        else if (btn->text() == "✗ 取消") connect(btn, &QPushButton::clicked, this, &QDialog::reject);
        else if (btn->text() == "⟳ 恢复默认") connect(btn, &QPushButton::clicked, this, &CameraParamsDialog::onRestoreDefault);
    }
}

void CameraParamsDialog::updateTable()
{
    m_table->setRowCount(static_cast<int>(m_paramsList.size()));
    for (size_t i = 0; i < m_paramsList.size(); ++i) {
        const auto& p = m_paramsList[i];
        QTableWidgetItem* idxItem = new QTableWidgetItem(QString::number(i));
        idxItem->setFlags(idxItem->flags() & ~Qt::ItemIsEditable);
        m_table->setItem(static_cast<int>(i), 0, idxItem);

        QTableWidgetItem* focalItem = new QTableWidgetItem(QString::number(p.focal));
        focalItem->setFlags(focalItem->flags() & ~Qt::ItemIsEditable);
        m_table->setItem(static_cast<int>(i), 1, focalItem);

        QTableWidgetItem* ppItem = new QTableWidgetItem(QString("(%1, %2)").arg(p.ppx).arg(p.ppy));
        ppItem->setFlags(ppItem->flags() & ~Qt::ItemIsEditable);
        m_table->setItem(static_cast<int>(i), 2, ppItem);
    }
    m_table->resizeColumnsToContents();
}

void CameraParamsDialog::onAdd()
{
    CameraParams newParams;
    AllConfig defaults;
    defaults.setDefaults();
    if (!defaults.cameraParamsList.empty()) {
        newParams = defaults.cameraParamsList.front();
    }
    CameraParamEditDialog dlg(newParams, this);
    if (dlg.exec() == QDialog::Accepted) {
        m_paramsList.push_back(newParams);
        updateTable();
    }
}

void CameraParamsDialog::onEdit()
{
    int row = m_table->currentRow();
    if (row < 0 || row >= static_cast<int>(m_paramsList.size())) {
        QMessageBox::warning(this, "未选中", "请先选中要编辑的相机参数行。");
        return;
    }
    CameraParamEditDialog dlg(m_paramsList[row], this);
    if (dlg.exec() == QDialog::Accepted) {
        updateTable();
    }
}

void CameraParamsDialog::onDelete()
{
    int row = m_table->currentRow();
    if (row < 0 || row >= static_cast<int>(m_paramsList.size())) {
        QMessageBox::warning(this, "未选中", "请先选中要删除的相机参数行。");
        return;
    }
    if (QMessageBox::question(this, "确认删除", "确定删除该相机参数吗？",
                              QMessageBox::Yes | QMessageBox::No) == QMessageBox::Yes) {
        m_paramsList.erase(m_paramsList.begin() + row);
        updateTable();
    }
}

void CameraParamsDialog::onSave()
{
    accept();
}

void CameraParamsDialog::onRestoreDefault()
{
    AllConfig defaults;
    defaults.setDefaults();
    m_paramsList = defaults.cameraParamsList;
    updateTable();
}
