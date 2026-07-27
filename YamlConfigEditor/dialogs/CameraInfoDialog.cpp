#include "CameraInfoDialog.h"
#include "CameraInfoEditDialog.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QMessageBox>
#include <QPushButton>
#include <QTableWidget>
CameraInfoDialog::CameraInfoDialog(std::vector<CameraInfo>& cameraList, QWidget *parent)
    : FullScreenDialog(parent, "编辑相机信息")
    , m_cameraList(cameraList)
{
    setupContent();
}

void CameraInfoDialog::setupContent()
{
    QWidget *contentArea = findChild<QWidget*>("contentArea");
    if (!contentArea) return;

    QVBoxLayout *layout = new QVBoxLayout(contentArea);
    layout->setContentsMargins(22, 12, 22, 12);

    m_table = new QTableWidget;
    m_table->setColumnCount(9);
    QStringList headers = {"索引", "描述", "类型", "URL", "宽度", "高度", "去畸变", "模型", "帧率"};
    m_table->setHorizontalHeaderLabels(headers);
    m_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_table->setSelectionMode(QAbstractItemView::SingleSelection);
    m_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_table->verticalHeader()->setDefaultSectionSize(54);
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

    connect(addBtn, &QPushButton::clicked, this, &CameraInfoDialog::onAdd);
    connect(editBtn, &QPushButton::clicked, this, &CameraInfoDialog::onEdit);
    connect(deleteBtn, &QPushButton::clicked, this, &CameraInfoDialog::onDelete);

    refreshTable();

    // 连接基类中的按钮
    QList<QPushButton*> btns = findChildren<QPushButton*>();
    for (auto btn : btns) {
        if (btn->text() == "✓ 确定") connect(btn, &QPushButton::clicked, this, &CameraInfoDialog::onSave);
        else if (btn->text() == "✗ 取消") connect(btn, &QPushButton::clicked, this, &QDialog::reject);
        else if (btn->text() == "⟳ 恢复默认") connect(btn, &QPushButton::clicked, this, &CameraInfoDialog::onRestoreDefault);
    }
}

void CameraInfoDialog::refreshTable()
{
    m_table->clearContents();
    m_table->setRowCount(static_cast<int>(m_cameraList.size()));
    for (size_t i = 0; i < m_cameraList.size(); ++i) {
        const CameraInfo& info = m_cameraList[i];
        m_table->setItem(static_cast<int>(i), 0, new QTableWidgetItem(QString::number(info.video_index)));
        m_table->setItem(static_cast<int>(i), 1, new QTableWidgetItem(QString::fromStdString(info.description)));
        m_table->setItem(static_cast<int>(i), 2, new QTableWidgetItem(QString::fromStdString(info.type)));
        m_table->setItem(static_cast<int>(i), 3, new QTableWidgetItem(QString::fromStdString(info.url)));
        m_table->setItem(static_cast<int>(i), 4, new QTableWidgetItem(QString::number(info.width)));
        m_table->setItem(static_cast<int>(i), 5, new QTableWidgetItem(QString::number(info.height)));
        m_table->setItem(static_cast<int>(i), 6, new QTableWidgetItem(info.undistort ? "是" : "否"));
        m_table->setItem(static_cast<int>(i), 7, new QTableWidgetItem(QString::fromStdString(info.model)));
        m_table->setItem(static_cast<int>(i), 8, new QTableWidgetItem(QString::number(info.fps)));
    }
}

int CameraInfoDialog::getCurrentRow() const
{
    QList<QTableWidgetItem*> selected = m_table->selectedItems();
    if (selected.isEmpty()) return -1;
    return selected.first()->row();
}

void CameraInfoDialog::onAdd()
{
    CameraInfo newInfo;
    int maxIdx = -1;
    for (const auto& info : m_cameraList) {
        if (info.video_index > maxIdx) maxIdx = info.video_index;
    }
    newInfo.video_index = maxIdx + 1;
    newInfo.type = "camera";
    newInfo.url = "";
    newInfo.width = 1280;
    newInfo.height = 720;
    newInfo.undistort = true;
    newInfo.model = "pinhole";
    newInfo.fps = 30;

    CameraInfoEditDialog dlg(newInfo, true, this);
    if (dlg.exec() == QDialog::Accepted) {
        m_cameraList.push_back(dlg.getCameraInfo());
        refreshTable();
    }
}

void CameraInfoDialog::onEdit()
{
    int row = getCurrentRow();
    if (row < 0) {
        QMessageBox::warning(this, "警告", "请先选择一行要编辑的相机。");
        return;
    }
    CameraInfo& info = m_cameraList[static_cast<size_t>(row)];
    CameraInfoEditDialog dlg(info, false, this);
    if (dlg.exec() == QDialog::Accepted) {
        info = dlg.getCameraInfo();
        refreshTable();
    }
}

void CameraInfoDialog::onDelete()
{
    int row = getCurrentRow();
    if (row < 0) {
        QMessageBox::warning(this, "警告", "请先选择要删除的相机。");
        return;
    }
    if (QMessageBox::question(this, "确认删除", "确定要删除选中的相机吗？",
                              QMessageBox::Yes | QMessageBox::No) == QMessageBox::Yes) {
        m_cameraList.erase(m_cameraList.begin() + row);
        refreshTable();
    }
}

void CameraInfoDialog::onSave()
{
    accept();
}

void CameraInfoDialog::onRestoreDefault()
{
    AllConfig defaults;
    defaults.setDefaults();
    m_cameraList = defaults.cameraInfos;
    refreshTable();
}
