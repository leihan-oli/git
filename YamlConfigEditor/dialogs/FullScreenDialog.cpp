#include "FullScreenDialog.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QApplication>
#include <QScrollArea>
#include <QFrame>

FullScreenDialog::FullScreenDialog(QWidget *parent, const QString &title)
    : QDialog(nullptr)
{
    Q_UNUSED(parent)

    setupUI(title);
    setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint);
    setAttribute(Qt::WA_TranslucentBackground);
    setModal(true);
}

FullScreenDialog::~FullScreenDialog() {}

void FullScreenDialog::setupUI(const QString &title)
{
    setObjectName("FullScreenDialog");
    setStyleSheet(
        "#FullScreenDialog { background: rgba(0,0,0,90); }"
        "QDialog { background: transparent; }"
    );

    QWidget *central = new QWidget(this);
    central->setObjectName("centralCard");
    central->setStyleSheet(
        "#centralCard {"
        "  background: #ffffff;"
        "  border-radius: 22px;"
        "}"
    );

    QVBoxLayout *mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(28, 18, 28, 18);
    mainLayout->addWidget(central);

    QVBoxLayout *cardLayout = new QVBoxLayout(central);
    cardLayout->setContentsMargins(0, 0, 0, 0);
    cardLayout->setSpacing(0);

    QWidget *titleBar = new QWidget();
    titleBar->setFixedHeight(76);
    titleBar->setStyleSheet(
        "background: #2c3e50;"
        "border-top-left-radius: 22px;"
        "border-top-right-radius: 22px;"
    );

    QLabel *titleLabel = new QLabel(title);
    titleLabel->setStyleSheet("color: white; font-size: 30px; font-weight: bold; padding-left: 18px;");

    QPushButton *closeBtn = new QPushButton("✕");
    closeBtn->setFixedSize(58, 58);
    closeBtn->setStyleSheet(
        "QPushButton {"
        "  background: transparent;"
        "  color: white;"
        "  border: none;"
        "  font-size: 34px;"
        "  border-radius: 29px;"
        "}"
        "QPushButton:hover {"
        "  background: #e74c3c;"
        "}"
    );
    connect(closeBtn, &QPushButton::clicked, this, &QDialog::reject);

    QHBoxLayout *titleLayout = new QHBoxLayout(titleBar);
    titleLayout->addWidget(titleLabel);
    titleLayout->addStretch();
    titleLayout->addWidget(closeBtn);
    titleLayout->setContentsMargins(18, 0, 22, 0);
    cardLayout->addWidget(titleBar);

    // 内容区域放到滚动区里，字体放大后不会被底部按钮栏截断。
    QScrollArea *scrollArea = new QScrollArea();
    scrollArea->setObjectName("contentScrollArea");
    scrollArea->setWidgetResizable(true);
    scrollArea->setFrameShape(QFrame::NoFrame);
    scrollArea->setStyleSheet(
        "#contentScrollArea {"
        "  background: #fefefe;"
        "  border: none;"
        "}"
    );

    QWidget *contentArea = new QWidget();
    contentArea->setObjectName("contentArea");
    contentArea->setStyleSheet("#contentArea { background: #fefefe; }");
    scrollArea->setWidget(contentArea);

    cardLayout->addWidget(scrollArea, 1);

    QWidget *buttonBar = new QWidget();
    buttonBar->setFixedHeight(92);
    buttonBar->setStyleSheet(
        "background: #f5f7fa;"
        "border-bottom-left-radius: 22px;"
        "border-bottom-right-radius: 22px;"
    );

    QHBoxLayout *btnLayout = new QHBoxLayout(buttonBar);
    btnLayout->setContentsMargins(26, 12, 26, 12);
    btnLayout->addStretch();

    QPushButton *okBtn = new QPushButton("✓ 确定");
    QPushButton *cancelBtn = new QPushButton("✗ 取消");
    QPushButton *defaultBtn = new QPushButton("⟳ 恢复默认");

    okBtn->setFixedSize(170, 62);
    cancelBtn->setFixedSize(170, 62);
    defaultBtn->setFixedSize(230, 62);

    QString btnStyle =
        "QPushButton { background: #3498db; color: white; border: none; border-radius: 16px; font-size: 26px; font-weight: bold; }"
        "QPushButton:hover { background: #2980b9; }";
    QString cancelStyle =
        "QPushButton { background: #95a5a6; color: white; border: none; border-radius: 16px; font-size: 26px; font-weight: bold; }"
        "QPushButton:hover { background: #7f8c8d; }";
    QString defaultStyle =
        "QPushButton { background: #f1c40f; color: #2c3e50; border: none; border-radius: 16px; font-size: 26px; font-weight: bold; }"
        "QPushButton:hover { background: #f39c12; }";

    okBtn->setStyleSheet(btnStyle);
    cancelBtn->setStyleSheet(cancelStyle);
    defaultBtn->setStyleSheet(defaultStyle);

    btnLayout->addWidget(defaultBtn);
    btnLayout->addSpacing(22);
    btnLayout->addWidget(okBtn);
    btnLayout->addSpacing(14);
    btnLayout->addWidget(cancelBtn);

    cardLayout->addWidget(buttonBar);

    connect(okBtn, &QPushButton::clicked, this, &QDialog::accept);
    connect(cancelBtn, &QPushButton::clicked, this, &QDialog::reject);
}

void FullScreenDialog::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        m_dragPos = event->globalPos() - frameGeometry().topLeft();
        event->accept();
    }
}

void FullScreenDialog::mouseMoveEvent(QMouseEvent *event)
{
    if (event->buttons() & Qt::LeftButton) {
        move(event->globalPos() - m_dragPos);
        event->accept();
    }
}
