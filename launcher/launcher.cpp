#include "launcher.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFile>
#include <QFileInfo>
#include <QPixmap>
#include <QApplication>
#include <QScreen>
#include <QDebug>
#include <QThread>
#include <QMessageBox>
#include <QSizePolicy>
#include <QtGlobal>

Launcher::Launcher(QWidget *parent)
    : QWidget(parent)
    , stitcherProcess(nullptr)
    , isStitcherRunning(false)
{
    largeImagePath = "/tmp/stitched.jpg";
    smallImagePath = "/tmp/seam_alpha_compare.jpg";

    config2Path = QCoreApplication::applicationDirPath() + "/YamlConfigEditor/config.yaml";
    config3Path = QCoreApplication::applicationDirPath() + "/YamlConfigEditor/config_3cam.yaml";
    currentConfigPath = config2Path;

    editor2Path = QCoreApplication::applicationDirPath() + "/YamlConfigEditor/YamlConfigEditor2";
    editor3Path = QCoreApplication::applicationDirPath() + "/YamlConfigEditor/YamlConfigEditor3";

    setupUI();
    loadStyle();
    adjustLayout();

    refreshTimer = new QTimer(this);
    connect(refreshTimer, &QTimer::timeout, this, &Launcher::updateImages);
    refreshTimer->start(33);
}

Launcher::~Launcher()
{
    if (refreshTimer) refreshTimer->stop();
    safeTerminateProcess(stitcherProcess, true);
    delete stitcherProcess;
    stitcherProcess = nullptr;

    for (QProcess *p : editorProcesses) {
        safeTerminateProcess(p, true);
        delete p;
    }
    editorProcesses.clear();

    isStitcherRunning = false;
}

void Launcher::setupUI()
{
    setWindowState(Qt::WindowFullScreen);
    setWindowFlags(Qt::FramelessWindowHint);
    setAttribute(Qt::WA_TranslucentBackground, true);
    setStyleSheet("background: transparent;");

    mainWidget = new QWidget(this);
    mainWidget->setObjectName("mainWidget");
    mainWidget->setGeometry(0, 0, width(), height());

    QVBoxLayout *mainLayout = new QVBoxLayout(mainWidget);

    // 针对 1280x600 HDMI 屏幕优化
    mainLayout->setContentsMargins(24, 16, 24, 16);
    mainLayout->setSpacing(16);

    // ===== 中央区域：左侧大图 + 右侧（小图在上，启动按钮在下） =====
    QHBoxLayout *centerLayout = new QHBoxLayout();
    centerLayout->setSpacing(24);

    // --- 左侧大图 ---
    QWidget *leftContainer = new QWidget();
    QVBoxLayout *leftLayout = new QVBoxLayout(leftContainer);
    leftLayout->setContentsMargins(0, 0, 0, 0);
    leftLayout->setSpacing(0);

    largeImageLabel = new QLabel();
    largeImageLabel->setObjectName("largeImageLabel");
    largeImageLabel->setAlignment(Qt::AlignCenter);
    largeImageLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    largeImageLabel->setStyleSheet("background: rgba(0,0,0,0.3); border-radius: 15px;");
    leftLayout->addWidget(largeImageLabel);

    centerLayout->addWidget(leftContainer, 1);

    // --- 右侧容器（小图在顶部，启动按钮在底部） ---
    QWidget *rightContainer = new QWidget();

    // 右上角 seam_alpha_compare 是很宽的横图。
    // 如果想让文字清楚，仅增加 smallH 没用，必须同步增加 smallW。
    // 520x190 比 440x160 更清楚，右侧容器宽度也要跟着放大。
    rightContainer->setFixedWidth(550);

    QVBoxLayout *rightLayout = new QVBoxLayout(rightContainer);
    rightLayout->setContentsMargins(0, 0, 0, 0);
    rightLayout->setSpacing(12);

    // 1. 小图放在顶部，水平居中
    QHBoxLayout *topRightLayout = new QHBoxLayout();
    topRightLayout->setContentsMargins(0, 0, 0, 0);
    topRightLayout->setSpacing(0);

    topRightLayout->addStretch();

    smallImageLabel = new QLabel();
    smallImageLabel->setObjectName("smallImageLabel");
    smallImageLabel->setAlignment(Qt::AlignCenter);
    smallImageLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    smallImageLabel->setStyleSheet("background: rgba(0,0,0,0.3); border-radius: 10px;");
    topRightLayout->addWidget(smallImageLabel);

    topRightLayout->addStretch();
    rightLayout->addLayout(topRightLayout);

    // 2. 弹性伸缩，将启动按钮推到底部
    rightLayout->addStretch();

    // 3. 两个启动按钮垂直排列，水平居中
    start2Btn = new QPushButton("▶ 启动二路");
    start2Btn->setObjectName("startBtn");
    start2Btn->setCursor(Qt::PointingHandCursor);

    start3Btn = new QPushButton("▶ 启动三路");
    start3Btn->setObjectName("startBtn");
    start3Btn->setCursor(Qt::PointingHandCursor);

    QVBoxLayout *buttonLayout = new QVBoxLayout();
    buttonLayout->setContentsMargins(0, 0, 0, 0);
    buttonLayout->setSpacing(18);
    buttonLayout->addWidget(start2Btn, 0, Qt::AlignHCenter);
    buttonLayout->addWidget(start3Btn, 0, Qt::AlignHCenter);
    rightLayout->addLayout(buttonLayout);

    rightLayout->addSpacing(8);

    centerLayout->addWidget(rightContainer);
    mainLayout->addLayout(centerLayout, 1);

    // ===== 底部四个按钮 =====
    QHBoxLayout *bottomLayout = new QHBoxLayout();
    bottomLayout->setContentsMargins(0, 0, 0, 0);
    bottomLayout->setSpacing(18);

    settings2Btn = new QPushButton("⚙️ 设置二路");
    settings2Btn->setObjectName("settingsBtn");
    settings2Btn->setCursor(Qt::PointingHandCursor);

    settings3Btn = new QPushButton("⚙️ 设置三路");
    settings3Btn->setObjectName("settingsBtn");
    settings3Btn->setCursor(Qt::PointingHandCursor);

    stopBtn = new QPushButton("⏹ 结束拼接");
    stopBtn->setObjectName("stopBtn");
    stopBtn->setCursor(Qt::PointingHandCursor);
    stopBtn->setEnabled(false);

    exitBtn = new QPushButton("✖ 退出");
    exitBtn->setObjectName("exitBtn");
    exitBtn->setCursor(Qt::PointingHandCursor);

    bottomLayout->addStretch();
    bottomLayout->addWidget(settings2Btn);
    bottomLayout->addStretch();
    bottomLayout->addWidget(settings3Btn);
    bottomLayout->addStretch();
    bottomLayout->addWidget(stopBtn);
    bottomLayout->addStretch();
    bottomLayout->addWidget(exitBtn);
    bottomLayout->addStretch();

    mainLayout->addLayout(bottomLayout);

    // ===== 信号连接 =====
    connect(settings2Btn, &QPushButton::clicked, this, &Launcher::onSettings2Clicked);
    connect(settings3Btn, &QPushButton::clicked, this, &Launcher::onSettings3Clicked);
    connect(start2Btn, &QPushButton::clicked, this, &Launcher::onStart2Clicked);
    connect(start3Btn, &QPushButton::clicked, this, &Launcher::onStart3Clicked);
    connect(stopBtn, &QPushButton::clicked, this, &Launcher::onStopClicked);
    connect(exitBtn, &QPushButton::clicked, this, &Launcher::onExitClicked);
}

void Launcher::loadStyle()
{
    this->setStyleSheet(
        "#mainWidget {"
        "  background: qlineargradient(x1:0, y1:0, x2:1, y2:1,"
        "                              stop:0 #1a1a2e, stop:1 #16213e);"
        "}"
        "QPushButton {"
        "  background: qlineargradient(x1:0, y1:0, x2:0, y2:1,"
        "                              stop:0 #0f3460, stop:1 #16213e);"
        "  color: #e94560;"
        "  font-size: 24px;"
        "  font-weight: bold;"
        "  border: 3px solid #e94560;"
        "  border-radius: 16px;"
        "}"
        "QPushButton:hover {"
        "  background: #e94560;"
        "  color: #1a1a2e;"
        "  border-color: #1a1a2e;"
        "}"
        "QPushButton:disabled {"
        "  background: #2a2a3e;"
        "  color: #666;"
        "  border-color: #555;"
        "}"
        "#largeImageLabel, #smallImageLabel {"
        "  background: rgba(0,0,0,0.3);"
        "  border-radius: 15px;"
        "}"
    );
}

void Launcher::adjustLayout()
{
    /*
     * 1280x600 HDMI 屏幕推荐尺寸：
     *
     * 右侧小图 seam_alpha_compare：
     *   smallW = 520
     *   smallH = 190
     *
     * 注意：
     *   由于 updateSingleImage() 使用 Qt::KeepAspectRatio，
     *   图片实际显示大小主要由宽度 smallW 决定。
     *   只改 smallH，例如改到 220，如果 smallW 不变，图片本身不会明显变大。
     */

    QRect screenGeometry = QApplication::primaryScreen()->geometry();

    int sw = screenGeometry.width();
    int sh = screenGeometry.height();

    int smallW;
    int smallH;
    int btnW;
    int btnH;

    if (sw <= 1280 && sh <= 600) {
        // 适合你的 1280x600 HDMI 屏幕
        smallW = 520;
        smallH = 190;

        btnW = 230;
        btnH = 82;
    } else {
        // 其它分辨率下温和自适应
        smallW = qMin(560, qMax(520, sw / 3));
        smallH = qRound(smallW / 2.75);

        btnW = qMin(260, qMax(230, sw / 6));
        btnH = qMin(92,  qMax(82,  sh / 7));
    }

    smallImageLabel->setFixedSize(smallW, smallH);

    QList<QPushButton*> buttons;
    buttons << settings2Btn
            << settings3Btn
            << stopBtn
            << exitBtn
            << start2Btn
            << start3Btn;

    for (QPushButton *btn : buttons) {
        if (btn) {
            btn->setFixedSize(btnW, btnH);
        }
    }
}

void Launcher::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
    adjustLayout();
    mainWidget->resize(width(), height());
}

void Launcher::updateImages()
{
    updateSingleImage(largeImageLabel, largeImagePath, true);
    updateSingleImage(smallImageLabel, smallImagePath, false);
}

void Launcher::updateSingleImage(QLabel *label, const QString &path, bool isLarge)
{
    Q_UNUSED(isLarge)

    QFileInfo fileInfo(path);
    if (!fileInfo.exists()) {
        label->setText(QString("图片不存在\n%1").arg(path));
        label->setAlignment(Qt::AlignCenter);
        label->setStyleSheet("color: white; font-size: 14px; background: rgba(0,0,0,0.5);");
        return;
    }

    QPixmap pixmap(path);
    if (pixmap.isNull()) {
        qDebug() << "Failed to load image:" << path;
        return;
    }

    QPixmap scaled = pixmap.scaled(label->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
    label->setPixmap(scaled);
}

void Launcher::safeTerminateProcess(QProcess *process, bool forceKill)
{
    if (!process) return;

    if (process->state() == QProcess::Running) {
        if (forceKill) {
            process->kill();
            if (!process->waitForFinished(3000)) {
                process->waitForFinished(-1);
            }
        } else {
            process->terminate();
            if (!process->waitForFinished(3000)) {
                process->kill();
                process->waitForFinished(2000);
            }
        }
    }
}

void Launcher::startExternalApp(const QString &appPath, const QStringList &arguments, QPushButton *btn)
{
    if (!QFile::exists(appPath)) {
        QMessageBox::critical(this, "系统错误",
                              QString("找不到程序：\n%1\n请确保已正确部署。").arg(appPath));
        btn->setEnabled(true);
        btn->setText(btn->property("originalText").toString());
        return;
    }

    QProcess *process = new QProcess(this);
    editorProcesses.append(process);

    connect(process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, [this, process, btn](int, QProcess::ExitStatus) {
                editorProcesses.removeOne(process);
                process->deleteLater();
                btn->setEnabled(true);
                btn->setText(btn->property("originalText").toString());
            });

    connect(process, &QProcess::errorOccurred, this, [this, process, btn](QProcess::ProcessError error) {
        Q_UNUSED(error)
        QMessageBox::critical(this, "启动失败", QString("无法启动程序：%1").arg(btn->property("originalText").toString()));
        editorProcesses.removeOne(process);
        process->deleteLater();
        btn->setEnabled(true);
        btn->setText(btn->property("originalText").toString());
    });

    QString workingDir = QFileInfo(appPath).absolutePath();
    process->setWorkingDirectory(workingDir);
    process->start(appPath, arguments);

    btn->setEnabled(false);
    btn->setText("⏳ 启动中...");
}

// ---------- 设置二路/三路 ----------
void Launcher::onSettings2Clicked()
{
    QStringList arguments;
    arguments << config2Path;
    settings2Btn->setProperty("originalText", "⚙️ 设置二路");
    startExternalApp(editor2Path, arguments, settings2Btn);
}

void Launcher::onSettings3Clicked()
{
    QStringList arguments;
    arguments << config3Path;
    settings3Btn->setProperty("originalText", "⚙️ 设置三路");
    startExternalApp(editor3Path, arguments, settings3Btn);
}

// ---------- 启动拼接 ----------
void Launcher::onStart2Clicked()
{
    if (isStitcherRunning) {
        QMessageBox::warning(this, "进程已运行", "拼接程序已在运行中！");
        return;
    }

    if (!QFile::exists(config2Path)) {
        QMessageBox::critical(this, "配置文件缺失",
                              QString("找不到二路配置文件：\n%1\n请确保配置文件存在。").arg(config2Path));
        return;
    }

    QString programPath = "/root/build/RTStitcher";
    if (!QFile::exists(programPath)) {
        QMessageBox::critical(this, "程序缺失",
                              QString("找不到 RTStitcher 程序：\n%1\n请将 RTStitcher 放在 /root/build 下。").arg(programPath));
        return;
    }

    currentConfigPath = config2Path;
    stitcherProcess = new QProcess(this);
    QStringList arguments;
    arguments << "-c" << currentConfigPath;
    stitcherProcess->setWorkingDirectory(QCoreApplication::applicationDirPath());

    connect(stitcherProcess, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, &Launcher::onStitcherFinished, Qt::UniqueConnection);
    connect(stitcherProcess, &QProcess::errorOccurred,
            this, &Launcher::onStitcherError, Qt::UniqueConnection);

    stitcherProcess->start(programPath, arguments);
    if (stitcherProcess->waitForStarted(3000)) {
        isStitcherRunning = true;
        start2Btn->setEnabled(false);
        start3Btn->setEnabled(false);
        settings2Btn->setEnabled(false);
        settings3Btn->setEnabled(false);
        stopBtn->setEnabled(true);
        qDebug() << "RTStitcher (二路) 已启动，配置文件：" << currentConfigPath;
    } else {
        QMessageBox::critical(this, "启动失败", "无法启动 RTStitcher 程序。");
        delete stitcherProcess;
        stitcherProcess = nullptr;
    }
}

void Launcher::onStart3Clicked()
{
    if (isStitcherRunning) {
        QMessageBox::warning(this, "进程已运行", "拼接程序已在运行中！");
        return;
    }

    if (!QFile::exists(config3Path)) {
        QMessageBox::critical(this, "配置文件缺失",
                              QString("找不到三路配置文件：\n%1\n请确保配置文件存在。").arg(config3Path));
        return;
    }

    QString programPath = "/root/build/RTStitcher";
    if (!QFile::exists(programPath)) {
        QMessageBox::critical(this, "程序缺失",
                              QString("找不到 RTStitcher 程序：\n%1\n请将 RTStitcher 放在 /root/build 下。").arg(programPath));
        return;
    }

    currentConfigPath = config3Path;
    stitcherProcess = new QProcess(this);
    QStringList arguments;
    arguments << "-c" << currentConfigPath;
    stitcherProcess->setWorkingDirectory(QCoreApplication::applicationDirPath());

    connect(stitcherProcess, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, &Launcher::onStitcherFinished, Qt::UniqueConnection);
    connect(stitcherProcess, &QProcess::errorOccurred,
            this, &Launcher::onStitcherError, Qt::UniqueConnection);

    stitcherProcess->start(programPath, arguments);
    if (stitcherProcess->waitForStarted(3000)) {
        isStitcherRunning = true;
        start2Btn->setEnabled(false);
        start3Btn->setEnabled(false);
        settings2Btn->setEnabled(false);
        settings3Btn->setEnabled(false);
        stopBtn->setEnabled(true);
        qDebug() << "RTStitcher (三路) 已启动，配置文件：" << currentConfigPath;
    } else {
        QMessageBox::critical(this, "启动失败", "无法启动 RTStitcher 程序。");
        delete stitcherProcess;
        stitcherProcess = nullptr;
    }
}

// ---------- 停止拼接 ----------
void Launcher::onStopClicked()
{
    if (!isStitcherRunning || !stitcherProcess) return;

    disconnect(stitcherProcess, nullptr, this, nullptr);
    safeTerminateProcess(stitcherProcess, false);

    delete stitcherProcess;
    stitcherProcess = nullptr;
    isStitcherRunning = false;

    start2Btn->setEnabled(true);
    start3Btn->setEnabled(true);
    settings2Btn->setEnabled(true);
    settings3Btn->setEnabled(true);
    stopBtn->setEnabled(false);
}

void Launcher::onExitClicked()
{
    if (isStitcherRunning && stitcherProcess) {
        onStopClicked();
    }
    QApplication::quit();
}

void Launcher::onStitcherFinished(int, QProcess::ExitStatus)
{
    qDebug() << "RTStitcher 进程自然结束";
    if (stitcherProcess) {
        delete stitcherProcess;
        stitcherProcess = nullptr;
    }
    isStitcherRunning = false;
    start2Btn->setEnabled(true);
    start3Btn->setEnabled(true);
    settings2Btn->setEnabled(true);
    settings3Btn->setEnabled(true);
    stopBtn->setEnabled(false);
}

void Launcher::onStitcherError(QProcess::ProcessError error)
{
    if (!stitcherProcess) return;
    qDebug() << "RTStitcher error occurred:" << error;

    if (error != QProcess::Crashed) {
        QMessageBox::critical(this, "进程错误", "RTStitcher 运行过程中发生错误！");
    }

    disconnect(stitcherProcess, nullptr, this, nullptr);
    safeTerminateProcess(stitcherProcess, true);
    delete stitcherProcess;
    stitcherProcess = nullptr;
    isStitcherRunning = false;

    start2Btn->setEnabled(true);
    start3Btn->setEnabled(true);
    settings2Btn->setEnabled(true);
    settings3Btn->setEnabled(true);
    stopBtn->setEnabled(false);
}
