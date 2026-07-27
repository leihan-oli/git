#include "MainWindow.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QPushButton>
#include <QStatusBar>
#include <QMessageBox>
#include <QFileDialog>
#include <QDir>
#include <QApplication>
#include <QScrollArea>
#include <QLabel>
#include <QSizePolicy>
#include <QScreen>
#include <QDebug>

#include "dialogs/BasicSettingsDialog.h"
#include "dialogs/ScaleDialog.h"
#include "dialogs/FeatureDialog.h"
#include "dialogs/CameraAdjustDialog.h"
#include "dialogs/ExposureDialog.h"
#include "dialogs/SeamDialog.h"
#include "dialogs/GazeDialog.h"
#include "dialogs/CameraInfoDialog.h"
#include "dialogs/CameraParamsDialog.h"

static void applyGlobalDialogStyle()
{
    static bool applied = false;
    if (applied) return;
    applied = true;

    qApp->setStyleSheet(qApp->styleSheet() + R"(
        QDialog {
            font-size: 26px;
            background: #f3f6f8;
        }

        QDialog QLabel,
        QDialog QCheckBox,
        QDialog QRadioButton {
            font-size: 26px;
            color: #1f2d3d;
        }

        QDialog QGroupBox {
            font-size: 26px;
            font-weight: bold;
            color: #1e466e;
            border: 2px solid #c8d3dc;
            border-radius: 12px;
            margin-top: 18px;
            padding: 14px 12px 10px 12px;
            background: #ffffff;
        }

        QDialog QGroupBox::title {
            subcontrol-origin: margin;
            left: 16px;
            padding: 0 8px;
            background: #ffffff;
        }

        QDialog QLineEdit,
        QDialog QComboBox,
        QDialog QSpinBox,
        QDialog QDoubleSpinBox {
            font-size: 26px;
            min-height: 52px;
            padding: 5px 12px;
            border: 2px solid #aebdcc;
            border-radius: 10px;
            background: white;
            color: #1f2d3d;
        }

        QDialog QComboBox::drop-down {
            width: 44px;
        }

        QDialog QPushButton {
            font-size: 26px;
            font-weight: bold;
            min-width: 180px;
            min-height: 62px;
            padding: 8px 18px;
            border-radius: 16px;
        }

        QDialog QDialogButtonBox QPushButton {
            min-width: 160px;
            min-height: 58px;
        }

        QDialog QTableWidget {
            font-size: 24px;
            gridline-color: #c8d3dc;
            selection-background-color: #4a6a8b;
            selection-color: white;
        }

        QDialog QHeaderView::section {
            font-size: 24px;
            font-weight: bold;
            min-height: 48px;
            padding: 8px;
            background: #d9e2e8;
            color: #1e466e;
            border: 1px solid #c8d3dc;
        }

        QDialog QScrollArea {
            border: none;
            background: transparent;
        }

        QScrollBar:vertical {
            width: 26px;
            background: #e8eef2;
            margin: 0px;
            border-radius: 13px;
        }

        QScrollBar::handle:vertical {
            background: #8fa3b5;
            min-height: 50px;
            border-radius: 13px;
        }

        QScrollBar:horizontal {
            height: 26px;
            background: #e8eef2;
            margin: 0px;
            border-radius: 13px;
        }

        QScrollBar::handle:horizontal {
            background: #8fa3b5;
            min-width: 50px;
            border-radius: 13px;
        }

        QStatusBar {
            font-size: 20px;
        }

        QMessageBox QLabel {
            font-size: 24px;
        }

        QMessageBox QPushButton {
            font-size: 24px;
            min-width: 140px;
            min-height: 54px;
        }
    )");
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , m_exitButton(nullptr)
{
    setupUI();
    loadConfig();
}

MainWindow::~MainWindow() {}

void MainWindow::setupUI()
{
    applyGlobalDialogStyle();

    setWindowTitle("YAML配置编辑器");
    setStyleSheet("QMainWindow { background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #eef2f3, stop:1 #d9e2e8); }");

    QWidget *central = new QWidget(this);
    central->setObjectName("centralWidget");
    central->setStyleSheet("#centralWidget { background: transparent; }");
    setCentralWidget(central);

    QVBoxLayout *mainLayout = new QVBoxLayout(central);
    mainLayout->setContentsMargins(42, 18, 42, 18);
    mainLayout->setSpacing(14);

    QLabel *titleLabel = new QLabel("⚙️ 图像拼接配置编辑器", this);
    titleLabel->setAlignment(Qt::AlignCenter);
    titleLabel->setFixedHeight(58);
    titleLabel->setStyleSheet(
        "font-size: 34px;"
        "font-weight: bold;"
        "color: #1e466e;"
        "background: transparent;"
    );
    mainLayout->addWidget(titleLabel);

    QWidget *gridWidget = new QWidget(this);
    gridWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    QGridLayout *gridLayout = new QGridLayout(gridWidget);
    gridLayout->setContentsMargins(0, 0, 0, 0);
    gridLayout->setHorizontalSpacing(26);
    gridLayout->setVerticalSpacing(22);

    for (int c = 0; c < 3; ++c) {
        gridLayout->setColumnStretch(c, 1);
        gridLayout->setColumnMinimumWidth(c, 330);
    }
    for (int r = 0; r < 4; ++r) {
        gridLayout->setRowStretch(r, 1);
        gridLayout->setRowMinimumHeight(r, 92);
    }

    QPushButton *btnBasic = new QPushButton("基础设置");
    QPushButton *btnScale = new QPushButton("图像缩放");
    QPushButton *btnFeature = new QPushButton("特征与匹配");
    QPushButton *btnAdjust = new QPushButton("相机位姿");
    QPushButton *btnExposure = new QPushButton("☀️ 曝光补偿");
    QPushButton *btnSeam = new QPushButton("接缝融合");
    QPushButton *btnGaze = new QPushButton("注视感知");
    QPushButton *btnCamInfo = new QPushButton("相机信息");
    QPushButton *btnCamParams = new QPushButton("相机内参");
    QPushButton *btnSave = new QPushButton("保存配置");

    m_exitButton = new QPushButton("退出程序");

    QString btnStyle = R"(
        QPushButton {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #4a6a8b, stop:1 #2c3e50);
            color: white;
            font-size: 30px;
            font-weight: bold;
            border: none;
            border-radius: 22px;
            padding: 12px 10px;
            min-height: 92px;
        }
        QPushButton:hover {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #5a7a9b, stop:1 #3c4e60);
        }
        QPushButton:pressed {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #2c3e50, stop:1 #1e2e40);
        }
    )";

    QString saveStyle = R"(
        QPushButton {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #27ae60, stop:1 #1e8449);
            color: white;
            font-size: 31px;
            font-weight: bold;
            border: none;
            border-radius: 22px;
            padding: 12px 10px;
            min-height: 92px;
        }
        QPushButton:hover {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #2ecc71, stop:1 #229954);
        }
        QPushButton:pressed {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #1e8449, stop:1 #145a32);
        }
    )";

    QString exitStyle = R"(
        QPushButton {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #c0392b, stop:1 #e74c3c);
            color: white;
            font-size: 31px;
            font-weight: bold;
            border: none;
            border-radius: 22px;
            padding: 12px 10px;
            min-height: 92px;
        }
        QPushButton:hover {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #a93226, stop:1 #c0392b);
        }
        QPushButton:pressed {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #922b21, stop:1 #7b241c);
        }
    )";

    QList<QPushButton*> normalButtons;
    normalButtons << btnBasic
                  << btnScale
                  << btnFeature
                  << btnAdjust
                  << btnExposure
                  << btnSeam
                  << btnGaze
                  << btnCamInfo
                  << btnCamParams;

    for (QPushButton *btn : normalButtons) {
        btn->setStyleSheet(btnStyle);
        btn->setCursor(Qt::PointingHandCursor);
        btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    }

    btnSave->setStyleSheet(saveStyle);
    btnSave->setCursor(Qt::PointingHandCursor);
    btnSave->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    m_exitButton->setStyleSheet(exitStyle);
    m_exitButton->setCursor(Qt::PointingHandCursor);
    m_exitButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    gridLayout->addWidget(btnBasic,     0, 0);
    gridLayout->addWidget(btnScale,     0, 1);
    gridLayout->addWidget(btnFeature,   0, 2);

    gridLayout->addWidget(btnAdjust,    1, 0);
    gridLayout->addWidget(btnExposure,  1, 1);
    gridLayout->addWidget(btnSeam,      1, 2);

    gridLayout->addWidget(btnGaze,      2, 0);
    gridLayout->addWidget(btnCamInfo,   2, 1);
    gridLayout->addWidget(btnCamParams, 2, 2);

    gridLayout->addWidget(btnSave,      3, 0, 1, 2);
    gridLayout->addWidget(m_exitButton, 3, 2);

    // 关键：这里给 gridWidget stretch=1，让按钮区域吃掉剩余高度；
    // 不再 mainLayout->addStretch()，避免全部集中在上半屏。
    mainLayout->addWidget(gridWidget, 1);

    connect(btnBasic, &QPushButton::clicked, this, &MainWindow::onBasicSettings);
    connect(btnScale, &QPushButton::clicked, this, &MainWindow::onScaleSettings);
    connect(btnFeature, &QPushButton::clicked, this, &MainWindow::onFeatureSettings);
    connect(btnAdjust, &QPushButton::clicked, this, &MainWindow::onCameraAdjustSettings);
    connect(btnExposure, &QPushButton::clicked, this, &MainWindow::onExposureSettings);
    connect(btnSeam, &QPushButton::clicked, this, &MainWindow::onSeamSettings);
    connect(btnGaze, &QPushButton::clicked, this, &MainWindow::onGazeSettings);
    connect(btnCamInfo, &QPushButton::clicked, this, &MainWindow::onCameraInfoSettings);
    connect(btnCamParams, &QPushButton::clicked, this, &MainWindow::onCameraParamsSettings);
    connect(btnSave, &QPushButton::clicked, this, &MainWindow::onSave);
    connect(m_exitButton, &QPushButton::clicked, this, &MainWindow::exitApplication);

    statusBar()->setStyleSheet("QStatusBar { font-size: 20px; color: #1e466e; }");
    statusBar()->showMessage("就绪");

    QScreen *screen = QApplication::primaryScreen();
    if (screen) {
        qDebug() << "Qt screen geometry:" << screen->geometry()
                 << "available:" << screen->availableGeometry()
                 << "devicePixelRatio:" << screen->devicePixelRatio()
                 << "logicalDPI:" << screen->logicalDotsPerInch();
    }
}

void MainWindow::loadConfig()
{
    QString configPath = QDir::current().absoluteFilePath("config_3cam.yaml");
    if (!m_config.loadFromFile(configPath.toStdString())) {
        m_config.setDefaults();
        statusBar()->showMessage("未找到或无法加载 config_3cam.yaml，使用默认配置", 5000);
    } else {
        statusBar()->showMessage("已加载配置: " + configPath, 3000);
    }
}

void MainWindow::onBasicSettings()
{
    BasicSettingsDialog dlg(m_config.basic, nullptr);
    dlg.showFullScreen();
    dlg.exec();
}

void MainWindow::onScaleSettings()
{
    ScaleDialog dlg(m_config.scale, nullptr);
    dlg.showFullScreen();
    dlg.exec();
}

void MainWindow::onFeatureSettings()
{
    FeatureDialog dlg(m_config.feature, nullptr);
    dlg.showFullScreen();
    dlg.exec();
}

void MainWindow::onCameraAdjustSettings()
{
    CameraAdjustDialog dlg(m_config.cameraAdjust, nullptr);
    dlg.showFullScreen();
    dlg.exec();
}

void MainWindow::onExposureSettings()
{
    ExposureDialog dlg(m_config.exposure, nullptr);
    dlg.showFullScreen();
    dlg.exec();
}

void MainWindow::onSeamSettings()
{
    SeamDialog dlg(m_config.seam, nullptr);
    dlg.showFullScreen();
    dlg.exec();
}

void MainWindow::onGazeSettings()
{
    GazeDialog dlg(m_config.gaze, m_config.saliency, nullptr);
    dlg.showFullScreen();
    dlg.exec();
}

void MainWindow::onCameraInfoSettings()
{
    CameraInfoDialog dlg(m_config.cameraInfos, nullptr);
    dlg.showFullScreen();
    dlg.exec();
}

void MainWindow::onCameraParamsSettings()
{
    CameraParamsDialog dlg(m_config.cameraParamsList, nullptr);
    dlg.showFullScreen();
    dlg.exec();
}

void MainWindow::onSave()
{
    QString savePath = QDir::current().absoluteFilePath("config_3cam.yaml");
    if (m_config.saveToFile(savePath.toStdString())) {
        statusBar()->showMessage("配置已保存到: " + savePath, 5000);
    } else {
        QMessageBox::critical(this, "保存失败", "无法保存配置文件，请检查路径权限或磁盘空间。");
    }
}

void MainWindow::exitApplication()
{
    qApp->quit();
}
