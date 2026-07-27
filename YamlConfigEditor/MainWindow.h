#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "AllConfig.h"
#include <QPushButton>
class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

private slots:
    void onBasicSettings();
    void onScaleSettings();
    void onFeatureSettings();
    void onCameraAdjustSettings();
    void onExposureSettings();
    void onSeamSettings();
    void onGazeSettings();
    void onCameraInfoSettings();
    void onCameraParamsSettings();
    void onSave();
    void exitApplication();          // 退出程序

private:
    AllConfig m_config;
    void setupUI();
    void loadConfig();
    QPushButton* m_exitButton;       // 退出按钮
};

#endif // MAINWINDOW_H
