#ifndef LAUNCHER_H
#define LAUNCHER_H

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QProcess>
#include <QMessageBox>

class Launcher : public QWidget
{
    Q_OBJECT

public:
    explicit Launcher(QWidget *parent = nullptr);
    ~Launcher();

protected:
    void resizeEvent(QResizeEvent *event) override;

private slots:
    // 设置按钮（二路/三路）
    void onSettings2Clicked();
    void onSettings3Clicked();

    // 启动拼接（二路/三路）
    void onStart2Clicked();
    void onStart3Clicked();

    void onStopClicked();
    void onExitClicked();
    void updateImages();
    void onStitcherFinished(int exitCode, QProcess::ExitStatus status);
    void onStitcherError(QProcess::ProcessError error);

private:
    void setupUI();
    void loadStyle();
    void adjustLayout();
    void updateSingleImage(QLabel *label, const QString &path, bool isLarge);
    void startExternalApp(const QString &appPath, const QStringList &arguments, QPushButton *btn);
    void safeTerminateProcess(QProcess *process, bool forceKill = false);

    // UI 组件
    QWidget *mainWidget;
    QLabel *largeImageLabel;
    QLabel *smallImageLabel;

    // 底部四个按钮
    QPushButton *settings2Btn;   // 设置二路
    QPushButton *settings3Btn;   // 设置三路
    QPushButton *stopBtn;
    QPushButton *exitBtn;

    // 右侧的两个启动按钮（原本在底部，现在移到右侧小图下方）
    QPushButton *start2Btn;
    QPushButton *start3Btn;

    QTimer *refreshTimer;

    // RTStitcher 进程管理
    QProcess *stitcherProcess;
    bool isStitcherRunning;
    QString currentConfigPath;

    // 文件路径
    QString largeImagePath;
    QString smallImagePath;
    QString config2Path;
    QString config3Path;
    QString editor2Path;
    QString editor3Path;

    // 外部编辑器进程列表
    QList<QProcess*> editorProcesses;
};

#endif // LAUNCHER_H
