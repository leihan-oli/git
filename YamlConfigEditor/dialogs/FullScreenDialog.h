#ifndef FULLSCREENDIALOG_H
#define FULLSCREENDIALOG_H

#include <QDialog>
#include <QMouseEvent>

class FullScreenDialog : public QDialog
{
    Q_OBJECT
public:
    explicit FullScreenDialog(QWidget *parent = nullptr, const QString &title = "设置");
    ~FullScreenDialog();

protected:
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    virtual void onRestoreDefault() {}

private:
    QPoint m_dragPos;
    void setupUI(const QString &title);
};

#endif // FULLSCREENDIALOG_H
