#include "MainWindow.h"
#include <QApplication>
#include <QFont>
#include <QTextCodec>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    QTextCodec::setCodecForLocale(QTextCodec::codecForName("UTF-8"));

    // 1280x600 HDMI 屏幕专用：全局基础字体加大。
    QFont font;
    QStringList chineseFonts = {
        "Noto Sans CJK SC", "Source Han Sans SC", "WenQuanYi Micro Hei",
        "Microsoft YaHei", "SimSun", "Arial Unicode MS", "sans-serif"
    };
    for (const QString& fontName : chineseFonts) {
        if (QFont(fontName).exactMatch()) {
            font.setFamily(fontName);
            break;
        }
    }
    font.setPointSize(16);
    font.setBold(false);
    a.setFont(font);

    QApplication::setStyle("Fusion");

    MainWindow w;
    w.showFullScreen();
    return a.exec();
}
