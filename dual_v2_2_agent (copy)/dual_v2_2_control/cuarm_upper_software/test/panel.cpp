#include <iostream>
#include "main_window.h"
#include <QFile>
#include <QStyleFactory>
#include <QApplication>
#include <QFont>
#include <QFontDatabase>
#include <QDebug>

int main(int argc, char **argv){
    QApplication app(argc, argv);
    app.setApplicationDisplayName(QStringLiteral("CUARM Control"));
    app.setStyle(QStyleFactory::create("Fusion"));

    QFontDatabase font_database;
    const QStringList installed_families = font_database.families();
    const QStringList preferred_families{
        QStringLiteral("Noto Sans CJK SC"),
        QStringLiteral("Source Han Sans SC"),
        QStringLiteral("Microsoft YaHei UI"),
        QStringLiteral("WenQuanYi Micro Hei")
    };
    QString font_family = QApplication::font().family();
    for (const QString& preferred : preferred_families) {
        if (installed_families.contains(preferred)) {
            font_family = preferred;
            break;
        }
    }
    QFont font(font_family, 10);
    font.setStyleStrategy(QFont::PreferAntialias);
    QApplication::setFont(font);
    
    //Add StyleSheet
    std::string project_base_path = PROJECT_SOURCE_DIR;
    QString qss_path = QString::fromStdString(project_base_path+"/src/panel/styleSheet/mainwindowStyleSheet.qss");
    QFile file(qss_path);
    file.open(QFile::ReadOnly);
    const QString styleSheet = QLatin1String(file.readAll());
    app.setStyleSheet(styleSheet);

    //Show main window
    std::string config_prefix_path = CONFIG_PREFIX_PATH;
    std::cout <<"config_base_path: " << config_prefix_path <<"\n";
    cuarm_panel::MainWindow w(config_prefix_path, project_base_path);
    w.show();
    return app.exec();
}
