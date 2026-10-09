#include <iostream>
#include "main_window.h"
#include <QFile>
#include <QStyleFactory>
#include <QApplication>
#include <QFont>
#include <QFontDatabase>
#include <QDebug>
#include <QCommandLineParser>
#include <QTimer>

int main(int argc, char **argv){
    bool requested_headless_auto = false;
    for (int i = 1; i < argc; ++i) {
        if (QString::fromLocal8Bit(argv[i]) == QStringLiteral("--headless-auto")) {
            requested_headless_auto = true;
            break;
        }
    }
    if (requested_headless_auto) {
        qputenv("QT_QPA_PLATFORM", QByteArrayLiteral("offscreen"));
        qputenv("QT_QUICK_BACKEND", QByteArrayLiteral("software"));
        qputenv("LIBGL_ALWAYS_SOFTWARE", QByteArrayLiteral("1"));
    }

    QApplication app(argc, argv);
    app.setApplicationDisplayName(QStringLiteral("CUARM Control"));
    app.setStyle(QStyleFactory::create("Fusion"));

    QCommandLineParser parser;
    parser.setApplicationDescription(
        QStringLiteral("CUARM dual_v2_2 upper control software"));
    parser.addHelpOption();
    const QCommandLineOption headless_auto_option(
        QStringLiteral("headless-auto"),
        QStringLiteral(
            "Run without windows, select hardware mode, and automatically "
            "Connect/Start the Remote planner bridge."));
    parser.addOption(headless_auto_option);
    parser.process(app);
    const bool headless_auto = parser.isSet(headless_auto_option);

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
    try {
        cuarm_panel::MainWindow w(
            config_prefix_path, project_base_path, headless_auto);
        if (headless_auto) {
            QTimer::singleShot(
                0, &w, &cuarm_panel::MainWindow::startHeadlessAuto);
        } else {
            w.show();
        }
        return app.exec();
    } catch (const std::exception& exc) {
        std::cerr << "[STARTUP ERROR] " << exc.what() << std::endl;
        return 1;
    }
}
