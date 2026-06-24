#include <QApplication>
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    MainWindow w;
    w.show();
    //rclcpp::shutdown();
    return app.exec();
}