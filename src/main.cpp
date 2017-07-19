#include "mainwindow.h"
#include <QApplication>
#include <ros/ros.h>
#include <ros/node_handle.h>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    ros::init(argc, argv, "image_player");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    MainWindow w;
    w.ros_init(nh, private_nh);
    w.show();

    return a.exec();
}
