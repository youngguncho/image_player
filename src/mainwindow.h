#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <ros/ros.h>

#include <QMainWindow>
#include <QDir>
#include <QTimer>
#include <QString>
#include <QDebug>
#include <QPixmap>
#include <QThread>
#include <QFileInfoList>
#include <QCollator>

#include <iostream>
#include <algorithm>
#include <vector>
#include <boost/assign/list_of.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
//#include <camera_info_manager/camera_info_manager.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>

#include "rosthread.h"
#include "image_player/data_manager.h"



namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void ros_init(ros::NodeHandle &nh, ros::NodeHandle &private_nh);

    QTimer *timer;
    RosThread *rosth;

public slots:
    void timer_slot();

    void on_idx_slider_sliderReleased();

    void on_idx_slider_sliderPressed();

    void on_framerate_box_editingFinished();

    void on_idx_slider_valueChanged(int value);

private:
    Ui::MainWindow *ui;

    DataManager dm_;

    // image related
    int image_idx_;
    QDir image_dir_;
    QStringList image_list_;


    cv::Mat image_;
    QPixmap image_qt_;
    sensor_msgs::CameraInfo cam_info_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Publisher image_pub_;
    ros::Publisher cam_info_pub_;

    void prepare_image();
    void prepare_cam_info();

    void publish_image();
    void publish_camera_info();

    void natural_sort(QStringList &list);

    void print_file_list(QStringList list);

};

#endif // MAINWINDOW_H
