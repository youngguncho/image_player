#ifndef ROSTHREAD_H
#define ROSTHREAD_H

#include <QObject>
#include <QThread>
#include <QDebug>

#include <ros/ros.h>
#include <iostream>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

class RosThread : public QThread
{
   Q_OBJECT

public:
    RosThread(QObject *parent = 0);

    void
    run ();

    void get_cv_img (cv::Mat &cv_img) { cv_img = move(img_); }

private:
    cv::Mat img_;
    ros::NodeHandle nh_;

    ros::Subscriber img_subs_;

    void image_callback(const sensor_msgs::Image::ConstPtr &msg);

signals:
    void image_signal (void);


};

#endif // ROSTHREAD_H
