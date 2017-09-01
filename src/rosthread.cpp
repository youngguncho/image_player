#include "rosthread.h"


RosThread::RosThread(QObject *parent, ros::NodeHandle &pnh)
    : pnh_(pnh), cfg_server_(pnh)
{
    img_subs_ = nh_.subscribe<sensor_msgs::Image>("zed/left/image_rect_color", 2, boost::bind(&RosThread::image_callback, this, _1));
    cfg_server_.setCallback(boost::bind(&RosThread::config_callback, this, _1, _2));

}

void RosThread::run()
{
  ros::spin();
}

void RosThread::config_callback(image_player::image_playerConfig &config, int level)
{
    cout << "dyn config " << config.test2 << endl;
}

void
RosThread::image_callback(const sensor_msgs::Image::ConstPtr& msg)
{
    try
    {
        cout << "subscribe image" << endl;
        cv::Mat img_color;
        img_color = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::cvtColor(img_color, img_, CV_BGR2GRAY);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    emit image_signal();
}
