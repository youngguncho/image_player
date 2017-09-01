#ifndef DATA_MANAGER_H
#define DATA_MANAGER_H

#include <ros/ros.h>
#include <string>

#include <QDir>
#include <QStringList>
#include <QImage>
#include <QPixmap>
#include <QCollator>
#include <QDebug>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/assign/list_of.hpp>


#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class DataManager
{
public:
    DataManager() {idx_ = 0;}

    void set_index(int idx) { idx_ = idx; }
    void set_frame_rate(double framerate);

    void prepare_image();
    void prepare_cam_info();

    void framerate(double fr) { framerate_ = fr; }
    double framerate() { return framerate_; }
    double frameduration() {return 1000/framerate_; }

    std::string image_name() { return image_name_; }
    std::string image_info_name() {return image_info_name_; }
    sensor_msgs::CameraInfo& camera_info() { return camera_info_; }

    cv::Mat& image() { return image_; }
    QPixmap& qimage() { return qpixmap_; }
    int height() { return height_; }
    int width() { return width_; }

    QStringList image_list() { return image_list_; }

    void idx_up() { idx_++; idx_ = (idx_ < end_idx_) ? idx_ : end_idx_ ;}
    void idx_down() { idx_--; idx_ = (idx_ > 0) ? idx_ : 0 ;}
    int idx() { return idx_; }
    int end_idx() { return end_idx_; }

    void print_file_list(QStringList list);

    void initialize(ros::NodeHandle &private_nh);

protected:
    void natural_sort(QStringList &list);
    QImage cv_mat_to_qimage( const cv::Mat &in_mat );
    QPixmap cv_mat_to_qpixmap( const cv::Mat &in_mat );



private:
    // node handler
    ros::NodeHandle private_nh_;

    // image related
    int idx_;
    int end_idx_;
    int height_;
    int width_;
    std::string image_name_;
    std::string image_info_name_;

    double framerate_;

    QDir image_dir_;
    QStringList image_list_;

    cv::Mat image_;
    QPixmap qpixmap_;
    sensor_msgs::CameraInfo camera_info_;


};

#endif // DATA_MANAGER_H
