#include "image_player/data_manager.h"

//DataManager::DataManager()
//{

//}

void
DataManager::initialize(ros::NodeHandle& private_nh)
{
    private_nh_ = private_nh;

    // get ros params
    std::string root_dir, file_ext;
    private_nh_.param("file_path", root_dir, std::string("/mnt/data/Dataset/kitti/dataset/sequences/10/image_0/"));
    private_nh_.param("file_ext", file_ext, std::string("png"));
    private_nh_.param("name", image_name_, std::string("image"));
    private_nh_.param("framerate", framerate_, 10.0);
    image_info_name_ = image_name_ + "/info";

    QString qfile_ext;
    image_dir_ = QString::fromStdString(root_dir);
    qfile_ext = QString::fromStdString(file_ext);
    qfile_ext = "*." + qfile_ext;
    image_dir_.setNameFilters(QStringList()<<qfile_ext);
    image_list_ = image_dir_.entryList();
    natural_sort(image_list_);

    end_idx_ = image_list_.size() - 1;

    prepare_image();
    prepare_cam_info();

    height_ = image_.rows;
    width_ = image_.cols;

}


QImage DataManager::cv_mat_to_qimage( const cv::Mat &in_mat )
{
    switch ( in_mat.type() )
    {
       // 8-bit, 3 channel
       case CV_8UC3:
       {
          QImage image( in_mat.data,
                        in_mat.cols, in_mat.rows,
                        static_cast<int>(in_mat.step),
                        QImage::Format_RGB888 );

          return image.rgbSwapped();
       }
       // 8-bit, 1 channel
       case CV_8UC1:
       {
          static QVector<QRgb>  sColorTable( 256 );
          // only create our color table the first time
          if ( sColorTable.isEmpty() ) {
             for ( int i = 0; i < 256; ++i ) {
                sColorTable[i] = qRgb( i, i, i );
             }
          }

          QImage image( in_mat.data,
                        in_mat.cols, in_mat.rows,
                        static_cast<int>(in_mat.step),
                        QImage::Format_Grayscale8 );

          image.setColorTable( sColorTable );
          return image;
       }

       default:
          qWarning() << "ASM::cvMatToQImage() - cv::Mat image type not handled in switch:" << in_mat.type();
          break;
    }

    return QImage();
}

QPixmap DataManager::cv_mat_to_qpixmap( const cv::Mat &in_mat )
{
    return QPixmap::fromImage( cv_mat_to_qimage( in_mat ) );
}


void DataManager::natural_sort(QStringList &list)
{
    QCollator collator;
    collator.setNumericMode(true);
    std::sort(
        list.begin(),
        list.end(),
        [&collator](const QString &file1, const QString &file2)
        {
            return collator.compare(file1, file2) < 0;
        }
    );
}

void DataManager::prepare_image()
{
    image_ = cv::imread(image_dir_.absoluteFilePath(image_list_[idx_]).toStdString());
    qpixmap_ = cv_mat_to_qpixmap(image_);
}

void DataManager::prepare_cam_info()
{
    camera_info_.header.seq = idx_;
    QString image_string = image_list_[idx_].left(19);
    camera_info_.header.stamp.fromNSec(image_string.toLongLong());
    camera_info_.header.frame_id = image_info_name_;
    // Fill image size
    camera_info_.height = image_.rows;
    camera_info_.width = image_.cols;

    // Add the most common distortion model as sensor_msgs/CameraInfo says
    camera_info_.distortion_model = "plumb_bob";
    // Don't let distorsion matrix be empty
    camera_info_.D.resize(5, 0.0);
    // Give a reasonable default intrinsic camera matrix

    camera_info_.K = boost::assign::list_of(481.20) (0.0) (319.50)
                                           (0.0) (480.0) (239.50)
                                           (0.0) (0.0) (1.0);
    // Give a reasonable default rectification matrix
    camera_info_.R = boost::assign::list_of (1.0) (0.0) (0.0)
                                            (0.0) (1.0) (0.0)
                                            (0.0) (0.0) (1.0);
    // Give a reasonable default projection matrix
    camera_info_.P = boost::assign::list_of (481.20) (0.0) (319.50) (0.0)
                                            (0.0) (480.00) (239.50) (0.0)
                                            (0.0) (0.0) (1.0) (0.0);


  //  cam_info_.K = boost::assign::list_of(538.7) (0.0) (319.2)
  //                                         (0.0) (540.7) (233.6)
  //                                         (0.0) (0.0) (1.0);
  //  // Give a reasonable default rectification matrix
  //  cam_info_.R = boost::assign::list_of (1.0) (0.0) (0.0)
  //                                          (0.0) (1.0) (0.0)
  //                                          (0.0) (0.0) (1.0);
  //  // Give a reasonable default projection matrix
  //  cam_info_.P = boost::assign::list_of (583.7) (0.0) (319.20) (0.0)
  //                                          (0.0) (540.70) (233.60) (0.0)
  //                                          (0.0) (0.0) (1.0) (0.0);

}

void DataManager::print_file_list(QStringList list)
{
    qDebug() << "Print file list ";
    for (size_t i = 0; i < list.size(); ++i) {
        qDebug() << image_list_[i];
    }

}

