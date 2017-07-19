#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  QString root_dir = "/home/yg/catkin_ws/data/images/";
//  QString root_dir = "/mnt/data/Dataset/rgbd-dataset/ethl/real/real_local/";

  std::string test;
  nh_.getParam("/test", test);
  std::cout << test << std::endl;

  image_idx_ = 0;
  image_dir_ = root_dir + "cai_zed/";
  image_dir_.setNameFilters(QStringList()<<"*.png");
  image_list_ = image_dir_.entryList();
  natural_sort(image_list_);

  image_pub_ = nh_.advertise<sensor_msgs::Image>("icl/rgb/image", 2);
  cam_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("icl/rgb/camera_info", 2);
  qDebug() << image_list_[0];
  QString image_name = image_list_[0].left(19);
  qDebug() << image_name << " / " << image_name.toLongLong();

  prepare_image();
  prepare_cam_info();

  // set thread
  rosth = new RosThread(this);
  rosth->start();

  timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(timer_slot()));
  timer->start(50);

}

MainWindow::~MainWindow()
{
  rosth->quit();
  if(!rosth->wait(500))
  {
      rosth->terminate();
      rosth->wait();
  }
  delete ui;
}

void MainWindow::ros_init(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
    nh_ = nh;
}

void MainWindow::timer_slot()
{
//    print_file_list(image_list_);
    double begin = ros::Time::now().toNSec();
    publish_camera_info();
    publish_image();

    ++image_idx_;

    prepare_image();
    prepare_cam_info();
    double end = ros::Time::now().toNSec();

    qDebug() << "Processing time " << (end - begin) / 10e9;
}

void MainWindow::prepare_image()
{
    image_ = cv::imread(image_dir_.absoluteFilePath(image_list_[image_idx_]).toStdString());
    float beta = 0.5;
    float A[3] = {222/255, 230/255, 233/255};

    cv::Mat r(image_.rows, image_.cols, CV_32FC1);
    cv::Mat g(image_.rows, image_.cols, CV_32FC1);
    cv::Mat b(image_.rows, image_.cols, CV_32FC1);

    image_qt_ = cvMatToQPixmap(image_);
}

void MainWindow::prepare_cam_info()
{
  cam_info_.header.seq = image_idx_;
  QString image_string = image_list_[image_idx_].left(19);
  cam_info_.header.stamp.fromNSec(image_string.toLongLong());
  cam_info_.header.frame_id = string("icl/rgb/image");
  // Fill image size
  cam_info_.height = image_.rows;
  cam_info_.width = image_.cols;

  // Add the most common distortion model as sensor_msgs/CameraInfo says
  cam_info_.distortion_model = "plumb_bob";
  // Don't let distorsion matrix be empty
  cam_info_.D.resize(5, 0.0);
  // Give a reasonable default intrinsic camera matrix

  cam_info_.K = boost::assign::list_of(481.20) (0.0) (319.50)
                                         (0.0) (480.0) (239.50)
                                         (0.0) (0.0) (1.0);
  // Give a reasonable default rectification matrix
  cam_info_.R = boost::assign::list_of (1.0) (0.0) (0.0)
                                          (0.0) (1.0) (0.0)
                                          (0.0) (0.0) (1.0);
  // Give a reasonable default projection matrix
  cam_info_.P = boost::assign::list_of (481.20) (0.0) (319.50) (0.0)
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

void MainWindow::publish_image()
{
    this->ui->img_label->setPixmap(image_qt_.scaled(this->ui->img_label->size()));
    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_).toImageMsg();
    QString image_string = image_list_[image_idx_].left(19);
    image_msg->header.stamp.fromNSec(image_string.toLongLong());
    image_pub_.publish(image_msg);
    ros::spinOnce();
}

void MainWindow::publish_camera_info()
{
    cam_info_pub_.publish(cam_info_);
}

void MainWindow::natural_sort(QStringList &list)
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

void MainWindow::print_file_list(QStringList list)
{
    qDebug() << "Print file list ";
    for (size_t i = 0; i < list.size(); ++i) {
        qDebug() << image_list_[i];
    }

}
