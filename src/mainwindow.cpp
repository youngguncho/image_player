#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // set thread
    rosth = new RosThread(this, private_nh_);
    rosth->start();

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(timer_slot()));
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

void MainWindow::ros_init(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
{
    nh_ = nh;
    private_nh_ = private_nh;
    dm_.initialize(private_nh_);

    image_pub_ = nh_.advertise<sensor_msgs::Image>(dm_.image_name(), 2);
    cam_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(dm_.image_info_name(), 2);
    // timer->start(dm_.frameduration());


    this->ui->idx_slider->setRange(0, dm_.end_idx());
    this->ui->idx_label->setText(QString("0 / ")+QString::number(dm_.end_idx()));
    this->ui->name_label->setText(QString::fromStdString(dm_.image_name()));
    this->ui->size_label->setText(QString::number(dm_.height())+QString(" x ")+QString::number(dm_.width()));
    this->ui->framerate_box->setValue(static_cast<int>(dm_.framerate()));
}

void MainWindow::timer_slot()
{
    if (dm_.idx() == dm_.end_idx()) return;

    int32_t begin = ros::Time::now().toNSec();
    publish_camera_info();
    publish_image();
    this->ui->idx_slider->setValue(dm_.idx());

    dm_.idx_up();

    dm_.prepare_image();
    dm_.prepare_cam_info();
    int32_t end = ros::Time::now().toNSec();

    qDebug() << "Processing time " << static_cast<double>(end - begin) / 10e9;
}

void MainWindow::publish_image()
{
    this->ui->img_label->setPixmap(dm_.qimage().scaled(this->ui->img_label->size()));
    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dm_.image()).toImageMsg();
    QStringList image_list = dm_.image_list();
    QString image_string = image_list[dm_.idx()].left(19);
    image_msg->header.stamp.fromNSec(image_string.toLongLong());
    image_pub_.publish(image_msg);
    ros::spinOnce();
}

void MainWindow::publish_camera_info()
{
    cam_info_pub_.publish(dm_.camera_info());
}

void MainWindow::on_idx_slider_sliderPressed()
{
    timer->stop();
}

void MainWindow::on_idx_slider_sliderReleased()
{
    int idx = this->ui->idx_slider->value();
    dm_.set_index(idx);
    timer->start(dm_.frameduration());
}

void MainWindow::on_idx_slider_valueChanged(int value)
{
    this->ui->idx_label->setText(QString::number(dm_.idx()) + QString(" / ") + QString::number(dm_.end_idx()));
}

void MainWindow::on_framerate_box_editingFinished()
{
    int value = this->ui->framerate_box->value();
    timer->stop();
    dm_.framerate(static_cast<double>(value));
    timer->start(dm_.frameduration());
}

void MainWindow::on_start_button_clicked(bool checked)
{
    if (checked) {
        this->ui->start_button->setCheckable(false);
        timer->stop();
        this->ui->start_button->setText(QString("START"));
    }
    else {
        this->ui->start_button->setCheckable(true);
        timer->start(dm_.frameduration());
        this->ui->start_button->setText(QString("PAUSE"));
    }
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    qDebug("keyPressEvent(%x)", event->key());
}
