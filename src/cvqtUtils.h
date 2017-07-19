#ifndef CVQTUTILS_H
#define CVQTUTILS_H

#include <QImage>
#include <QPixmap>
#include <QDebug>
#include <opencv2/core.hpp>

QImage cvMatToQImage( const cv::Mat &inMat );

QPixmap cvMatToQPixmap( const cv::Mat &inMat );

#endif // CVQTUTILS_H
