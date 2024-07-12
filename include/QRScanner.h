#pragma once

#include <opencv2/opencv.hpp>
#include <zbar.h>

class QRScanner {
  public:
    QRScanner();
    ~QRScanner();
    void run();

  private:
    cv::VideoCapture cap;
    zbar::ImageScanner scanner;

    void processFrame(cv::Mat &frame);
    void decodeAndDraw(cv::Mat &frame, zbar::Image &image);
};