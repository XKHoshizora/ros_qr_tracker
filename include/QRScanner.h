#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <zbar.h>

class QRScanner {
  public:
    QRScanner();
    ~QRScanner();
    void run();

  private:
    cv::VideoCapture cap;
    zbar::ImageScanner scanner;

    bool initializeCamera();
    bool captureFrame(cv::Mat &frame);
    void processFrame(cv::Mat &frame);
    void drawQRCode(cv::Mat &frame, const zbar::Symbol &symbol);
    cv::Point calculateCenter(const std::vector<cv::Point> &points);
    void drawQRText(cv::Mat &frame, const std::string &qr_data,
                    const cv::Point &center);
    cv::Point determineTextPosition(const cv::Mat &frame,
                                    const cv::Point &center,
                                    const cv::Size &textSize);
    void displayFrame(const cv::Mat &frame);
};