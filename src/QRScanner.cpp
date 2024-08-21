#include "QRScanner.h"
#include <chrono>
#include <iostream>
#include <numeric>
#include <thread>

QRScanner::QRScanner() : cap(0) {
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
}

QRScanner::~QRScanner() { cap.release(); }

void QRScanner::run() {
    if (!initializeCamera()) {
        return;
    }

    cv::Mat frame;
    while (true) {
        if (!captureFrame(frame)) {
            break;
        }

        processFrame(frame);
        displayFrame(frame);

        if (cv::waitKey(1) == 'q') {
            std::cout << "User pressed 'q'. Exiting..." << std::endl;
            break;
        }
    }
}

bool QRScanner::initializeCamera() {
    std::cout << "Initializing camera..." << std::endl;
    for (int i = 0; i < 5; ++i) {
        if (cap.open(0)) {
            std::cout << "Camera opened successfully" << std::endl;
            return true;
        }
        std::cerr << "Failed to open camera. Retrying..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cerr << "Error: Unable to open camera after multiple attempts"
              << std::endl;
    return false;
}

bool QRScanner::captureFrame(cv::Mat &frame) {
    cap >> frame;
    if (frame.empty()) {
        std::cerr << "Error: Unable to capture frame" << std::endl;
        return false;
    }
    return true;
}

void QRScanner::processFrame(cv::Mat &frame) {
    cv::Mat grey;
    cv::cvtColor(frame, grey, cv::COLOR_BGR2GRAY);

    zbar::Image image(frame.cols, frame.rows, "Y800", grey.data,
                      frame.cols * frame.rows);
    scanner.scan(image);

    for (zbar::Image::SymbolIterator symbol = image.symbol_begin();
         symbol != image.symbol_end(); ++symbol) {
        drawQRCode(frame, *symbol);
    }
}

void QRScanner::drawQRCode(cv::Mat &frame, const zbar::Symbol &symbol) {
    std::vector<cv::Point> points;
    for (int i = 0; i < symbol.get_location_size(); i++) {
        points.push_back(
            cv::Point(symbol.get_location_x(i), symbol.get_location_y(i)));
    }

    cv::polylines(frame, points, true, cv::Scalar(0, 255, 0), 2);

    cv::Point center = calculateCenter(points);
    std::string qr_data = symbol.get_data();

    drawQRText(frame, qr_data, center);
}

cv::Point QRScanner::calculateCenter(const std::vector<cv::Point> &points) {
    return std::accumulate(points.begin(), points.end(), cv::Point(0, 0)) /
           static_cast<int>(points.size());
}

void QRScanner::drawQRText(cv::Mat &frame, const std::string &qr_data,
                           const cv::Point &center) {
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.8;
    int thickness = 2;
    int baseline = 0;
    cv::Size textSize =
        cv::getTextSize(qr_data, fontFace, fontScale, thickness, &baseline);

    cv::Point textOrg = determineTextPosition(frame, center, textSize);

    cv::Rect backgroundRect(textOrg.x - 5, textOrg.y - textSize.height - 5,
                            textSize.width + 10,
                            textSize.height + baseline + 10);
    cv::rectangle(frame, backgroundRect, cv::Scalar(0, 0, 0, 180), cv::FILLED);

    cv::putText(frame, qr_data, textOrg, fontFace, fontScale,
                cv::Scalar(255, 255, 255), thickness, cv::LINE_AA);
}

cv::Point QRScanner::determineTextPosition(const cv::Mat &frame,
                                           const cv::Point &center,
                                           const cv::Size &textSize) {
    cv::Point textOrg;
    if (center.x < frame.cols / 2) {
        textOrg = cv::Point(center.x, center.y - textSize.height);
    } else {
        textOrg =
            cv::Point(center.x - textSize.width, center.y - textSize.height);
    }

    textOrg.x = std::max(0, std::min(textOrg.x, frame.cols - textSize.width));
    textOrg.y = std::max(textSize.height,
                         std::min(textOrg.y, frame.rows - textSize.height));

    return textOrg;
}

void QRScanner::displayFrame(const cv::Mat &frame) {
    cv::imshow("Camera", frame);
}