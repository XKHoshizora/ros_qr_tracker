#include "QRScanner.h"
#include <iostream>

QRScanner::QRScanner() : cap(0) {
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
}

QRScanner::~QRScanner() { cap.release(); }

void QRScanner::run() {
    std::cout << "Entering QRScanner::run()" << std::endl;
    if (!cap.isOpened()) {
        std::cerr << "Error: Unable to open camera" << std::endl;
        return;
    }
    std::cout << "Camera opened successfully" << std::endl;

    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Error: Unable to capture frame" << std::endl;
            break;
        }
        std::cout << "Frame captured" << std::endl;

        processFrame(frame);

        cv::imshow("Camera", frame);
        std::cout << "Frame displayed" << std::endl;

        if (cv::waitKey(1) == 'q') {
            std::cout << "User pressed 'q'. Exiting..." << std::endl;
            break;
        }
    }
    std::cout << "Exiting QRScanner::run()" << std::endl;
}

void QRScanner::processFrame(cv::Mat &frame) {
    cv::Mat grey;
    cv::cvtColor(frame, grey, cv::COLOR_BGR2GRAY);

    zbar::Image image(frame.cols, frame.rows, "Y800", grey.data,
                      frame.cols * frame.rows);

    decodeAndDraw(frame, image);
}

void QRScanner::decodeAndDraw(cv::Mat &frame, zbar::Image &image) {
    scanner.scan(image);

    for (zbar::Image::SymbolIterator symbol = image.symbol_begin();
         symbol != image.symbol_end(); ++symbol) {
        std::cout << "二维码内容: " << symbol->get_data() << std::endl;

        std::vector<cv::Point> points;
        for (int i = 0; i < symbol->get_location_size(); i++) {
            points.emplace_back(symbol->get_location_x(i),
                                symbol->get_location_y(i));
        }

        cv::polylines(frame, points, true, cv::Scalar(0, 255, 0), 2);
        cv::putText(frame, symbol->get_data(), points[0],
                    cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(255, 0, 0), 2);
    }
}