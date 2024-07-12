#include "QRScanner.h"
#include <iostream>
#include <numeric>

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
    scanner.scan(image);

    for (zbar::Image::SymbolIterator symbol = image.symbol_begin();
         symbol != image.symbol_end(); ++symbol) {
        std::vector<cv::Point> points;
        for (int i = 0; i < symbol->get_location_size(); i++) {
            points.push_back(cv::Point(symbol->get_location_x(i),
                                       symbol->get_location_y(i)));
        }

        // 绘制 QR 码边界
        cv::polylines(frame, points, true, cv::Scalar(0, 255, 0), 2);

        // 计算 QR 码中心点
        cv::Point center(0, 0);
        for (const auto &point : points) {
            center += point;
        }
        center.x /= points.size();
        center.y /= points.size();

        // 准备显示文本
        std::string qr_data = symbol->get_data();

        // 增大文本大小和相关参数
        int fontFace = cv::FONT_HERSHEY_SIMPLEX;
        double fontScale = 0.8; // 增大字体大小
        int thickness = 2;      // 增加文本粗细
        int baseline = 0;
        cv::Size textSize =
            cv::getTextSize(qr_data, fontFace, fontScale, thickness, &baseline);

        cv::Point textOrg;
        if (center.x < frame.cols / 2) {
            textOrg = cv::Point(center.x, center.y - textSize.height);
        } else {
            textOrg = cv::Point(center.x - textSize.width,
                                center.y - textSize.height);
        }

        // 确保文本在图像内
        textOrg.x =
            std::max(0, std::min(textOrg.x, frame.cols - textSize.width));
        textOrg.y = std::max(textSize.height,
                             std::min(textOrg.y, frame.rows - baseline));

        // 绘制更大的半透明背景
        cv::Rect backgroundRect(textOrg.x - 5, textOrg.y - textSize.height - 5,
                                textSize.width + 10,
                                textSize.height + baseline + 10);
        cv::rectangle(frame, backgroundRect, cv::Scalar(0, 0, 0, 180),
                      cv::FILLED);

        // 绘制更大的文本
        cv::putText(frame, qr_data, textOrg, fontFace, fontScale,
                    cv::Scalar(255, 255, 255), thickness, cv::LINE_AA);
    }
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