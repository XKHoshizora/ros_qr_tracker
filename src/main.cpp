// main.cpp

#include "QRScanner.h"
#include <iostream>

int main() {
    std::cout << "Starting QR Scanner..." << std::endl;
    QRScanner scanner;
    std::cout << "QR Scanner initialized." << std::endl;
    scanner.run();
    std::cout << "QR Scanner finished." << std::endl;
    return 0;
}