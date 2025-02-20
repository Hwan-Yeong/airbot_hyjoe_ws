#include "robot_bringup/uart_helpers.h"

UARTHelpers::UARTHelpers(const std::string& port, unsigned int baud_rate)
    : serial_(port, baud_rate) {}

UARTHelpers::~UARTHelpers() {
    if (serial_.isOpen()) {
        serial_.close();
    }
}

void UARTHelpers::openPort() {
    if (!serial_.isOpen()) {
        serial_.open();
    }
}

void UARTHelpers::closePort() {
    if (serial_.isOpen()) {
        serial_.close();
    }
}

size_t UARTHelpers::available() {
    return serial_.available();
}

size_t UARTHelpers::read(unsigned char* buffer, size_t size) {
    return serial_.read(buffer, size);
}

void UARTHelpers::sendData(const std::vector<uint8_t>& data) {
    serial_.write(data);
}
