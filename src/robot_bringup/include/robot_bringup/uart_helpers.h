#ifndef UART_HELPERS_H
#define UART_HELPERS_H

#include <string>
#include <serial/serial.h>

class UARTHelpers {
public:
    UARTHelpers(const std::string& port, unsigned int baud_rate);
    ~UARTHelpers();

    void openPort();
    void closePort();
    size_t available();
    size_t read(unsigned char* buffer, size_t size);
    void sendData(const std::vector<uint8_t>& data);

private:
    serial::Serial serial_;
};

#endif // UART_HELPERS_H
