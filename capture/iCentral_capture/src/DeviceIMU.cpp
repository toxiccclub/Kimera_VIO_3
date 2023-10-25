#include "DeviceIMU.h"

#include "log.h"

#include <boost/asio/read.hpp>

#include <fmt/core.h>

#include <iostream>

using namespace std::literals::chrono_literals;

VIO::capture::DeviceIMU::DeviceIMU(const std::string& port, unsigned int rate):
_port(port), _rate(rate), _interface("UART"), _command(0xFFFF) {
    int no;
    sscanf(_port.c_str(),"%*[^0123456789]%d", &no);
    _id = fmt::format("IMU-{}",no);

}

VIO::capture::DeviceIMU::~DeviceIMU() {
    close();
}

bool VIO::capture::DeviceIMU::open() {
    ioservice.reset(new boost::asio::io_service());
    serial.reset(new boost::asio::serial_port(*ioservice, _port));

    if(serial->is_open()) {
            // Configure basic serial port parameters: 115.2kBaud, 8N1
        serial->set_option(boost::asio::serial_port_base::baud_rate(_rate));
        serial->set_option(boost::asio::serial_port_base::character_size(8 /* data bits */));
        serial->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial->set_option(boost::asio::serial_port_base::stop_bits(
        boost::asio::serial_port_base::stop_bits::one));

        ::tcflush(serial->lowest_layer().native_handle(), TCIOFLUSH);
        FrameCallback x = [](const void *, const IMV_FrameInfo *)->void{
            CONSOLE("First IMU frame incoming");
        };
        while(!nextFrame(x,1000)){}
        nextFrame(x,1000);
        return true;
    }
    ioservice.reset();
    serial.reset();
    return false;
}

bool VIO::capture::DeviceIMU::close() {
    serial->close();
    serial.reset();
    ioservice.reset();
    return true;
}

const std::string& VIO::capture::DeviceIMU::getInterfaceId() const {
    return _interface;;
}

const std::string& VIO::capture::DeviceIMU::getId() const {
    return _id;
}

bool VIO::capture::DeviceIMU::startAcquisition() {
    return serial->is_open();
}

void VIO::capture::DeviceIMU::stopAcquisition() {}

bool VIO::capture::DeviceIMU::nextFrame(FrameCallback& callback) {
    return nextFrame(callback, std::numeric_limits<unsigned int>::max());
}

bool VIO::capture::DeviceIMU::nextFrame(FrameCallback& callback,
                                        unsigned int timeout) {
    if(!serial->is_open()) {
        return false;
    }
    ::tcflush(serial->lowest_layer().native_handle(), TCIOFLUSH);
    auto r = serial->write_some(boost::asio::buffer(&_command, 2));
    boost::asio::streambuf buffer;
    bool frameProcessed = false;
    //boost::asio::read_until(*serial, buffer, "\r\n");
    boost::asio::async_read_until(*serial, buffer, "\r\n", [&](boost::system::error_code ec, size_t bytes_transferred) {
        if (ec) {
            TRACE_N_CONSOLE(WARNING, "Error receiving message: %s", ec.message().c_str());
        }
        IMV_FrameInfo *frameInfo = new IMV_FrameInfo();
        int16_t mock;

        sscanf((const char*)buffer.data().data(), "%lu,%hd,%hd,%hd,%hd,%hd,%hd",
            &frameInfo->timeStamp, &mock, &mock, &mock, &mock, &mock, &mock);

        printf("[%lu]  IMU says: %s", frameInfo->timeStamp, (const char*)buffer.data().data());
        frameInfo->timeStamp *= 1000;
        frameInfo->size = buffer.data().size();
        frameInfo->height = 1;
        frameInfo->width = buffer.data().size();
        callback(buffer.data().data(), frameInfo);
        delete frameInfo;
        frameProcessed = true;
    });

    await_operation(std::chrono::milliseconds(timeout), *serial);

    return frameProcessed;
}
