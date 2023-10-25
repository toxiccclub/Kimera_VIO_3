#include "Device.h"

#include <boost/asio.hpp>

#include <memory>

namespace VIO{
namespace capture{

class DeviceIMU:public Device {
public:
    DeviceIMU(const std::string &port, unsigned int rate);
    ~DeviceIMU();

    bool open() override;
    bool close() override;
    const std::string &getInterfaceId() const override;
    const std::string &getId() const override;

    bool startAcquisition() override;
    //bool startAcquisition(const std::vector<std::shared_ptr<unsigned char>> &pool, size_t size);
    void stopAcquisition() override;

    bool nextFrame(FrameCallback &callback) override;

    bool nextFrame(FrameCallback &callback, unsigned int timeout) override;

private:
    std::string _port;
    unsigned int _rate;

    std::shared_ptr<boost::asio::io_service> ioservice;
  // Open serial port
    std::shared_ptr<boost::asio::serial_port> serial;

    std::string _id;
    std::string _interface;

    uint16_t _command;

    template<typename AllowTime, typename Cancel> void await_operation_ex(AllowTime const& deadline_or_duration, Cancel&& cancel) {
        using namespace boost::asio;
        using error_code = boost::system::error_code;

        ioservice->reset();
        {
            high_resolution_timer tm(*ioservice, deadline_or_duration);
            tm.async_wait([&cancel](error_code ec) { if (ec != error::operation_aborted) std::forward<Cancel>(cancel)(); });
            ioservice->run_one();
        }
        ioservice->run();
    }

    template<typename AllowTime, typename ServiceObject> void await_operation(AllowTime const& deadline_or_duration, ServiceObject& so) {
        return await_operation_ex(deadline_or_duration, [&so]{ so.cancel(); });
    }

};
}
}