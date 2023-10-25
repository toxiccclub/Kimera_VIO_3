//
// Created by victor on 14.06.2023.
//

#ifndef ODOMETRY_DEVICE_H
#define ODOMETRY_DEVICE_H

#include "FrameHeader.h"

#include <IMVApi.h>

#include <functional>
#include <map>
#include <memory>
#include <string>

namespace VIO{
namespace capture{

using FrameCallback = std::function<void(const void *, const IMV_FrameInfo *)>;

class Device {
    public:
    //Device(const std::tuple<std::string, std::string, unsigned int> &descriptor) {};
    //virtual ~Device() = 0;

    virtual bool open() = 0;
    virtual bool close() = 0;
    virtual const std::string &getInterfaceId() const = 0;
    virtual const std::string &getId() const = 0;

    virtual bool startAcquisition() = 0;
    //bool startAcquisition(const std::vector<std::shared_ptr<unsigned char>> &pool, size_t size);
    virtual void stopAcquisition() = 0;

    virtual bool nextFrame(FrameCallback &callback) = 0;

    virtual bool nextFrame(FrameCallback &callback, unsigned int timeout) = 0;
};

class DeviceGenIApi:public Device {
public:
    DeviceGenIApi(const std::tuple<std::string, std::string, unsigned int> &descriptor);
    ~DeviceGenIApi();

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
    std::tuple<std::string, std::string, unsigned int> _descriptor;
    IMV_HANDLE _devHandle;
};
}
}

#endif //ODOMETRY_DEVICE_H