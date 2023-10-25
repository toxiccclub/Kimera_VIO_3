//
// Created by victor on 14.06.2023.
//

#ifndef ODOMETRY_DEVICELIST_H
#define ODOMETRY_DEVICELIST_H

//#include "GenTLWrapper/System.h"

#include "Device.h"

#include <IMVApi.h>

#include <map>
#include <memory>
#include <string>
#include <unordered_map>


namespace VIO{
namespace capture{

//struct Interface;

class DeviceList {
public:
    explicit DeviceList(/*tmk::capture::LayerImplementation &layer,
                        std::shared_ptr<tmk::capture::System> system*/);

    DeviceList(const DeviceList&)=delete;
    DeviceList& operator=(const DeviceList&)=delete;

    DeviceList(DeviceList&&)=default;
    DeviceList& operator=(DeviceList&&)=default;

    ~DeviceList();
    bool rescan();

    bool contains(unsigned int ip);
    bool contains(const std::string &ip);

    std::tuple<std::string, std::string, unsigned int> findDevice(unsigned int ip, unsigned int interfaceIp);
    std::tuple<std::string, std::string, unsigned int> findDevice(const std::string &ip, const std::string &interfaceIp);

    std::shared_ptr<DeviceGenIApi> getCamera(unsigned int ip);
    std::shared_ptr<DeviceGenIApi> getCamera(const std::string &ip);

    std::shared_ptr<DeviceGenIApi> getCamera(unsigned int ip, unsigned int interfaceIp);
    std::shared_ptr<DeviceGenIApi> getCamera(const std::string &ip, const std::string &interfaceIp);
private:
    std::unordered_map<unsigned int, std::tuple<std::string, std::string, unsigned int>> _devices;
    std::map<unsigned int, std::string> _interfacesByIp;

    //tmk::capture::LayerImplementation &_layer;
    //std::shared_ptr<tmk::capture::System> _system;

    std::unordered_map<unsigned int, std::shared_ptr<DeviceGenIApi>> _devicesInUse;
    //std::map<std::string, std::shared_ptr<Interface>> _interfacesInUse;

    IMV_DeviceList _deviceInfoList;
};

}
}

#endif //ODOMETRY_DEVICELIST_H
