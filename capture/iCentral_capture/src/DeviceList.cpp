#include "DeviceList.h"

#include "log.h"

#include <boost/asio.hpp>

#include <fmt/core.h>

namespace {
inline uint ip2uint(const std::string &ip) {
    uint uintIp = 0;
    try {
        uintIp = boost::asio::ip::make_address_v4(ip).to_uint();
    }
    catch(...) {
        uintIp = 0;
    }
    return uintIp;
}

inline std::string uint2ip(uint ip) {
    std::string strIp = "0.0.0.0";
    try {
        strIp = boost::asio::ip::make_address_v4(ip).to_string();
    }
    catch (...) {
        strIp = "0.0.0.0";
    }
    return strIp;
}
}

VIO::capture::DeviceList::DeviceList() {}

VIO::capture::DeviceList::~DeviceList() {
    if(!_devicesInUse.empty()) {
        for(auto &d: _devicesInUse) {
            _devicesInUse[d.first].reset();
        }
        _devicesInUse.clear();
    }
    /*if(!_interfacesInUse.empty()) {
        for(auto &i: _interfacesInUse) {
            _interfacesInUse[i.first].reset();
        }
        _interfacesInUse.clear();
    }*/

    if(_deviceInfoList.nDevNum) {
        delete [] _deviceInfoList.pDevInfo;
    }
}

bool VIO::capture::DeviceList::rescan() {
    _devices.clear();
    _interfacesByIp.clear();

    if(_deviceInfoList.nDevNum) {
        delete [] _deviceInfoList.pDevInfo;
    }

    auto ret = IMV_EnumDevices(&_deviceInfoList,  _IMV_EInterfaceType::interfaceTypeGige);

    if (IMV_OK != ret)
	{
		//printf("Enumeration devices failed! ErrorCode[%d]\n", ret);
		return false;
	}
	if (_deviceInfoList.nDevNum < 1)
	{
		//printf("no camera\n");
		return false;
	}

    for(unsigned int i = 0; i < _deviceInfoList.nDevNum; ++i) {
        auto dev = _deviceInfoList.pDevInfo[i];

        auto devIp = ip2uint(dev.DeviceSpecificInfo.gigeDeviceInfo.ipAddress);
        auto devNetmask = ip2uint(dev.DeviceSpecificInfo.gigeDeviceInfo.subnetMask);

        auto interfaceIp = ip2uint(dev.InterfaceInfo.gigeInterfaceInfo.ipAddress);
        auto interfaceNetmask = ip2uint(dev.InterfaceInfo.gigeInterfaceInfo.subnetMask);

        auto netId = interfaceIp & interfaceNetmask;
        if((devIp & interfaceNetmask) == netId) {
                if(_devices.contains(devIp)) {
                    TRACE_N_CONSOLE(WARNING, "%s",
                                    fmt::format("Device {} already noted on interface {}, id {}. "
                                                         "Alternative interface {}}",
                                          dev.DeviceSpecificInfo.gigeDeviceInfo.ipAddress,
                                          std::get<0>(_devices.at(devIp)), std::get<1>(_devices.at(devIp)),
                                          dev.InterfaceInfo.gigeInterfaceInfo.ipAddress).c_str());
                }
            }
            else {
                TRACE_N_CONSOLE(WARNING, "%s",
                                fmt::format("Device {} is foreign for interface {}.",
                                            dev.DeviceSpecificInfo.gigeDeviceInfo.ipAddress,
                                            dev.InterfaceInfo.gigeInterfaceInfo.ipAddress).c_str());
            }
            _devices.insert({devIp, std::make_tuple(dev.InterfaceInfo.gigeInterfaceInfo.macAddress,
            dev.DeviceSpecificInfo.gigeDeviceInfo.macAddress, i)});
    }

    return !_devices.empty();
}

bool VIO::capture::DeviceList::contains(unsigned int ip) {
    return _devices.contains(ip);
}

bool VIO::capture::DeviceList::contains(const std::string& ip) {
    return _devices.contains(ip2uint(ip));
}

std::tuple<std::string, std::string, unsigned int> VIO::capture::DeviceList::findDevice(
    unsigned int ip,
    unsigned int interfaceIp) {
    if(!_devices.contains(ip)) {
        TRACE_N_CONSOLE(WARNING, "%s", fmt::format("Camera with ip {} not listed", uint2ip(ip)).c_str());
        return {"", "", 0};
    }
    size_t id = _devices.bucket(ip);

    if(!interfaceIp) {
        TRACE_N_CONSOLE(INFO, "%s", fmt::format("Looking for camera with ip {} on all interfaces", uint2ip(ip)).c_str());
        auto devicesCount = _devices.bucket_size(id);
        if(devicesCount > 1) {
            TRACE_N_CONSOLE(WARNING, "%s", fmt::format("{} devices with ip {} detected. "
                                     "Interface Id required to select exact device. "
                                     "First device in buket will be returned",
                            devicesCount, uint2ip(ip)).c_str());
        }
        return _devices.begin(id)->second;
    }

    if(!_interfacesByIp.contains(interfaceIp)) {
        TRACE_N_CONSOLE(WARNING, "%s", fmt::format("Interface with ip {} not listed", uint2ip(interfaceIp)).c_str());
        return {"", "", 0};
    }
    TRACE_N_CONSOLE(INFO, "%s", fmt::format("Looking for camera with ip {} through interface with ip {}", uint2ip(ip), uint2ip(interfaceIp)).c_str());
    auto interface_ = _interfacesByIp.at(interfaceIp);

    auto result = std::find_if(_devices.begin(id), _devices.end(id),
                               [interface_](auto it) {
        return std::get<0>(it.second) == interface_;
    });

    if(result != _devices.end(id)) {
        return result->second;
    }
    TRACE_N_CONSOLE(WARNING, "%s", fmt::format("Camera with ip {} not listed for interface with ip {}", uint2ip(ip), uint2ip(interfaceIp)).c_str());
    return {"", "", 0};
}

std::tuple<std::string, std::string, unsigned int> VIO::capture::DeviceList::findDevice(
    const std::string& ip,
    const std::string& interfaceIp) {
    return findDevice(ip2uint(ip), ip2uint(interfaceIp));
}

std::shared_ptr<VIO::capture::DeviceGenIApi> VIO::capture::DeviceList::getCamera(unsigned int ip) {
    return getCamera(ip, 0);
}

std::shared_ptr<VIO::capture::DeviceGenIApi> VIO::capture::DeviceList::getCamera(const std::string &ip){
    return getCamera(ip2uint(ip));
}

std::shared_ptr<VIO::capture::DeviceGenIApi> VIO::capture::DeviceList::getCamera(unsigned int ip,
                                              unsigned int interfaceIp) {
    std::tuple<std::string, std::string, unsigned int> descriptor = findDevice(ip, interfaceIp);

    if(std::get<0>(descriptor).empty() || std::get<0>(descriptor).empty()) {
        return nullptr;
    }

    if(_devicesInUse.contains(ip)) {
        size_t id = _devicesInUse.bucket(ip);
        for(auto it = _devicesInUse.begin(id); it != _devicesInUse.end(id); ++it) {
            if(it->second->getInterfaceId() == std::get<0>(descriptor)) {
                return it->second;
            }
        }
    }

    std::shared_ptr<VIO::capture::DeviceGenIApi> device;
    device.reset(new VIO::capture::DeviceGenIApi(descriptor));
    if(device->open()) {
        _devicesInUse[ip] = nullptr;
        _devicesInUse.at(ip).swap(device);
        return _devicesInUse.at(ip);
    }

    /*if(!_interfacesInUse.contains(descriptor.first)) {
        _interfacesInUse[descriptor.first] = _layer.openInterface(descriptor.first);
    }
    auto device = _interfacesInUse[descriptor.first]->openDevice(descriptor.second);
    if(device) {
        _devicesInUse[ip] = nullptr;
        _devicesInUse.at(ip).swap(device);
        _devicesInUse.at(ip)->setInterfaceId(descriptor.first);
        return _devicesInUse.at(ip);
    }*/
    TRACE(WARNING, "Attempt to activate device %s on interface %s failed",
          uint2ip(ip), interfaceIp ? uint2ip(interfaceIp) : "ANY");

    return nullptr;
}

std::shared_ptr<VIO::capture::DeviceGenIApi> VIO::capture::DeviceList::getCamera(const std::string& ip,
                                              const std::string& interfaceIp) {
    return getCamera(ip2uint(ip), ip2uint(interfaceIp));
}
