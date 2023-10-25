//
// Created by Victor Kats on 12.06.2023
//

#ifndef ODOMETRY_CAMERASETTINGS_H
#define ODOMETRY_CAMERASETTINGS_H

#include<string>

namespace VIO{
namespace capture{
class CameraSettings{
public:
    CameraSettings(const std::string &ip, bool isMaster, size_t width, size_t height, size_t bpp,
    const std::string &settingsFile);
    CameraSettings(const std::string &ip, const std::string &interface, bool isMaster, size_t width, size_t height, size_t bpp,
    const std::string &settingsFile);
    //CameraSettings(nlohmann::json &from);

    const std::string &getIp() const;
    const std::string &getInterface() const;

    bool isMaster() const;

    //const std::string &getRingBufferName() const;

    size_t getWidth() const;

    size_t getHeight() const;

    size_t getBpp() const;

    const std::string &getSettingsFile() const;

private:
    std::string _ip;
    std::string _interface;
    bool _master;

    //std::string _ringBufferName;

    size_t _width;
    size_t _height;
    size_t _bpp;

    std::string _settingsFile;
//public:
//    bool isValidateAcquisitionStart() const;
};
}
}

#endif //ODOMETRY_CAMERASETTINGS_H