#include "CameraSettings.h"

VIO::capture::CameraSettings::CameraSettings(const std::string& ip,
                                             bool isMaster,
                                             size_t width,
                                             size_t height,
                                             size_t bpp,
                                             const std::string& settingsFile):
                                             _ip(ip),
                                             _master(isMaster),
                                             _width(width),
                                             _height(height),
                                             _bpp(bpp),
                                             _settingsFile(settingsFile) {}

VIO::capture::CameraSettings::CameraSettings(const std::string& ip,
                                             const std::string& interface,
                                             bool isMaster,
                                             size_t width,
                                             size_t height,
                                             size_t bpp,
                                             const std::string& settingsFile):
                                             _ip(ip),
                                             _interface(interface),
                                             _master(isMaster),
                                             _width(width),
                                             _height(height),
                                             _bpp(bpp),
                                             _settingsFile(settingsFile){}

const std::string& VIO::capture::CameraSettings::getIp() const {
    return _ip;
}

const std::string& VIO::capture::CameraSettings::getInterface() const {
    return _interface;
}

bool VIO::capture::CameraSettings::isMaster() const {
    return _master;
}

size_t VIO::capture::CameraSettings::getWidth() const {
    return _width;
}

size_t VIO::capture::CameraSettings::getHeight() const {
    return _height;
}

size_t VIO::capture::CameraSettings::getBpp() const {
    return _bpp;
}

const std::string& VIO::capture::CameraSettings::getSettingsFile() const {
    return _settingsFile;
}
