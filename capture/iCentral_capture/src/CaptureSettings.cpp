#include "CaptureSettings.h"

VIO::capture::CaptureSettings::CaptureSettings(const Cameras& cameras,
                                               bool externalPPS,
                                               bool stereo):
                                               _cameras(cameras),
                                               _externalPPS(externalPPS),
                                               _stereo(stereo) {}

const VIO::capture::CameraSettings& VIO::capture::CaptureSettings::getLeft() const {
    return _cameras.left;
}

const VIO::capture::CameraSettings& VIO::capture::CaptureSettings::getRight() const {
    return _cameras.right;
}

const bool VIO::capture::CaptureSettings::isPPSExternal() const{
    return _externalPPS;
}

const bool VIO::capture::CaptureSettings::isStereo() const {
    return _stereo;
}
