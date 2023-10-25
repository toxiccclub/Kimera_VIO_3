//
// Created by Victor Kats on 12.06.2023
//

#ifndef ODOMETRY_CAPTURESETTINGS_H
#define ODOMETRY_CAPTURESETTINGS_H

#include "CameraSettings.h"

#include <string>
#include <vector>

namespace VIO{
namespace capture{

struct Cameras{
    CameraSettings left;
    CameraSettings right;
};

class CaptureSettings{
public:
    explicit CaptureSettings(const Cameras &cameras, bool externalPPS, bool stereo);
    const CameraSettings &getLeft() const;
    const CameraSettings &getRight() const;

    const bool isPPSExternal() const;

    const bool isStereo() const;

private:
    Cameras _cameras;
    bool _externalPPS;
    bool _stereo;
};

}
}

#endif //ODOMETRY_CAPTURESETTINGS_H