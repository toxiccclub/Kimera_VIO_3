//
// Created by Victor Kats on 12.06.2023
//

#ifndef ODOMETRY_CAMERACAPTURESERVICE_H
#define ODOMETRY_CAMERACAPTURESERVICE_H

//#include <libcommon/CaptureServiceSettings.h>

//#include <libinfrastructure/channel_manager/OutputRBHolder.h>

//#include <GenTLWrapper/LayerImplementation.h>

//#include <GenApiUtils/AcquisitionThread.h>
//#include <GenApiUtils/ChannelManager.h>
//#include <GenApiUtils/DeviceList.h>

#include "AcquisitionThread.h"
#include "CaptureSettings.h"
#include "DeviceList.h"

#include <memory>
#include <thread>

#include <boost/atomic.hpp>

//void run_capture(const tmk::common::configuration::CaptureServiceSettings &settings);

namespace VIO{
namespace capture{
class CameraCaptureService {
public:
    explicit CameraCaptureService(const CaptureSettings &settings,
        const std::string &storagePath,
        const std::string &imuPort, unsigned int imuRate,
        neosmart::neosmart_event_t *failureEvent);
    ~CameraCaptureService();
    bool start();
    bool pause();
    bool resume();
    bool stop();
private:
    bool addCamera(const CameraSettings &cam, bool addMaster, size_t camNum);

    bool addIMU(const std::string &port, unsigned int rate, size_t camNum);

    const CaptureSettings &_settings;
    const CameraSettings &_leftCamera;
    const CameraSettings &_rightCamera;

    boost::atomic<bool> _masterRun;
    boost::atomic<bool> _slavesRun;

    boost::atomic<bool> _pause;

    //tmk::capture::LayerImplementation _layer;
    //std::shared_ptr<tmk::capture::System> _system;
    std::shared_ptr<DeviceList> _devices;

    std::shared_ptr<AcquisitionThread> _masterCamera;
    std::vector<std::shared_ptr<AcquisitionThread>> _slaveCameras;

    std::shared_ptr<std::thread> _masterAcq;
    std::vector<std::shared_ptr<std::thread>> _slavesAcq;

    //tmk::infrastructure::channel_manager::RBHolder& _rbHolder;

    std::vector<std::shared_ptr<std::thread>> _failureMonitors;

    std::shared_ptr<std::thread> _stopper;

    std::string _storagePath;

    const std::string _imuPort;

    unsigned int _imuRate;

    neosmart::neosmart_event_t *_failureEvent;
};

}
}

#endif //ODOMETRY_CAMERACAPTURESERVICE_H