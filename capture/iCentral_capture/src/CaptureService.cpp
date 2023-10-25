#include "CaptureService.h"

#include "DeviceIMU.h"

#include "log.h"

#include <fmt/core.h>

#include <boost/filesystem.hpp>

VIO::capture::CameraCaptureService::CameraCaptureService(
    const CaptureSettings& settings,  const std::string &storagePath,
    const std::string &imuPort, unsigned int imuRate,
    neosmart::neosmart_event_t *failureEvent):
_settings(settings),
_leftCamera(settings.getLeft()),
_rightCamera(settings.getRight()),
_masterRun(false),
_slavesRun(false),
_pause(false),
//_layer(tmk::capture::LayerImplementation(settings.getProvider())),
//_system(_layer.getSystem()),
_devices(new DeviceList()),
_masterAcq(nullptr),
//_rbHolder(rbHolder),
_storagePath(storagePath+boost::filesystem::path::separator),
_imuPort(imuPort),
_imuRate(imuRate),
_failureEvent(failureEvent)
 {}

VIO::capture::CameraCaptureService::~CameraCaptureService() {
    stop();
}

bool VIO::capture::CameraCaptureService::start() {
        TRACE_N_CONSOLE(INFO, "Start to acquire cameras");
    if(_masterRun.load() || _slavesRun.load()) {
        TRACE_N_CONSOLE(INFO, "Acquisition already on-going");
        return false;
    }
    _pause.store(true);
    if(!_devices->rescan()) {
        TRACE_N_CONSOLE(INFO, "Devices enumeration finished with issue");
        return false;
    }
    size_t camNum=0;

    /*if(!_settings.isStereo()) {
        if(!addCamera(_settings.getLeft(), false, 0)) {
            return false;
        }
    }
    else {
        if(!addCamera(_settings.getLeft(), _settings.getLeft().isMaster(), 0) ||
        !addCamera(_settings.getRight(), _settings.getRight().isMaster(), 1)) {
            return false;
        }
    }*/

    if(!addCamera(_settings.getLeft(), false, 0)) {
            return false;
    }
    if(_settings.isStereo()) {
        if(!addCamera(_settings.getRight(), false, 0)) {
            return false;
    }
    }


    addIMU(_imuPort, _imuRate, 2);

    /*if(!_settings.isPPSExternal()) {
        _masterAcq.reset(new std::thread([&](){
        _masterCamera->run();
    }));
    }

    if(_settings.getLeft().isMaster() || _settings.getRight().isMaster()) {
        _failureMonitors.emplace_back(new std::thread([&](){
            if(_masterCamera->monitorFailure() && _failureEvent) {
                neosmart::SetEvent(*_failureEvent);
            }
        }));
    }*/


#ifdef DEBUG_LIVE_FOR_SECONDS
    _stopper.reset(new std::thread([&](){
        sleep(DEBUG_LIVE_FOR_SECONDS);
        neosmart::SetEvent(*_failureEvent);
    }));
#endif //#ifdef DEBUG_LIVE_FOR_SECONDS

    _slavesRun.store(true);
    _slavesRun.notify_all();

    for (auto& slave : _slaveCameras)
    {
        slave->waitForStart(15*1000);
    }
    TRACE_N_CONSOLE(INFO,"All slaves were started");

    /*_masterRun.store(true);
    _masterRun.notify_all();
    TRACE_N_CONSOLE(INFO,"Master was started");
    _masterCamera->waitForFirstFrame(15*1000);*/
    std::vector<uint64_t> corrections;
    std::vector<uint64_t> correctionsUTC;
    /*corrections.push_back(_masterCamera->getFirstAcquisitionTime());
    correctionsUTC.push_back(_masterCamera->getFirstAcquisitionTimeUTC());*/
    //CONSOLE("%llu, %llu", corrections.back(), correctionsUTC.back());
    for (auto& slave : _slaveCameras)  {
        slave->waitForFirstFrame(15*1000);
        corrections.push_back(slave->getFirstAcquisitionTime());
        correctionsUTC.push_back(slave->getFirstAcquisitionTimeUTC());
        //CONSOLE("%llu, %llu", corrections.back(), correctionsUTC.back());
    }
    std::sort(corrections.begin(), corrections.end());
    std::sort(correctionsUTC.begin(), correctionsUTC.end());
    TRACE_N_CONSOLE(INFO, "Initial frames came from cameras in time span form %ld to %ld till epoch",
                   correctionsUTC.front(), correctionsUTC.back());
    CONSOLE("%llu, %llu", corrections.back(), correctionsUTC.front());
    //_masterCamera->updateTimeCorrection(corrections.back(), correctionsUTC.front());
    //for (auto& slave : _slaveCameras) {
    //    slave->updateTimeCorrection(corrections.back(), correctionsUTC.front());
    //}
    resume();
    return true;
}

bool VIO::capture::CameraCaptureService::pause() {
    _pause.store(true);
    _pause.notify_all();
    return _pause.load();
}

bool VIO::capture::CameraCaptureService::resume() {
    _pause.store(false);
    _pause.notify_all();
    return !_pause.load();
}

bool VIO::capture::CameraCaptureService::stop() {
    _failureEvent = nullptr;
    _slavesRun.store(false);
    _slavesRun.notify_all();
    _masterRun.store(false);
    _masterRun.notify_all();

    for (auto c: _slaveCameras) {
        c->resetState();
    }
    if(_masterCamera) {
        _masterCamera->resetState();
    }

    for(auto m: _failureMonitors) {
        m->join();
        m.reset();
    }
    _failureMonitors.clear();

    if(_masterAcq) {
        _masterAcq->join();
        _masterAcq.reset();
    }

    for(auto slave: _slavesAcq) {
        slave->join();
        slave.reset();
    }

    _slavesAcq.clear();
    _masterCamera.reset();
    for (auto c: _slaveCameras) {
        c.reset();
    }

    if(_stopper) {
        _stopper->join();
        _stopper.reset();
    }

    return true;
}

bool VIO::capture::CameraCaptureService::addCamera(const CameraSettings& cam,
                                                   bool addMaster, size_t camNum) {
    AcquisitionThreadProducer acquisitionThreadProducer;
    if (!_devices->contains(cam.getIp())) {
            TRACE_N_CONSOLE(INFO, "No camera with IP %s connected", cam.getIp().c_str());
            stop();
            return false;
        }
        std::shared_ptr<VIO::capture::DeviceGenIApi> camera;
        auto interfaceId = cam.getInterface();
        if(interfaceId.empty() || interfaceId == "any" || interfaceId == "0.0.0.0") {
            camera = _devices->getCamera(cam.getIp());
        }
        else {
            camera = _devices->getCamera(cam.getIp(), interfaceId);
        }
        if(!camera) {
            TRACE_N_CONSOLE(WARNING, "Camera with IP %s connection failure", cam.getIp().c_str() );
            stop();
            return false;
        }
        //camera->getImplementation()->setValidateAcquisitionStart(cam.isValidateAcquisitionStart());

        ///TODO: reimplement
        /*tmk::capture::Configuration deserialized = tmk::capture::CameraConfiguration::deserialize(cam.getDeviceConfig());
        if(!deserialized.empty()) {
            tmk::capture::Configuration invalidFeatures;
            invalidFeatures = camera->getImplementation()->setConfiguration(deserialized);
            if(!invalidFeatures.empty()) {
                TRACE_N_CONSOLE(INFO, "Camera %s rejected %u features", cam.getIp(), invalidFeatures.size());
            }
        }*/

        if (addMaster) {
            //_masterCamera.reset(new AcquisitionThreadVio(camNum,camera, &_masterRun, &_pause, cam.getIp()));
            _masterCamera = acquisitionThreadProducer(camNum,camera, &_masterRun, &_pause,
                _storagePath+cam.getIp());
            /*TRACE_N_CONSOLE(INFO, "Master camera %s. Captures to %s. Internal identity is %s.",
                            cam.getIp().c_str(), cam.getRingBufferName().c_str(), camera->getId());*/
        }
        else {
            //std::shared_ptr<AcquisitionThreadVio> pSlaveAck(
            //    new AcquisitionThreadVio(camNum,camera, &_slavesRun, &_pause, cam.getIp()));
            auto pSlaveAck=acquisitionThreadProducer(camNum,camera, &_slavesRun, &_pause,
                _storagePath+cam.getIp());
            _slaveCameras.push_back(pSlaveAck);
            _slavesAcq.emplace_back(new std::thread([&,this,pSlaveAck]() {
                std::shared_ptr<AcquisitionThread> acq = pSlaveAck;
                acq->run();
                acq.reset();
            }));

            _failureMonitors.emplace_back(new std::thread([&,this,pSlaveAck]() {
                std::shared_ptr<AcquisitionThread> acq = pSlaveAck;
                if(acq->monitorFailure() && _failureEvent) {
                    neosmart::SetEvent(*_failureEvent);
                }
            }));

            /*TRACE_N_CONSOLE(INFO, "Slave camera %s. Captures to %s. Internal identity is %s.",
                            cam.getIp().c_str(), cam.getRingBufferName().c_str(), camera->getId());*/
        }
    if(addMaster) {
        _masterAcq.reset(new std::thread([&](){
            _masterCamera->run();
        }));

        _failureMonitors.emplace_back(new std::thread([&](){
            if(_masterCamera->monitorFailure() && _failureEvent) {
            neosmart::SetEvent(*_failureEvent);
            }
        }));
    }

    return true;
}

bool VIO::capture::CameraCaptureService::addIMU(const std::string& port, unsigned int rate,
                                                size_t camNum) {
    std::shared_ptr<VIO::capture::DeviceIMU> camera(new VIO::capture::DeviceIMU(port, rate));
    AcquisitionThreadProducer acquisitionThreadProducer;
    if(camera->open()) {
        auto pSlaveAck = acquisitionThreadProducer(
          camNum, camera, &_slavesRun, &_pause, _storagePath + camera->getId());
        _slaveCameras.push_back(pSlaveAck);
        _slavesAcq.emplace_back(new std::thread([&, this, pSlaveAck]() {
            std::shared_ptr<AcquisitionThread> acq = pSlaveAck;
            acq->run();
            acq.reset();
        }));

        _failureMonitors.emplace_back(new std::thread([&, this, pSlaveAck]() {
            std::shared_ptr<AcquisitionThread> acq = pSlaveAck;
            if (acq->monitorFailure() && _failureEvent) {
                neosmart::SetEvent(*_failureEvent);
            }
        }));
        return true;
    }
    return false;
}
