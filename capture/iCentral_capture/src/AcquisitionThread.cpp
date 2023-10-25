#include "AcquisitionThread.h"

#include "log.h"

#include <fmt/core.h>

#include <boost/filesystem.hpp>

#include <fstream>

namespace {
  uint64_t str2mac(std::string const& s) {
    unsigned char a[6];
    int last = -1;
    int rc = sscanf(s.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx%n",
                    a + 0, a + 1, a + 2, a + 3, a + 4, a + 5,
                    &last);
    if(rc != 6 || s.size() != last)
        //throw std::runtime_error("invalid mac address format " + s);
        return 0;
    return
        uint64_t(a[0]) << 40 |
        uint64_t(a[1]) << 32 | (
            // 32-bit instructions take fewer bytes on x86, so use them as much as possible.
            uint32_t(a[2]) << 24 |
            uint32_t(a[3]) << 16 |
            uint32_t(a[4]) << 8 |
            uint32_t(a[5])
        );
}
}

VIO::capture::AcquisitionThread::AcquisitionThread(
    std::shared_ptr<VIO::capture::Device> camera,
    boost::atomic<bool>* acquire,
    boost::atomic<bool>* pause,
    const std::string& storePath)
    : _camera(camera),
      _acquire(acquire),
      _pause(pause),
      _errorsSeqLength(3),
      _storePath(storePath),
      _visibleStoreName(_storePath.empty() ? "NaN" : _storePath),
      _counter(0),
      _startEvent(neosmart::CreateEvent(true,false)),
      _afterFirstCaptureEvent(neosmart::CreateEvent(true,false)),
      _failureEvent(neosmart::CreateEvent(true,false)),
      _baseTime(0),
      _timeCorrectionUTC(0),
      _camId(str2mac(camera->getId())) {}

VIO::capture::AcquisitionThread::~AcquisitionThread() { _camera.reset(); }

void VIO::capture::AcquisitionThread::run() {
  runGrab();
  if (!_isOk || _acquire->load()) {
    TRACE_N_CONSOLE(ERROR, "Acquisition of camera %s to %s finished before"
       "finish of capture process." " On hold till termination...",
       _camera->getId(), _visibleStoreName);
    neosmart::SetEvent(_failureEvent);
    if (_acquire->load()) {
      _acquire->wait(true);
    }
  } else {
    TRACE_N_CONSOLE(INFO, "Acquisition of camera %s to %s finished. %lld"
       "frames captured." " Terminating camera...", _camera->getId(),
       _visibleStoreName, _counter.load());
  }
  _camera->stopAcquisition();
}

const int VIO::capture::AcquisitionThread::getCounter() const {
    return int(_counter);
}

bool VIO::capture::AcquisitionThread::waitForStart(uint64_t toInMS) {
  return neosmart::WaitForEvent(_startEvent,toInMS) != WAIT_TIMEOUT;
}

bool VIO::capture::AcquisitionThread::waitForFirstFrame(uint64_t toInMS) {
  return neosmart::WaitForEvent(_afterFirstCaptureEvent,toInMS) != WAIT_TIMEOUT;
}

void VIO::capture::AcquisitionThread::updateTimeCorrection(
    uint64_t correctedFirstAcquisitionTime,
    uint64_t firstAcquisitionTimeUTC) {
    /*COMMON_ASSERT(correctedFirstAcquisitionTime >= _firstAcquisitionTime.load(), true,
                  fmt::format("correctedFirstAcquisitionTime = {}, _firstAcquisitionTime = {} camera = {}",
                  correctedFirstAcquisitionTime, _firstAcquisitionTime.load(), _camera->getId()).c_str());*/
    _timeCorrection.store(correctedFirstAcquisitionTime - _firstAcquisitionTime.load());

    if(_firstAcquisitionTime.load() < _baseTime.load()) {
        _timeCorrectionUTC.store(firstAcquisitionTimeUTC - _firstAcquisitionTime.load());
        _firstAcquisitionTimeUTC.store(firstAcquisitionTimeUTC);
        TRACE_N_CONSOLE(INFO, "Updated first frame for %s arrival time with device timestamp %ld to %ld.\n\tNew correction %ld",
                        _visibleStoreName, _firstAcquisitionTime.load(), _firstAcquisitionTimeUTC.load(), _timeCorrectionUTC.load());
    }
    else {
        TRACE_N_CONSOLE(INFO, "Camera on storage %s has synchronised time. No correction will be applied", _visibleStoreName);
        _timeCorrectionUTC.store(_timeCorrection.load());
        _firstAcquisitionTimeUTC.store(correctedFirstAcquisitionTime);
    }
    _firstAcquisitionTime.store(correctedFirstAcquisitionTime);
}

uint64_t VIO::capture::AcquisitionThread::getFirstAcquisitionTime() const {
  return _firstAcquisitionTime.load();
}

uint64_t VIO::capture::AcquisitionThread::getFirstAcquisitionTimeUTC() const {
  return _firstAcquisitionTimeUTC.load();
}

bool VIO::capture::AcquisitionThread::monitorFailure() {
    neosmart::WaitForEvent(_failureEvent);
    return !_isOk || _acquire->load();
}

void VIO::capture::AcquisitionThread::resetState() {
    neosmart::SetEvent(_startEvent);
    neosmart::SetEvent(_afterFirstCaptureEvent);
    neosmart::SetEvent(_failureEvent);
}

void VIO::capture::AcquisitionThread::runGrab() {
   TRACE_N_CONSOLE(INFO, "Ready capturing to %s", _visibleStoreName);
  _isOk = true;
  prepareStorage();
  // CONSOLE("Here we go again %s", _visibleStoreName);
  if (!_acquire->load()) {
     TRACE_N_CONSOLE(INFO, "Wait for starting gun (%s)", _storePath);
    _acquire->wait(false);
  }

  TRACE_N_CONSOLE(INFO, "Steady capturing to %s", _storePath);
  if (!_camera->startAcquisition()) {
    return;
  }
  neosmart::SetEvent(_startEvent);
  TRACE_N_CONSOLE(
      INFO, "Camera \"%s\" started. Waiting first frame.", _camera->getId());
  FrameCallback f =
      [&](const void*, const IMV_FrameInfo* header) -> void {
    _firstAcquisitionTimeUTC.store(
        std::chrono::time_point_cast<std::chrono::nanoseconds>(
            std::chrono::high_resolution_clock::now())
            .time_since_epoch()
            .count());
    _firstAcquisitionTime.store(header->timeStamp);
    TRACE_N_CONSOLE(INFO, "Go capturing to %s", _storePath);
  };

  for (int i = 0; i < INITIALIZATION_ITERATIONS_NO; ++i) {
    _baseTime.store(std::chrono::time_point_cast<std::chrono::nanoseconds>(
                        std::chrono::high_resolution_clock::now())
                        .time_since_epoch()
                        .count());
    _isOk = _camera->nextFrame(f);
  }
  if (!_isOk) {
    TRACE_N_CONSOLE(
        WARNING, "Data stream to %s was not established", _visibleStoreName);
    return;
  }
  updateTimeCorrection(_firstAcquisitionTime.load(),
                       _firstAcquisitionTimeUTC.load());
  neosmart::SetEvent(_afterFirstCaptureEvent);
  int errorsSeqLength = 0;
  while (_acquire->load() && _isOk) {
    _isOk = _camera->nextFrame(_callbackFunctor);
  }
}

VIO::capture::AcquisitionThreadVio::AcquisitionThreadVio(
    size_t camId,
    std::shared_ptr<VIO::capture::DeviceGenIApi> camera,
    boost::atomic<bool>* acquire,
    boost::atomic<bool>* pause,
    const std::string& storePath)
    : AcquisitionThread(camera, acquire, pause, storePath) {}

void VIO::capture::AcquisitionThreadVio::prepareStorage() {}

void VIO::capture::AcquisitionThreadVio::callback(const void* frame, const IMV_FrameInfo *header) {}

VIO::capture::AcquisitionThreadFs::AcquisitionThreadFs(
    size_t camId,
    std::shared_ptr<VIO::capture::Device> camera,
    boost::atomic<bool>* acquire,
    boost::atomic<bool>* pause,
    const std::string& storePath): AcquisitionThread(camera,acquire,pause,storePath) {
    _callbackFunctor = std::bind(&AcquisitionThreadFs::callback, this, std::placeholders::_1, std::placeholders::_2);

}

void VIO::capture::AcquisitionThreadFs::prepareStorage() {
    if(! boost::filesystem::exists(_storePath)) {
        boost::filesystem::create_directory(_storePath);
    }
    if(! boost::filesystem::exists(_storePath)) {
        TRACE(ERROR, "%s", fmt::format("Store path {} is unaccessible", _storePath).c_str());
    }
}

void VIO::capture::AcquisitionThreadFs::callback(const void* frame, const IMV_FrameInfo *header) {
    if(_pause && _pause->load()) {
        return;
    }

    FrameHeader newHeader;
    memset(&newHeader, 0, sizeof(FrameHeader));

    newHeader.cameraId = _camId;

    newHeader.timestamp = header->timeStamp;
    newHeader.timestampRaw = newHeader.timestamp;
    newHeader.timestampUTC = newHeader.timestamp + _timeCorrectionUTC.load();
    newHeader.timestamp += _timeCorrection.load();

    newHeader.payload_size = header->size;
    newHeader.height = header->height;
    newHeader.width = header->width;

    auto file = std::fstream(fmt::format("{}{}image{:010}.raw", _storePath,
                              boost::filesystem::path::separator, _counter.load()),
                             std::ios::out | std::ios::binary);
    file.write((char*)(&newHeader), sizeof(FrameHeader));
    file.write((char*)header, sizeof(IMV_FrameInfo));
    file.write((char*)frame, header->size);
    file.close();
    //std::cout << fmt::format("Frame {} -> {}", getCounter(), _storePath) << std::endl;
    _counter++;
}

std::shared_ptr<VIO::capture::AcquisitionThread>
VIO::capture::AcquisitionThreadProducer::operator()(
    size_t camId,
    std::shared_ptr<VIO::capture::Device> camera,
    boost::atomic<bool>* acquire,
    boost::atomic<bool>* pause,
    const std::string& storePath) {
    std::shared_ptr<AcquisitionThread> thr(
        new AcquisitionThreadFs(camId, camera, acquire, pause, storePath));
    return thr;
}
