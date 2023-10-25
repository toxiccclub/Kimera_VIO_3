//
// Created by Victor Kats on 14.06.2023
//

#ifndef ODOMETRY_ACQUISITIONTHREAD_H
#define ODOMETRY_ACQUISITIONTHREAD_H

//#include <GenTLWrapper/LayerImplementation.h>
//#include <GenApiUtils/ChannelManager.h>

//#include <storage/IRingBuffer.h>
//#include <pevents/pevent_wrapper.h>
///#include <libinfrastructure/channel_manager/OutputRBHolder.h>

#include "FrameHeader.h"
#include "Device.h"

#include "pevents.h"

#include <boost/atomic.hpp>

#include <chrono>
#include <cstdint>
#include <functional>
#include <string>

//#define INITIALIZATION_ITERATIONS_NO 1
//using RingbufferType = decltype(std::declval<tmk::infrastructure::channel_manager::RBHolder>()[0]);

#define INT64_INFTY ~((uint64_t)0)

namespace VIO{
namespace capture{

class AcquisitionThread {
public:
    explicit AcquisitionThread(std::shared_ptr<VIO::capture::Device> camera, boost::atomic<bool> *acquire,
                               boost::atomic<bool> *pause, const std::string &storePath);
    AcquisitionThread() = delete;
    ~AcquisitionThread();
    virtual void prepareStorage() = 0;

    void run();

    [[nodiscard]] const int getCounter() const;
    bool waitForStart(uint64_t toInMS=INT64_INFTY);
    bool waitForFirstFrame(uint64_t toInMS=INT64_INFTY);

    void updateTimeCorrection(uint64_t correctedFirstAcquisitionTime, uint64_t firstAcquisitionTimeUTC);
    uint64_t getFirstAcquisitionTime() const;
    uint64_t getFirstAcquisitionTimeUTC() const;

    bool monitorFailure();

    void resetState();
#ifdef INITIALIZATION_ITERATIONS_NO_DEFINED
    constexpr static const int INITIALIZATION_ITERATIONS_NO=INITIALIZATION_ITERATIONS_NO_DEFINED;
#else
    constexpr static const int INITIALIZATION_ITERATIONS_NO=3;
#endif
protected:
    virtual void callback(const void *frame, const IMV_FrameInfo *header) = 0;

    std::shared_ptr<VIO::capture::Device> _camera;
    boost::atomic<bool> *_acquire;
    boost::atomic<bool> *_pause;

    int _errorsSeqLength;

    std::string _storePath;
    std::string _visibleStoreName;

    boost::atomic<long long unsigned int> _counter;

    FrameCallback _callbackFunctor;
    neosmart::neosmart_event_t _startEvent;
    neosmart::neosmart_event_t _afterFirstCaptureEvent;

    neosmart::neosmart_event_t _failureEvent;

    boost::atomic<uint64_t> _firstAcquisitionTime;
    boost::atomic<uint64_t> _firstAcquisitionTimeUTC;
    boost::atomic<uint64_t> _baseTime;
    boost::atomic<uint64_t> _timeCorrection;
    boost::atomic<uint64_t> _timeCorrectionUTC;

    bool _isOk;

    uint64_t _camId;
private:
    void runGrab();
};

class AcquisitionThreadFs: public AcquisitionThread  {
public:
    explicit AcquisitionThreadFs(size_t camId, std::shared_ptr<VIO::capture::Device> camera,
                               boost::atomic<bool> *acquire,
                               boost::atomic<bool> *pause, const std::string &storePath);
    void prepareStorage() override;

protected:
    void callback(const void *frame, const IMV_FrameInfo *header) override;
};


class AcquisitionThreadVio: public AcquisitionThread  {
public:
    explicit AcquisitionThreadVio(size_t camId, std::shared_ptr<VIO::capture::DeviceGenIApi> camera,
                               boost::atomic<bool> *acquire,
                               boost::atomic<bool> *pause, const std::string &storePath);
    void prepareStorage() override;

protected:
    void callback(const void *frame, const IMV_FrameInfo *header) override;

private:
    //std::shared_ptr<ChannelManager> _memoryManager;
    //RingbufferType _storeBuffer;
    //std::shared_ptr<tmk::storage::ring_buffer::IRingBuffer<char>> _storeBuffer;
    size_t _camId;
};

struct AcquisitionThreadProducer
{
    std::shared_ptr<AcquisitionThread> operator()(size_t camId, std::shared_ptr<VIO::capture::Device> camera,
                               boost::atomic<bool> *acquire,
                               boost::atomic<bool> *pause, const std::string &storePath);
};


}
}

#endif //ODOMETRY_ACQUISITIONTHREAD_H
