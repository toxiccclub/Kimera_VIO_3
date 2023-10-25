//
// Created by Victor Kats on 26.06.2023
//

#ifndef ODOMETRY_DATAACHANNEL_H
#define ODOMETRY_DATAACHANNEL_H

#include "FrameHeader.h"

#include <memory>
#include <vector>

#include <boost/atomic.hpp>

namespace VIO{
namespace capture {

class DataChannel {
public:
    explicit DataChannel(bool isMaster, size_t payloadSize, size_t poolSize);
    ~DataChannel();
    void getWritable(void **data);
    bool freeWritable(bool stepForward);
    size_t getReadable(void **data);
    size_t getReadable(void **data, size_t pos);
    bool freeReadable(bool stepForward);

    bool catchUpReadable();

    bool decReadPosition();
    bool incReadPosition();

private:
    void initPool();

    size_t assignMemory(void **&data, size_t pos, boost::atomic_bool &guard);
    bool freeMemory(size_t &pos, boost::atomic_bool &guard, bool stepForward);
    bool _isMaster;
    size_t _payloadSize;
    size_t _poolSize;
    std::shared_ptr<uint8_t[]> _data;
    std::vector<uint8_t*> _pointer;

    size_t _head;
    size_t _tail;

    boost::atomic_bool _readGuard;
    boost::atomic_bool _writeGuard;

};

}
}

#endif //ODOMETRY_ODOMETRY_DATAACHANNEL_H