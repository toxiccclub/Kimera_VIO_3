#include "DataChannel.h"

VIO::capture::DataChannel::DataChannel(bool isMaster,
                                       size_t payloadSize,
                                       size_t poolSize):
                                       _isMaster(isMaster),
                                       _payloadSize(payloadSize),
                                       _poolSize(poolSize),
                                       _data(new uint8_t[(payloadSize+sizeof(VIO::capture::FrameHeader))*poolSize]),
                                       _pointer(poolSize, nullptr),
                                       _head(0),
                                       _tail(0),
                                       _readGuard(false),
                                       _writeGuard(false) {}

VIO::capture::DataChannel::~DataChannel() {
    _pointer.clear();
    _data.reset();
}

void VIO::capture::DataChannel::getWritable(void** data) {
    assignMemory(data, _head, _writeGuard);
}

bool VIO::capture::DataChannel::freeWritable(bool stepForward) {
    if(freeMemory(_head, _writeGuard, stepForward)) {
        if(_head == _tail && stepForward) {
            incReadPosition();
        }
    }
    return false;
}

size_t VIO::capture::DataChannel::getReadable(void** data) {
    if(_head==_tail) {
        if(_writeGuard.load()) {
            _writeGuard.wait(true);
        }
    }
    if(_head==_tail) {
        *data = nullptr;
        return 0;
    }
    return assignMemory(data, _tail, _readGuard);
}

size_t VIO::capture::DataChannel::getReadable(void** data, size_t pos) {
    pos %= _poolSize;
    return assignMemory(data, pos, _readGuard);
}

bool VIO::capture::DataChannel::freeReadable(bool stepForward) {
    return freeMemory(_head, _readGuard, stepForward);
}

bool VIO::capture::DataChannel::catchUpReadable() {
    if(_readGuard.load()) {
        _readGuard.wait(true);
    }
    _readGuard.store(true);
    _readGuard.notify_all();
    bool isItDone = true;
    if(_head == _tail) {
        if(_writeGuard.load()) {
            _writeGuard.wait(true);
        }
    }
    if(_head == _tail) {
        isItDone = false;
    }
    else {
        _tail = (_head + _payloadSize - 1) % _poolSize;
    }
    _readGuard.store(false);
    _readGuard.notify_all();
    return isItDone;
}

bool VIO::capture::DataChannel::decReadPosition() {
    if(_readGuard.load()) {
        _readGuard.wait(true);
    }
    if(_tail - 1 == _head) {
        return false;
    }
    _readGuard.store(true);
    _readGuard.notify_all();
    _tail = (_tail + _poolSize - 1) % _poolSize;
    _readGuard.store(false);
    _readGuard.notify_all();
    return true;
}

bool VIO::capture::DataChannel::incReadPosition() {
    if(_readGuard.load()) {
        _readGuard.wait(true);
    }
    if(_tail + 1 == _head) {
        if(_writeGuard.load()) {
            _writeGuard.wait(true);
        }
    }
    if(_tail + 1 == _head) {
        return false;
    }
    _readGuard.store(true);
    _readGuard.notify_all();
    _tail = (_tail + 1) % _poolSize;
    _readGuard.store(false);
    _readGuard.notify_all();
    return true;}

void VIO::capture::DataChannel::initPool() {
    auto data = _data.get();
    for(auto i = 0; i < _poolSize; ++i) {
        _pointer[i] = data + (_payloadSize + sizeof(VIO::capture::FrameHeader)) * _poolSize * i;
    }
}

size_t VIO::capture::DataChannel::assignMemory(void** &data,
                                               size_t pos,
                                               boost::atomic_bool& guard) {
    if(!data) {
        throw(std::runtime_error("NULL pointer to assign readable memory to"));
    }
    if(guard.load()) {
        guard.wait(true);
    }
    guard.store(true);
    guard.notify_all();
    *data = _pointer[pos];
    return pos;
}

bool VIO::capture::DataChannel::freeMemory(size_t &pos,
                                           boost::atomic_bool& guard, bool stepForward) {
    if(!guard.load()) {
        return false;
    }
    if(stepForward) {
        ++pos;
        pos %= _poolSize;
    }
    pos += stepForward;
    guard.store(false);
    guard.notify_all();
    return true;
}
