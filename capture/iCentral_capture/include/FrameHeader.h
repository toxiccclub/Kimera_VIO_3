//
// Created by victor on 31.01.2022.
//

#ifndef ODOMETRY_FRAMEHEADER_H
#define ODOMETRY_FRAMEHEADER_H

#include <cstddef>
#include <cstdint>

namespace VIO{
namespace capture{

#pragma pack(push, 1)

struct FrameHeader {

    //Timestamp the buffer was acquired, in units of 1 ns (1 000 000 000 ticks per second). If the device is internally
    //using another tick frequency than 1GHz, the GenTL Producer must convert the value to nanoseconds. Data type: UINT64
    //For synchronised cameras system this timestamp will be corrected to be the same (or at least similar) for all cameras
    uint64_t timestamp;     // BUFFER_INFO_TIMESTAMP_NS

    //A sequentially incremented number of the frame. This information refers for example to the information provided in
    //the GigE Vision image stream block id. For other technologies this is to be implemented accordingly.
    //The wrap around of this number is transportation technology dependent. For GigE Vision it is (so far) 16bit
    //wrapping to 1. Other technologies may implement a larger bit depth. Data type: UINT64
    uint64_t frameId;       // BUFFER_INFO_FRAMEID

    //Height of the data in the buffer in number of pixels as configured. For variable size images this is the
    //maximum height of the buffer. For example this information refers to the height entry in the GigE Vision image
    //stream data leader. For other technologies this is to be implemented accordingly. Data type: SIZET
    size_t height;          // BUFFER_INFO_HEIGHT

    //Width of the data in the buffer in number of pixels. This information refers for example to the width entry in
    //the GigE Vision image stream data leader. For other technologies this is to be implemented accordingly.
    //Data type: SIZET
    size_t width;           // BUFFER_INFO_WIDTH

    //Size of the data intended to be written to the buffer last time it has been filled. This value is reset to 0
    //when the buffer is placed into the Input Buffer Pool. If the buffer is incomplete the number still reports
    //the full size of the original data including the lost parts. If the buffer is complete, the number equals
    //to the size reported through BUFFER_INFO_SIZE_FILLED.
    //Data type: SIZET
    uint64_t payload_size;    // BUFFER_INFO_DATA_SIZE

    //Pixelformat of the data. This information refers for example to the information provided in the GigE
    //Vision image stream data leader. For other technologies this is to be implemented accordingly. The
    //interpretation of the pixel format depends on the namespace the pixel format belongs to. This can be inquired
    //using the BUFFER_INFO_PIXELFORMAT_NAMESPACE command. Data type: UINT64
    uint64_t pixelFormat;   //BUFFER_INFO_PIXELFORMAT

    //This information refers to the constants defined in PIXELFORMAT_NAMESPACE_IDs to allow interpretation of
    //BUFFER_INFO_PIXELFORMAT. Data type: UINT64
    uint64_t pixelFormatNS; //BUFFER_INFO_PIXELFORMAT_NAMESPACE

    //Flag to indicate that a buffer was filled but an error occurred during that process. Data type: BOOL8
    uint8_t isIncomplete = 0;//BUFFER_INFO_IS_INCOMPLETE

    //Nominal size of buffer
    size_t nominalPayloadSize;

    //Camera identity (IP address)
    uint64_t cameraId;

    //Nanosecond timestamp till epoch the buffer was acquired, exact (for supporting PTP cameras) or estimated
    //For synchronised cameras system this timestamp will be the same (or at least similar) for all cameras
    uint64_t timestampUTC;

    //Value of this->timestamp field before correction on time synchronisation for cameras system
    uint64_t timestampRaw;

    //Reserved fields
    uint64_t reserved_0 = 0;
    uint64_t reserved_1 = 0;
    uint64_t reserved_2 = 0;
    uint64_t reserved_3 = 0;
    uint64_t reserved_4 = 0;
    uint64_t reserved_5 = 0;
    uint64_t reserved_6 = 0;
    uint64_t reserved_7 = 0;
};

#pragma pack(pop)

}
}
#endif //ODOMETRY_FRAMEHEADER_H