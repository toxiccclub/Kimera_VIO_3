//
// Created by Victor Kats on 10.09.2023
//

#ifndef ODOMETRY_IMUIMUDATA_H
#define ODOMETRY_IMUIMUDATA_H



#include <stdint.h>



//namespace VIO{
//namespace imu{



#pragma pack(push, 1)


struct ImuData {
    uint8_t startA;
    uint8_t startB;

    uint32_t timestamp;

    int16_t accelerometerRawX;
    int16_t accelerometerRawY;
    int16_t accelerometerRawZ;

    int16_t gyroscopeRawX;
    int16_t gyroscopeRawY;
    int16_t gyroscopeRawZ;

    uint8_t stopA;
    uint8_t stopB;
};

#pragma pack(pop)

//}
//}


#endif //ODOMETRY_ACQUISITIONTHREAD_H
