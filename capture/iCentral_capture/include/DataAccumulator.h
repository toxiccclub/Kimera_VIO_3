//
// Created by Victor Kats on 24.06.2023
//

#ifndef ODOMETRY_DATAACCUMULATOR_H
#define ODOMETRY_DATAACCUMULATOR_H


namespace VIO{
namespace capture {

class DataChannel {
public:
    explicit DataChannel(bool isMaster);
    ~DataChannel();
};

class DataAccumulator {
public:
    DataAccumulator();
    ~DataAccumulator();

private:
};

}
}

#endif //ODOMETRY_DATAACCUMULATOR_H
