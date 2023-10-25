//
// Created by Victor Kats on 21.06.2023
//

#ifndef ODOMETRY_LOG_H
#define ODOMETRY_LOG_H

#include <cstdint>


#include <glog/log_severity.h>


namespace VIO{
namespace aux {

enum class LogActivity: std::uint8_t {
    console = 0b00000001,
    trace = 0b00000010,
    nothing = 0b10000000,
};

void doLog(uint8_t activity, LogSeverity severity, const char* file,int line,const char* fmt,...);

}
}

#define INFO (LogSeverity)0
#define WARNING  (LogSeverity)1
#define ERROR  (LogSeverity)2
#define TRACE(sev,...) VIO::aux::doLog((uint8_t)VIO::aux::LogActivity::trace, sev,__FILE__,__LINE__,__VA_ARGS__)
#define CONSOLE(...) VIO::aux::doLog((uint8_t)VIO::aux::LogActivity::console, INFO,__FILE__,__LINE__,__VA_ARGS__)
#define TRACE_N_CONSOLE(sev,...) \
VIO::aux::doLog((uint8_t)VIO::aux::LogActivity::trace||(uint8_t)VIO::aux::LogActivity::console, sev,__FILE__,__LINE__,__VA_ARGS__)

#endif //ODOMETRY_LOG_H
