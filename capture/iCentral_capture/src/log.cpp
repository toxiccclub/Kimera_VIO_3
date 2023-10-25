#include "log.h"

#include <glog/logging.h>

#include <iostream>
#include <cstdarg>
#include <cstddef>
#include <string>
#include <syncstream>

void VIO::aux::doLog(uint8_t activity, LogSeverity severity, const char* file, int line, const char* fmt,...)
{
    va_list args;
    va_start(args,fmt);
    size_t req_len= vsnprintf(nullptr,0,fmt,args);
    va_end(args);
    if(req_len > 0)
    {
        std::string msg(req_len+1,'\0');
        va_start(args,fmt);
        vsnprintf( msg.data(),req_len+1,fmt,args);
        va_end(args);
        if(activity&(uint8_t)LogActivity::trace) {
            google::LogMessage( file, line, severity).stream() << msg.c_str();
        }
        if(activity&(uint8_t)LogActivity::console) {
            std::osyncstream(std::cout) << msg << '\n';
        }
    }
    va_end(args);
}
