#ifndef UTILITIESSZ_TIME_HPP
#define UTILITIESSZ_TIME_HPP
 
#include <iostream>
#include <string>
#include <time.h>
 
namespace utils
{
    // Get current date/time, format is YYYY-MM-DD.HH:mm:ss
    static std::string CurrentDateTime()
    {
        time_t     now = time(NULL);
        struct tm  tstruct;
        char       buf[80];
        tstruct = *localtime(&now);
        strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);
        return buf;
    }
}
#endif