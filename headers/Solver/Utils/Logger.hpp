#ifndef LOGGGER_HPP
#define LOGGGER_HPP

#include "Utils.hpp"
#include <iostream>

#ifndef LOG_LEVEL
#define LOG_LEVEL 1
#endif

enum LOG_TYPE
{
    FATAL = 0,
    ERROR = 1,
    WARNING = 2,
    INFO = 3,
    DEBUG = 4
};

namespace Log
{
    static void print(const int &level, const std::string &msg)
    {
        std::string log_levels[5] = {"Fatal", "Error", "Warning", "Info", "Debug"};
        std::cout << Util::CurrentDateTime() << " <" << log_levels[level] << ">:" << msg <<std::endl;
    }

    static void LOG(const LOG_TYPE &level, const std::string &msg)
    {
        int level_int = static_cast<int> (level);
        if (level_int <= LOG_LEVEL)
        {
            print(level_int, msg);
        }
    }

    static void WARN(const std::string &msg)
    {
        LOG(LOG_TYPE::WARNING, msg);
    }

    static void INFO(const std::string &msg)
    {
        Log::LOG(LOG_TYPE::INFO, msg);
    }

    static void DEBUG(const std::string &msg)
    {
        Log::LOG(LOG_TYPE::DEBUG, msg);
    }

    static void ERROR(const std::string &msg)
    {
        Log::LOG(LOG_TYPE::ERROR, msg);
    }

    static void FATAL(const std::string &msg)
    {
        Log::LOG(LOG_TYPE::FATAL, msg);
    }

}

#endif
