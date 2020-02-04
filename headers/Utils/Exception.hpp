#ifndef MY_EXCEPTION_HPP
#define MY_EXCEPTION_HPP

#include <iostream>
#include <exception>
#include <string> //string


enum ExceptionType
{
    STOP_EXECUTION,
    DO_NOTHING,
    VERTEX_ZERO_OR_NEGATIVE,
};

class Exception : public std::exception
{
private:
    std::string msg;
    ExceptionType type;

public:
    Exception(const std::string &msg_, const ExceptionType &e = ExceptionType::DO_NOTHING) : 
    msg(msg_), type(e) {}
    std::string get_msg() const
    {
        return this->msg;
    }

    void add_to_msg(const std::string &extrat)
    {
        this->msg += extrat;
    }

    void execute() const
    {
        switch (this->type)
        {
        case ExceptionType::DO_NOTHING:
            std::cerr << this->get_msg() << std::endl;
            break;
        case ExceptionType::VERTEX_ZERO_OR_NEGATIVE:
            std::cerr << this->get_msg() << std::endl;
            std::cerr << "ERROR: Non valid vertices." << std::endl
                      << " Value of vertices must be btw 1 and graph's dimension"
                      << std::endl;
            exit(1);
            break;
        case ExceptionType::STOP_EXECUTION:
            std::cerr << this->get_msg() << std::endl;
            exit(1);
            break;

        default:
            break;
        }
    }

    ~Exception() {}
};


#endif
