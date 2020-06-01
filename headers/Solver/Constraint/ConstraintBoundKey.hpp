#ifndef CONSTRAINT_BOUND_KEY_HPP
#define CONSTRAINT_BOUND_KEY_HPP

#include <string>

enum ConstraintBoundKey
{
    EQUAL,          // (=)
    INFERIOR_EQUAL, // (<=)
    SUPERIOR_EQUAL, //(>=)
};

class ConstraintBoundKeyString
{
public:
    static std::string to_string(const ConstraintBoundKey &bound)
    {
        switch (bound)
        {
        case EQUAL:
            return " = ";

        case INFERIOR_EQUAL:
            return " <= ";

        case SUPERIOR_EQUAL:
            return " >= ";

        default:
            break;
        }
    }
};

#endif