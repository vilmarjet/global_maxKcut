#ifndef CONSTRAINTS_SDP_HPP
#define CONSTRAINTS_SDP_HPP

#include <vector>
#include "ConstraintSDP.hpp"
#include "../../Utils/Exception.hpp"
#include "ConstraintsGeneric.hpp"

class ConstraintsSDP : public ConstraintsGeneric<ConstraintSDP>
{
private:
    ConstraintsSDP(/* args */) {}

public:
    static ConstraintsSDP *create()
    {
        return new ConstraintsSDP();
    }

    ~ConstraintsSDP()
    {
    }
};

#endif