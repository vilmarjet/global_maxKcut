#ifndef LINEAR_CONSTRAINTS_HPP
#define LINEAR_CONSTRAINTS_HPP

#include "LinearConstraint.hpp"
#include <vector>
#include "../../Utils/Exception.hpp"
#include "ConstraintsGeneric.hpp"


class LinearConstraints : public ConstraintsGeneric<LinearConstraint>
{
private:
    LinearConstraints(/* args */){}

public:
    static LinearConstraints* create ()
    {
        return new LinearConstraints();
    }
    
    ~LinearConstraints(){}
};


#endif