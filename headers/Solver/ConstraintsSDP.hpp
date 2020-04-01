#ifndef CONSTRAINTS_SDP_HPP
#define CONSTRAINTS_SDP_HPP

#include <vector>
#include "ConstraintSDP.hpp"

class ConstraintsSDP
{
private:
    std::vector <ConstraintSDP*> constraints;
public:
    ConstraintsSDP(/* args */){}
    ~ConstraintsSDP()
    {
    }

    ConstraintSDP * add_constraint(const double &lb,
                                 const double &ub,
                                 const ConstraintType &typ)
    {
        constraints.push_back(ConstraintSDP::create(lb, ub, typ));

        return constraints[size()-1];
    }
    
    const size_t size()
    {
        return constraints.size();
    }
};

#endif