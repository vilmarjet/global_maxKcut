#ifndef CONSTRAINTS_SDP_HPP
#define CONSTRAINTS_SDP_HPP

#include <vector>
#include "ConstraintSDP.hpp"
#include "../../Utils/Exception.hpp"

class ConstraintsSDP
{
private:
    std::vector <ConstraintSDP*> constraints;
    void validate(const int &i) const
    {
        if (i <0 || i > size())
        {
            Exception("Invalid index in ConstraintsSDP", ExceptionType::STOP_EXECUTION).execute();
        }
    }

public:
    ConstraintsSDP(/* args */){}
    ~ConstraintsSDP()
    {
        
    }

    ConstraintSDP * add_constraint(const double &lb,const double &ub,const ConstraintType &typ)
    {
        constraints.push_back(ConstraintSDP::create(lb, ub, typ));

        return constraints[size()-1];
    }

    const ConstraintSDP * get_constraint(const int &i)
    {
        validate(i);
        return constraints[i];
    }
    
    const size_t size() const
    {
        return constraints.size();
    }
};

#endif