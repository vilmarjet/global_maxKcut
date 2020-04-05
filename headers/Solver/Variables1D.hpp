#ifndef VARIABLES1dKEY_CONTAINER_HPP
#define VARIABLES1dKEY_CONTAINER_HPP

#include "Variable.hpp"
#include "../Utils/Exception.hpp"
#include <vector>
#include <map>
#include <string>
#include <utility>

template <typename D>
class Variables1D
{
private:
    std::map<D, const Variable *> variables;

public:
    Variables1D() {}
    ~Variables1D() {}


    void add_variable(D key, const Variable *var)
    {
        variables.insert(std::make_pair(key, var));
    }

    const Variable *get_variable(D key) const
    {
        typename std::map<D, const Variable *>::const_iterator it_const = variables.find(key);

        if (it_const == variables.end())
        {
            std::string msg = "Variable from does not exist, for KEY" ;
            throw Exception(msg, ExceptionType::STOP_EXECUTION);
        }

        return it_const->second;
    }

    int size() const
    {
        return this->variables.size();
    }
};

#endif
