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
    std::map<int, D> index_variables;

public:
    Variables1D() {}
    ~Variables1D() {}


    void add_variable(const int &idx_variable, D key, const Variable *var)
    {
        variables.insert(std::make_pair(key, var));
        index_variables.insert(std::pair<int, D>(idx_variable, key));
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

    const Variable *get_variable_by_index(const int &idx) const
    {
        typename std::map<int, D>::const_iterator it_idx_c;
        it_idx_c = index_variables.find(idx);
        if (it_idx_c == index_variables.end())
        {
            throw Exception("Variable of index " std::to_string(idx) " does not exist",
                            ExceptionType::STOP_EXECUTION);
        }
        return get_variable(it_idx_c->second);
    }

    int size() const
    {
        return this->variables.size();
    }
};

#endif
