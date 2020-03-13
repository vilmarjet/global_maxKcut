#ifndef VARIABLES1d_CONTAINER_HPP
#define VARIABLES1d_CONTAINER_HPP

#include "Variable.hpp"
#include "DimensionVariable.hpp"
#include "../Utils/Exception.hpp"
#include <utility> // std::pair, std::make_pair
#include <vector>
#include <map>
#include <string>
#include <typeinfo>

class Variables
{
private:
    std::vector<Variable* > variables;
    std::map <const Variable *, int> index_variables;
    

    void validate_index(const int &idx) const
    {
        if (idx >= variables.size() || idx < 0)
        {
            throw Exception("Variables: Index of variable does not exist",
                            ExceptionType::STOP_EXECUTION);
        }
    }

public:
    Variables(){};
    ~Variables(){};

    void add_variable(const Variable &var)
    {
        //FIXME to be optimized 
        variables.push_back(new Variable(var));
        int idx = variables.size() -1;
        index_variables.insert(std::pair<const Variable *, int>(get_variable(idx), idx));    
    }

    const Variable *const get_variable(const int &idx) const
    {
        validate_index(idx);
        return variables[idx];
    }

    const int get_index(const Variable *var) const
    {
         std::map <const Variable *, int>::const_iterator it_idx = index_variables.find(var);

        if (it_idx == index_variables.end())
        {
            throw Exception("Variables1D: Variable does not exist = " + var->to_string() ,
                            ExceptionType::STOP_EXECUTION);
        }

        return it_idx->second;
    }

    void set_solution_value(const int &idx, const double &val)
    {
        validate_index(idx);
        variables[idx]->update_solution(val);
    }

    std::string to_string() const
    {
        std::string s;
        // s = typeid(this).name() + ": \n";

        for (int i = 0; i < size(); ++i)
        {
            s += get_variable(i)->to_string() +  "\n" ;
        }

        return s;
    }

    int size() const
    {
        return this->variables.size();
    }
};

#endif
