#ifndef VARIABLES_CONTAINER_HPP
#define VARIABLES_CONTAINER_HPP

#include "Variable.hpp"
#include "../Utils/Exception.hpp"
#include <vector>

class VariablesOld
{
private:
    std::vector<Variable> variables;
    int number_variables  = 0;

public:
    VariablesOld(const int &size) { variables.resize(size); }
    VariablesOld() {variables.resize(1);}
    ~VariablesOld() {}

    void add_variable(const Variable &var)
    {
        int idx = var.get_index();
        if (variables.size() <= idx) 
        {
            variables.resize(idx*2);
        }

        number_variables++;
        variables[idx] = var;
    }

    Variable *get_variable(const int &idx)
    {
        validate_index_variable(idx);

        return &this->variables[idx];
    }


    double get_solution_variable(const int &idx) const
    {
        validate_index_variable(idx);

        return this->variables[idx].get_solution();
    }

    bool validate_index_variable(const int &idx) const
    {
        if (idx < 0 || idx > this->size())
        {
            throw Exception("In Variables: Out of bound", ExceptionType::STOP_EXECUTION);
        }
    }

    void update_solution_variable(const double &val, const int idx_variable)
    {
        variables[idx_variable].update_solution(val);
    }

    int size() const
    {
        return this->number_variables;
    }
};

#endif
