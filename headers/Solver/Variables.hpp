#ifndef VARIABLES1d_CONTAINER_HPP
#define VARIABLES1d_CONTAINER_HPP

#include "Variable.hpp"
#include "DimensionVariable.hpp"
#include "../Utils/Exception.hpp"
#include <vector>
#include <map>
#include <string>

template <class Dimension>
class Variables
{
private:
    std::map<const Dimension *, Variable *> variables;
    typename std::map<const Dimension *, Variable *>::iterator it;

    std::map<int, const Dimension *> index_variables;
    typename std::map<int, const Dimension *>::iterator it_idx;

    int idx_variable;

public:
    Variables() : idx_variable(0) {}
    ~Variables() {}

    void add_variable(const Dimension *key, const Variable &var)
    {
        variables.insert(std::pair<const Dimension *,
                                   Variable *>(key, new Variable(idx_variable, var)));

        index_variables.insert(std::pair<int, const Dimension *>(idx_variable, key));

        idx_variable++;
    }

    Variable *get_variable(const Dimension *key)
    {
        it = variables.find(key);

        if (it == variables.end())
        {
            throw Exception("Variables1D: Variable does not exist",
                            ExceptionType::STOP_EXECUTION);
        }

        return it->second;
    }

    const Variable *get_variable(const Dimension *key) const
    {

        std::map<const Dimension *, Variable *>::const_iterator it_const = variables.find(key);

        if (it_const == variables.end())
        {
            // std::string msg = "Variable from" + key->get_code() + "does not exist";
            std::string msg = "Variable from does not exist";
            throw Exception(msg, ExceptionType::STOP_EXECUTION);
        }

        return it_const->second;
    }

    Variable *get_variable_by_index(const int &idx)
    {
        it_idx = index_variables.find(idx);
        if (it_idx == index_variables.end())
        {
            throw Exception("Variables: Index of variable does not exist",
                            ExceptionType::STOP_EXECUTION);
        }
        return get_variable(it_idx->second);
    }

    const Variable *get_variable_by_index(const int &idx) const
    {
        std::map<int, const Dimension *>::const_iterator it_idx_c;
        it_idx_c = index_variables.find(idx);
        if (it_idx_c == index_variables.end())
        {
            throw Exception("Variables: Index of variable does not exist",
                            ExceptionType::STOP_EXECUTION);
        }
        return get_variable(it_idx_c->second);
    }

    std::string to_string() const
    {
        std::string s;
        s = "Variables: \n";

        for (int i = 0; i < size(); ++i)
        {
            s += get_variable_by_index(i)->to_string() + "\n";
        }
        return s;
    }

    int size() const
    {
        return this->variables.size();
    }
};

#endif
