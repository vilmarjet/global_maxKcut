#ifndef VARIABLES1dKEY_CONTAINER_HPP
#define VARIABLES1dKEY_CONTAINER_HPP

#include "./Solver/Variable.hpp"
#include "./Utils/Exception.hpp"
#include <vector>
#include <map>
#include <string>
#include <utility>

namespace maxkcut
{
template <class KEY>
class Variables1D
{
private:
    std::map<KEY, const Variable *> variables;
    std::map<int, KEY> index_variables;

public:
    Variables1D() {}
    ~Variables1D() {}


    void add_variable(const int &idx_variable, KEY key, const Variable *var)
    {
        variables.insert(std::make_pair(key, var));
        index_variables.insert(std::pair<int, KEY>(idx_variable, key));
    }

    const Variable *get_variable(KEY key) const
    {
        std::map<KEY, const Variable *>::const_iterator it_const = variables.find(key);

        if (it_const == variables.end())
        {
            // std::string msg = "Variable from" + key->get_code() + "does not exist";
            std::string msg = "Variable from does not exist, for KEY" ;
            throw Exception(msg, ExceptionType::STOP_EXECUTION);
        }

        return it_const->second;
    }

    const Variable *get_variable_by_index(const int &idx) const
    {
        std::map<int, const KEY *>::const_iterator it_idx_c;
        it_idx_c = index_variables.find(idx);
        if (it_idx_c == index_variables.end())
        {
            throw Exception("Variables: Index of variable does not exist",
                            ExceptionType::STOP_EXECUTION);
        }
        return get_variable(it_idx_c->second);
    }

    int size() const
    {
        return this->variables.size();
    }
};
} // namespace maxkcut

#endif
