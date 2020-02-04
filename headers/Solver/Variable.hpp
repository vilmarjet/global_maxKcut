#ifndef VARIABLE_SOLVER_HPP
#define VARIABLE_SOLVER_HPP

#include <string>

enum VariableType
{
    CONTINOUS, //
    INTEGER,   //
    SDP,       //
};

class Variable
{
private:
    int index;
    double lowerBound;
    double upperBound;
    double cost;
    VariableType type = VariableType::CONTINOUS;
    double solution = 0.0;
    std::string code;

public:
    Variable() : index(-1), lowerBound(0.0), upperBound(0.0), cost(0.0){};
    Variable(const int &idx,
             const double &lb,
             const double &ub,
             const double &cst = 0.0,
             const VariableType typ = VariableType::CONTINOUS,
             const std::string &name = "x") : index(idx),
                                              lowerBound(lb),
                                              upperBound(ub),
                                              cost(cst),
                                              type(typ),
                                              code(name)
    {
        code += "(" + std::to_string(this->index) + ")";
    }

    Variable(const int &idx, const Variable &var) : index(idx),
                                                    lowerBound(var.get_lower_bound()),
                                                    upperBound(var.get_upper_bound()),
                                                    cost(var.get_cost()),
                                                    type(var.get_type()),
                                                    code(var.get_code()) {}

    static Variable *of(Variable variable, const int &idx)
    {
        return new Variable(idx,
                            variable.lowerBound,
                            variable.upperBound,
                            variable.cost, variable.type);
    }

    int get_index() const
    {
        return this->index;
    }

    void set_index(const int &idx)
    {
        this->index = idx;
    }

    void set_bounds(const double &lb, const double &ub)
    {
        this->lowerBound = lb;
        this->upperBound = ub;
    }

    void set_cost_variables(const double &cost)
    {
        this->cost = cost;
    }

    double get_lower_bound() const
    {
        return this->lowerBound;
    }

    double get_upper_bound() const
    {
        return this->upperBound;
    }

    double get_cost() const
    {
        return this->cost;
    }

    VariableType get_type() const
    {
        return this->type;
    }

    void update_solution(const double &val)
    {
        this->solution = val;
    }

    double get_solution() const
    {
        return this->solution;
    }

    std::string get_code() const
    {
        return this->code;
    }

    std::string to_string() const
    {
        std::string s;

        s = std::to_string(this->lowerBound) +
            "<= " + this->code + " <= " + std::to_string(this->upperBound);
        s += "; Cost=" + std::to_string(this->cost);
        s += "; Solution=" + std::to_string(this->solution);

        return s;
    }

    ~Variable() {}
};

#endif
