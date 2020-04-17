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
    const double lowerBound;
    const double upperBound;
    const double cost;
    const VariableType type = VariableType::CONTINOUS;
    const std::string code;

    double solution = 0.0;

    Variable(const double &lb,
             const double &ub,
             const double &sol = 0.0,
             const double &cst = 0.0,
             const VariableType typ = VariableType::CONTINOUS,
             const std::string &name = "x") : lowerBound(lb),
                                              upperBound(ub),
                                              cost(cst),
                                              type(typ),
                                              code(name),
                                              solution(sol) {}

public:
    static Variable *create()
    {
        return create(0.0, 0.0, 0.0, 0.0, VariableType::CONTINOUS, "X_null_");
    }

    static Variable *create(const double &lb,
                            const double &ub,
                            const double &sol = 0.0,
                            const double &cst = 0.0,
                            const VariableType typ = VariableType::CONTINOUS,
                            const std::string &name = "x")
    {
        return new Variable(lb, ub, sol, cst, typ, name);
    }

    static Variable *of(const Variable &variable)
    {
        return create(variable.lowerBound,
                      variable.upperBound,
                      variable.solution,
                      variable.cost,
                      variable.type,
                      variable.code);
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
        return this->code;
    }

    ~Variable() {}
};

#endif
