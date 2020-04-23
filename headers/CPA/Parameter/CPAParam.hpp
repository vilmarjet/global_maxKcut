#ifndef CPA_PARAMETERS_HPP
#define CPA_PARAMETERS_HPP

class CPAParam
{  
private:
    int number_max_iterations = 1;

public:
    CPAParam(/* args */) {}
    ~CPAParam() {}

    CPAParam *set_number_max_iterations(const int &number)
    {
        assert(number > 0);
        number_max_iterations = number;
        return this;
    }

    const int &get_number_max_iterations() const
    {
        return number_max_iterations;
    }
};

#endif