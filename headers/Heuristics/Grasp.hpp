#ifndef GRASP_HEURISTIC_HPP
#define GRASP_HEURISTIC_HPP

#include <set>
#include <algorithm>
#include <iostream>

template <class candidate_type>
class Candidate_for_grasp_order_rcl
{
private:
    candidate_type element;
    double value;

public:
    Candidate_for_grasp_order_rcl(const candidate_type &elem, const double &val) : element(elem), value(val) {}
    candidate_type get_candidate() const { return element; }
    double get_value() const { return value; }
    bool operator<(const Candidate_for_grasp_order_rcl &c) const
    {
        return (this->value <= c.value);
    }
};

template <class element_type>
class Grasp
{

private:
    double PROP_RCL; //For GRASP, propability of random selection in rcl
    std::set<Candidate_for_grasp_order_rcl<element_type>> rcl;
    typename std::set<Candidate_for_grasp_order_rcl<element_type>>::iterator it;

public:
    Grasp(double prop = 0.8) : PROP_RCL(prop)
    {
        std::srand(std::time(nullptr));
    }
    ~Grasp() {}

    void clear_rcl()
    {
        rcl.clear();
    }

    void add_candidate_to_rcl(const element_type &elem, const double &val)
    {
        rcl.insert(Candidate_for_grasp_order_rcl<element_type>(elem, val));
    }

    //@param: Default_candidate is returned if rcl is empty
    //@return element from rcl set. selected with greedy random.
    element_type select_candidate_from_rcl(const element_type &Default_candidate)
    {
        int random;
        double proR = this->PROP_RCL;

        if (rcl.size() == 0)
        {
            return Default_candidate; //flag value
        }

        if (rcl.size() == 1)
        {
            return (*rcl.begin()).get_candidate();
        }

        it = rcl.begin();
        while (true)
        {
            random = ((double)(std::rand() % 101) / 100);
            if (random <= proR)
            {
                return (*it).get_candidate();
            }

            if (++it == rcl.end())
            {
                proR *= 1.2; //increase of 20% the proportion
                it = rcl.begin();
            }
        }
    }
};

#endif