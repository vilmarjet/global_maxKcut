#ifndef TABU_HEURISTIC_HPP
#define TABU_HEURISTIC_HPP

#include <vector>
#include <string>
#include <algorithm>

template <class value_type>
class Tabu
{
private:
    int size_tabu_list;
    std::vector<value_type> tabu_list;
    int pos_in_tabu;

public:
    Tabu(int size = 7) : size_tabu_list(size), pos_in_tabu(0)
    {
        tabu_list.resize(size);
    }
    ~Tabu() {}

    void add_value(const value_type &elem)
    {
        tabu_list[pos_in_tabu++] = elem;

        if (pos_in_tabu == size_tabu_list)
        {
            pos_in_tabu = 0;
        }
    }

    bool has_element(const value_type &elem)
    {
        return find(tabu_list.begin(),
                    tabu_list.end(),
                    elem) != tabu_list.end()
                   ? true
                   : false;
    }

    void clean(const value_type &val)
    {
        pos_in_tabu = 0;
        std::fill(tabu_list.begin(), tabu_list.end(), val);
    }

    void print()
    {
        std::cout << "tabu_list=[";
        for (int i = 0; i < size_tabu_list; ++i)
            std::cout << tabu_list[i]<< ",";
        std::cout << "]";
    }
};

#endif