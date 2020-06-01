#ifndef MCK_UTIL
#define MCK_UTIL

#include <cmath> /* for std::abs(double) */
#include <vector>
#include <iostream>
#include <limits>

namespace maxkcut
{

const double ZERO = 1e-6;
const double EPSILON = 1e-5;
const double INFINITY_DOUBLE = std::numeric_limits<double>::max();
const double INFINITY_INT = std::numeric_limits<int>::max();

class MKCUtil
{
private:
    /* data */
public:
    MKCUtil(){};
    ~MKCUtil(){};
    static bool isZero(const double &d)
    {
        return std::abs(d) < ZERO ? true : false;
    }

    template <typename T>
    static void print_vector(const std::vector<T> &v)
    {
        std::cout << "Print vector, of size = " << v.size() << " : {";
        for (std::size_t j = 0; j < v.size(); j++)
        {
            std::cout << v[j];
            if (j < v.size() - 1)
                std::cout << ", ";
        }
        std::cout << "};" << std::endl;
    }

    template <typename T>
    static void print_array(const T *v, const int &size)
    {
        std::cout << "Print vector, of size = " << size << " : {";
        for (int j = 0; j < size; j++)
        {
            std::cout << v[j];
            if (j < size - 1)
                std::cout << ", ";
        }
        std::cout << "};" << std::endl;
    }

    template <typename T>
    static void print_matrix(const std::vector<T> &v,const int &dim)
    {
        std::cout << "Print vector, of size = " << v.size() << " :" << std::endl;
            for(std::size_t j = 0; j < v.size(); j++) {
                std::cout << v[j] <<  ", ";
            if (((j+1)%dim ==0)&&(j != 0))
            std::cout << "| " << std::endl;
            }
            std::cout << "| ;"<<std::endl;
    }

};
} // namespace maxkcut

#endif