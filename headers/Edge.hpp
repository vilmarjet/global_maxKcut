#ifndef MKC_GraphEDGE
#define MKC_GraphEDGE

#include "./Utils/Exception.hpp"
#include <string>
#include <algorithm>

namespace maxkcut
{
class Edge
{
private:
    int i, j;
    double weight;
    std::string code;

public:
    Edge() : i(-1), j(-1), weight(0.0)
    {
        this->code = "Edge(not defined)";
    }
    Edge(const int &vi, const int &vj, const double &w) : weight(w), i(vi), j(vj)
    {
        Edge::validate_vertices_i_j(this->i, this->j);
        this->code = "e(" + std::to_string(vi) + "," + std::to_string(vj) + ")";
    }

    /**
     * Return idx of a vertex. 
     * Remember vertex starts by 1
     * */
    inline int get_vertex_i() const
    {
        return this->i;
    }

    /**
     * Return idx of a vertex. 
     * Remember vertex starts by 1
     * */
    inline int get_vertex_j() const
    {
        return this->j;
    }
    inline int get_weight() const
    {
        return this->weight;
    }

    std::string get_code() const
    {
        return code;
    }

    void add_weight(const double &w)
    {
        weight += w;
    }

    static void validate_vertices_i_j(int &vi, int &vj)
    {
        if (vi <= 0 || vj <= 0)
        {
            std::string msg = "Got: (" + std::to_string(vi) +
                              msg += "," + std::to_string(vj) + "). \n";
            throw Exception(msg, ExceptionType::VERTEX_ZERO_OR_NEGATIVE);
        }

        if (vi > vj)
        {
            std::swap(vi, vj);
        }
    }

    inline Edge operator=(Edge e)
    {
        this->i = e.i;
        this->j = e.j;
        this->weight = e.weight;
        this->code = e.code;

        return e;
    }

    inline std::string to_string() const
    {
        std::string s;
        s = "(" + std::to_string(this->i);
        s += "," + std::to_string(this->j);
        s += ") = " + std::to_string(this->weight);

        return s;
    }

    ~Edge() {}
};
} // namespace maxkcut

#endif