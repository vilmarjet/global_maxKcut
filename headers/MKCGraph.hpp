#ifndef MKC_GRAPH
#define MKC_GRAPH

#include <string> //string
#include <vector>
#include <cstddef>
#include "Edges.hpp"

namespace maxkcut
{

enum GraphType
{
    COMPLETE = 0, // with all complete graphs
    CHORDAL = 12,  // no cycle bigger than 4
    SIMPLE = 13,   // as it is (can be sparse)
};

class MKCGraph
{
private:
    int dimension; /*Dimention = number of vertices*/
    Edges edges;
    std::vector<std::vector<int>> vertices_maximal_clique;
    GraphType type;

public:
    static int get_number_edges_complete_grpah(const int &dim)
    {
        return (int)(((dim - 1) * dim) / 2.0);
    }

    MKCGraph(const Edges &ed, GraphType t = GraphType::SIMPLE)
        : dimension(ed.get_number_vertices()),
          edges(ed),
          type(t)
    {
        if (edges.get_number_edges() == edges.get_number_edges_if_complete_grpah())
        {
            this->type = GraphType::COMPLETE;
        }
    }

    ~MKCGraph() {}

    GraphType get_type() const
    {
        return type;
    }

    int get_dimension() const
    {
        return this->dimension;
    }

    int get_number_edges() const
    {
        return this->edges.get_number_edges();
    }

    const Edges *get_edges() const
    {
        return &this->edges;
    }

    bool is_sparse() const
    {
        return get_number_edges() < 0.4 * get_number_edges_complete_grpah();
    }

    bool is_complete()
    {
        return get_number_edges_complete_grpah() == get_number_edges();
    }

    inline int get_number_edges_complete_grpah() const
    {
        return get_number_edges_complete_grpah(this->dimension);
    }

    std::string to_string() const
    {
        std::string s;
        s = "\n *** Input graph *** \n";
        s += "Dim: " + std::to_string(get_dimension()) + "\n";
        s += "Number of edges: " + std::to_string(get_number_edges()) + "\n";
        s += edges.to_string();

        return s;
    }
};
} // namespace maxkcut

#endif