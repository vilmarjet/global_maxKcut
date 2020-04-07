#ifndef MKC_INSTANCE_HPP
#define MKC_INSTANCE_HPP

#include "MKCGraph.hpp"

namespace maxkcut
{
class MKCInstance
{
private:
    MKCGraph graph;
    int K_partitions;

public:
    MKCInstance(const MKCGraph &graph_,
                const int &K_) : graph(graph_),
                                 K_partitions(K_) {}

    ~MKCInstance() {}

    const MKCGraph *get_graph() const
    {
        return &this->graph;
    }

    int get_K() const
    {
        return this->K_partitions;
    }

};
} // namespace maxkcut

#endif