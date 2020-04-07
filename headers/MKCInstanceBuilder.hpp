#ifndef MKC_INSTANCE_BUILDER_HPP
#define MKC_INSTANCE_BUILDER_HPP

#include <cstddef>
#include "MKCGraphBuilder.hpp"
#include "MKCInstance.hpp"
#include <iostream>

namespace maxkcut
{
template <class C>
class MKCInstanceBuilder
{
private:
    C *parent;
    MKCGraphBuilder<MKCInstanceBuilder<C>> *graph_builder;
    int K = 3;

    MKCInstanceBuilder(C *parent_) : parent(parent_),
                                     graph_builder(nullptr) {}

public:
    static MKCInstanceBuilder<C> *create(C *parent)
    {
        return new MKCInstanceBuilder<C>(parent);
    }

    static MKCInstanceBuilder *create()
    {
        std::nullptr_t np = nullptr;
        return MKCInstanceBuilder<C>::create(np);
    }

    MKCGraphBuilder<MKCInstanceBuilder> *set_graph()
    {
        // MKCGraphBuilder<MKCInstanceBuilder<C>> graph_b = 
        this->graph_builder = MKCGraphBuilder<MKCInstanceBuilder<C>>::create(this);

        return graph_builder;
    }

    MKCInstanceBuilder<C> *set_K(const int &k)
    {
        this->K = k;

        return this;
    }

    C end()
    {
        return *this->parent;
    }

    MKCInstance* build()
    {
        MKCGraph graph = this->graph_builder->build();

        return new MKCInstance(graph, this->K);
    }

    ~MKCInstanceBuilder(){};
};
} // namespace maxkcut

#endif
