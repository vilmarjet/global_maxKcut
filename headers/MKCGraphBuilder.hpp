#ifndef MKC_GRAPH_BUILDER
#define MKC_GRAPH_BUILDER

#include <string> //string
#include <vector>
#include <cstddef>
#include <sstream>
#include <fstream>

#include "Utils/Exception.hpp"
#include "MKCUtil.hpp"

#include "Edges.hpp"
#include "MKCGraph.hpp"

namespace maxkcut
{
template <class C>
class MKCGraphBuilder
{
private:
    std::string input_file = "ERROR";
    C *parent;
    GraphType typeGraph = GraphType::SIMPLE;
    MKCGraphBuilder(C *parent) : parent(parent) 
    {
    }

public:
    static MKCGraphBuilder<C> *create(C *parent)
    {
        return new MKCGraphBuilder<C>(parent);
    }

    static MKCGraphBuilder<C> *create()
    {
        std::nullptr_t np = nullptr;
        return MKCGraphBuilder<std::nullptr_t>::create(np);
    }

    MKCGraphBuilder<C> *set_type_graph(const GraphType &t)
    {
        std::cout << "before this->typeGraph = " << this->typeGraph << "\n";
        this->typeGraph = t;
        std::cout << "after this->typeGraph = " << this->typeGraph << "\n";
        return this;
    }

    MKCGraphBuilder<C> *set_graph_input_file(const std::string &input_file)
    {
        this->input_file = input_file;
        return this;
    }

    MKCGraphBuilder<C> *set_graph_input_file(const char *input_file_c)
    {
        this->input_file = "";
        for (int i = 0; input_file_c[i]; ++i)
            this->input_file += input_file_c[i];
        return this;
    }

    C *end_graph()
    {
        return this->parent;
    }

    std::string get_file_name()
    {
        return input_file;
    }

    MKCGraph build()
    {
        Edges edges(0);

        try
        {
            read_instance(&edges);

            std::cout << "this->typeGraph = " << this->typeGraph << "\n";

            switch (this->typeGraph)
            {
            case (GraphType::CHORDAL):
                edges.set_edges_to_chordal_graph();
                break;
            case (GraphType::COMPLETE):
                edges.set_edges_to_complete_graph();
                break;
            default:
                break;
            }
        }
        catch (Exception e)
        {
            e.execute();
        }
        catch (...)
        {
            Exception e = Exception("Not handled exception \n", ExceptionType::STOP_EXECUTION);
            e.execute();
        }

        return MKCGraph(edges, this->typeGraph);
    }

    void read_instance(Edges *edges)
    {
        int vi, vj,
            index = 0,
            dim, nb_edges;
        double weight;

        std::ifstream fichier(this->input_file.c_str(), std::ios::in);

        if (!fichier) //file exception
        {
            throw Exception("Input file: " + this->input_file + ". Nonexistent", ExceptionType::STOP_EXECUTION);
        }

        fichier >> dim;
        fichier >> nb_edges; //no used, but can be used to improve allocation of edges.

        edges->set_number_vertices(dim);

        for (int i = 0; i < nb_edges && !fichier.eof(); ++i)
        {
            fichier >> vi;
            fichier >> vj;
            fichier >> weight;

            if (vi != vj && !maxkcut::MKCUtil::isZero(weight))
            {
                edges->add_edge(vi, vj, weight);
            }

            vi = vj = -1;
        }

        /*End of fichier*/
        fichier.close();
    }

    ~MKCGraphBuilder(){};
};
} // namespace maxkcut

#endif