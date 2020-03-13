#ifndef EDGES_FOR_GRAPH_HPP
#define EDGES_FOR_GRAPH_HPP

#include <string>
#include <vector>
#include <cstddef>
#include "Edge.hpp"
#include "MKCUtil.hpp"

namespace maxkcut
{
class Edges
{
private:
    std::vector<Edge> edges;
    std::vector<int> easy_index_edges;
    int number_vertices;
    std::vector<std::vector<int>> vertices_maximal_clique;

public:
    static Edges *create(const int &n_vertices)
    {
        return new Edges(n_vertices);
    }

    Edges(const int &n_vertices) : number_vertices(n_vertices)
    {
        edges.clear();
        //reset easy_index_edges
        easy_index_edges.clear();
        create_index_edges();
    }
    ~Edges() {}

    void set_number_vertices(const int &nb)
    {
        number_vertices = nb;
        easy_index_edges.clear();
        create_index_edges();
    }

    bool add_edge(const int &vi, const int &vj, const double &weight, bool sum_if_repeated = false)
    {
        if (vi > number_vertices || vj > number_vertices)
        {
            std::string msg = "\nVertices larger than dimension in Edges::dd_edge() or not possible.";
            msg += "Got (" + std::to_string(vi) + "," + std::to_string(vj) + ") \n";
            throw Exception(msg, ExceptionType::STOP_EXECUTION);
        }

        int pos_edge_in_easy_indexing = calculate_position_edge(vi, vj);

        if (this->easy_index_edges[pos_edge_in_easy_indexing] == -1)
        {
            edges.push_back(Edge(vi, vj, weight));
            this->easy_index_edges[pos_edge_in_easy_indexing] = edges.size() - 1;
            if (is_complete())
            {
                this->set_all_vertices_in_maximal_clique();
            }
        }
        else
        {
            if (sum_if_repeated == true)
            {
                int idx = this->easy_index_edges[pos_edge_in_easy_indexing];
                edges[idx].add_weight(weight);
            }
            else
            {
                return false;
            }
        }

        return true;
    }

    void resize(const int &size)
    {
        edges.resize(size);
    }

    int get_number_edges_if_complete_grpah() const
    {
        return get_number_edges_if_complete_grpah(this->number_vertices);
    }

    int get_number_edges_if_complete_grpah(const int &dim) const
    {
        return (int)(((dim - 1) * dim) / 2.0);
    }

    //return int with number of edges = edges.size()
    int get_number_edges() const
    {
        return this->edges.size();
    }

    //return number of vertices
    int get_number_vertices() const
    {
        return number_vertices;
    }

    //set in maximal_cliaque all vertices. Used if graph is complete
    void set_all_vertices_in_maximal_clique()
    {
        vertices_maximal_clique.clear();

        std::vector<int> all_vertices(this->number_vertices);

        for (int i = 0; i < this->number_vertices; ++i)
        {
            all_vertices[i] = i + 1; //vertices start by 1
        }

        vertices_maximal_clique.push_back(all_vertices);
    }

    //create index
    void create_index_edges()
    {
        this->easy_index_edges.resize(get_number_edges_if_complete_grpah(), -1);
    }

    /**
     * @param: a valid vertex
     * @return number of adjacent vertice (vertices with edge)   
     */
    int get_degree_vertice(const int &vertex)
    {
        int counter = 0;
        //vertex starts by 1
        for (int v = 1; v <= number_vertices; ++v)
        {
            if (get_edge_by_vertices(vertex, v) != nullptr)
            {
                counter++;
            }
        }

        return counter;
    }

    int get_vertex_min_degree(const std::vector<int> &degree_vertices,
                              const std::vector<bool> &is_alowed)
    {
        int min_degree = number_vertices + 1;
        int selected_vertex = -1;
        //vertices start by 1
        for (int v = 1; v <= number_vertices; ++v)
        {
            if (is_alowed[v] && degree_vertices[v] < min_degree)
            {
                min_degree = degree_vertices[v];
                selected_vertex = v;
            }
        }

        return selected_vertex;
    }

    bool is_complete()
    {
        return get_number_edges_if_complete_grpah() == get_number_edges();
    }

    //add edges to make a chordal graph
    //uses a heuristic approach: find all adjacents vertices of a vertex (min degree) and link all of them
    void set_edges_to_chordal_graph()
    {
        std::cout<< "chegou \n";
        if (this->is_complete())
        {
            return;
        }

        std::vector<int> vertices_degree(number_vertices + 1, 0);
        std::vector<bool> vertices_to_compute(number_vertices + 1, true);
        int vertex = -1;
        int min_size_degree = 2; //min number of vertices to be considered in maximal clique
        vertices_maximal_clique.clear();

        for (int v = 1; v <= number_vertices; ++v)
        {
            vertices_degree[v] = get_degree_vertice(v);
        }

        //loop all vertices (the last two are useless)
        for (int idx = 0; idx < number_vertices - 2; ++idx)
        {
            vertex = get_vertex_min_degree(vertices_degree, vertices_to_compute);
            vertices_to_compute[vertex] = false; //desactivate vertex

            std::cout << "vertex = " << vertex << "\n";

            if (vertices_degree[vertex] >= min_size_degree &&
                !is_vertex_in_maximal_clique(vertex))
            {
                std::vector<int> adjacent_vertices;
                fill_adjacent_vertices(vertex, &adjacent_vertices, vertices_to_compute);
                adjacent_vertices.push_back(vertex);
                add_subgraph_to_maximal_Clique(adjacent_vertices);

                MKCUtil::print_vector(adjacent_vertices);
                std::cout<< "\n";

                //setting up edges
                for (int i = 0; i < adjacent_vertices.size() - 1; ++i)
                {
                    int v_i = adjacent_vertices[i];
                    for (int j = i + 1; j < adjacent_vertices.size(); ++j)
                    {
                        int v_j = adjacent_vertices[j];
                        this->add_edge(v_i, v_j, 0.0);
                    }
                }
            }
        }
    }

    //adds edges to make a complete graph.
    void set_edges_to_complete_graph()
    {
        //loop all vertices
        for (int v_i = 1; v_i < number_vertices; ++v_i)
        {
            for (int v_j = v_i + 1; v_j <= number_vertices; ++v_j)
            {
                this->add_edge(v_i, v_j, 0.0);
            }
        }
    }

    void add_subgraph_to_maximal_Clique(const std::vector<int> &adjacent_vertices)
    {
        vertices_maximal_clique.push_back(adjacent_vertices);
    }

    void fill_adjacent_vertices(const int &vertex,
                                std::vector<int> *adjacent_vertices,
                                const std::vector<bool> &is_allowed)
    {
        (*adjacent_vertices).clear();

        for (int v = 1; v <= number_vertices; ++v)
        {
            if (get_edge_by_vertices(vertex, v) != nullptr)
            {
                (*adjacent_vertices).push_back(v);
            }
        }
    }

    const std::vector<std::vector<int>> *get_vector_of_maximal_cliques() const
    {
        return &vertices_maximal_clique;
    }

    bool is_vertex_in_maximal_clique(const int &vertex) const
    {
        for (int cq = 0; cq < vertices_maximal_clique.size(); ++cq)
        {
            if (find(vertices_maximal_clique[cq].begin(), vertices_maximal_clique[cq].end(), vertex) !=
                vertices_maximal_clique[cq].end())
            {
                return true;
            }
        }

        return false;
    }

    /*Attention: vertices must start by 1 in input file*/
    const Edge *get_edge_by_vertices(const int &i, const int &j) const
    {
        validate_vertices_i_j(i, j);

        int index = i != j ? this->easy_index_edges[calculate_position_edge(i, j)] : -1;

        if (index == -1)
        {
            return nullptr;
        }

        return this->get_edge_by_index(index);
    }

    bool validate_vertices_i_j(const int &vi, const int &vj) const
    {
        if (vi > this->number_vertices || vj > this->number_vertices || vj <= 0 || vi <= 0)
        {
            std::string msg = "\nVertices larger than dimension in Edges::validate_vertices_i_j() or not possible.";
            msg += "\nGot (" + std::to_string(vi) + "," + std::to_string(vj) + ") \n";
            throw Exception(msg, ExceptionType::VERTEX_ZERO_OR_NEGATIVE);

            return false;
        }

        return true;
    }

    const Edge *get_edge_by_index(const int &index) const
    {
        if (index >= 0 && index < edges.size())
        {
            return &this->edges[index];
        }

        return nullptr; //not found
    }

    bool has_edge(const int &vi, const int &vj) const
    {
        return !(this->easy_index_edges[calculate_position_edge(vi, vj)] == -1);
    }

    double sum_cost_all_edges() const
    {
        double sum = 0.0;
        for (int i = 0; i < this->edges.size(); ++i)
        {
            sum += edges[i].get_weight();
        }

        return sum;
    }

    /**
     * @info vi must be inferior than vj, vertices start by 1
     * @param vi and vj are vertices of grpah 
     **/
    int calculate_position_edge(const int &i, const int &j) const
    {
        int vi = i,
            vj = j;
        if (vi > vj)
        {
            std::swap(vi, vj);
        }

        int max_vi = get_number_edges_if_complete_grpah(vi);
        return ((((vj - 1) + (vi - 1) * this->number_vertices) - max_vi)) - vi;
    }

    inline Edges operator=(Edges e)
    {
        this->edges = e.edges;
        this->easy_index_edges = e.easy_index_edges;
        this->number_vertices = e.number_vertices;
        this->vertices_maximal_clique = e.vertices_maximal_clique;

        return e;
    }

    std::string to_string() const
    {
        std::string s;
        s += "Edges: \n";
        for (int i = 0; i < get_number_edges(); ++i)
        {
            int vi = this->edges[i].get_vertex_i();
            int vj = this->edges[i].get_vertex_j();

            s += this->edges[i].to_string() + "\n";
        }

        return s;
    }
};
} // namespace maxkcut

#endif