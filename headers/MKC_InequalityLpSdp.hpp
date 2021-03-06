#ifndef LP_SDP_INEQUALITY_HPP
#define LP_SDP_INEQUALITY_HPP

#include <vector>
#include <string>
#include <algorithm>
#include "./CPA/ViolatedConstraints.hpp"
#include "./MKCUtil.hpp"
#include "./Utils/Exception.hpp"
#include "./MKCGraph.hpp"
#include "./Heuristics/Tabu.hpp"
#include <cmath> // std::abs
#include "./Solver/Constraint/EigenValue/LP_SDPConstraint.hpp"

namespace maxkcut
{
class MKC_InequalityLpSdp : public ViolatedConstraints
{

public:
    static MKC_InequalityLpSdp *create()
    {
        return new MKC_InequalityLpSdp(nullptr, nullptr);
    }

    static MKC_InequalityLpSdp *create(const VariablesEdge *variables_, const MKCInstance *instance_)
    {
        return new MKC_InequalityLpSdp(variables_, instance_);
    }

private:
    const VariablesEdge *variables;
    const MKCInstance *instance;
    double rhs;

    MKC_InequalityLpSdp(const VariablesEdge *variables_,
                        const MKCInstance *instance_) : variables(variables_),
                                                        instance(instance_),
                                                        rhs(0.0)

    {
    }

public:
    ~MKC_InequalityLpSdp() {}

    std::string to_string() const
    {
        return typeid(this).name();
    }

    void find_violated_constraints()
    {
        LP_SDPConstraint lp_sdp_constraint;
        const MKCGraph *graph = instance->get_graph();
        const std::vector<std::vector<int>> *maximal_clique = graph->get_edges()->get_vector_of_maximal_cliques();

        //coefficients and constants for eigen value calculation
        int K = instance->get_K();
        double coeff_lp_to_sdp = (double)K / (K - 1);
        double const_lp_to_sdp = (1.0 / (K - 1));

        for (int clq = 0; clq < maximal_clique->size(); ++clq)
        {
            int size_vertices = (*maximal_clique)[clq].size();
            //resizing symmetric matrix
            std::vector<std::vector<const Variable *>> sym_matr_variables(size_vertices);
            for (int i = 0; i < size_vertices; ++i)
            {
                sym_matr_variables[i].resize(size_vertices);
            }

            //filling symmetric matrix
            for (int i = 0; i < size_vertices; ++i)
            {
                sym_matr_variables[i][i] = nullptr;

                for (int j = i + 1; j < size_vertices; ++j)
                {
                    int vi = (*maximal_clique)[clq][i];
                    int vj = (*maximal_clique)[clq][j];
                    const Edge *edge = graph->get_edges()->get_edge_by_vertices(vi, vj);
                    const Variable *var = variables->get_variable(edge);
                    //set in symmetric matrix
                    sym_matr_variables[i][j] = var;
                    sym_matr_variables[j][i] = var;
                }
            }

            //sent to generic constraint creator
            lp_sdp_constraint.fill_ViolatedConstraint_Eigen(coeff_lp_to_sdp,
                                                            const_lp_to_sdp,
                                                            sym_matr_variables,
                                                            &violated_constraints);
        }
    }
};
} // namespace maxkcut
#endif