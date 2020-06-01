#ifndef MKC_INEQ_SDP_DIAGONAL_HPP
#define MKC_INEQ_SDP_DIAGONAL_HPP


#include "./Solver/Abstract/Solver.hpp"
#include "./Solver/Constraint/ConstraintSDP.hpp"
#include "./Solver/Variable/Variable.hpp"


class MKC_InequalitySDPDiagonal
{
private:
    Solver *solver;
    MKC_InequalitySDPDiagonal(Solver *solver_) : solver(solver_)
    {
    }

public:
    static MKC_InequalitySDPDiagonal *create(Solver *solver_)
    {
        return new MKC_InequalitySDPDiagonal(solver_);
    }

    MKC_InequalitySDPDiagonal *populate()
    {
        const SDPVariables *sdp_vars = solver->get_sdp_variables();
        const SDPVariable<Variable> *sdp_var = sdp_vars->get_variable(0); //

        int dim = sdp_var->get_dimension();
        double lowerBound = 1.0;
        double upperBound = 1.0;
        ConstraintBoundKey type = ConstraintBoundKey::EQUAL;
        double coeff = 1.0;

        for (int i = 0; i < dim; ++i)
        {
            const Variable *variable = sdp_var->get_variable(i, i);
            ConstraintSDP *constraint = solver->add_constraint_SDP(ConstraintSDP::create(lowerBound, upperBound, type));
            constraint->add_coefficient(sdp_var, variable, coeff);
        }

    }

    ~MKC_InequalitySDPDiagonal() {}
};

#endif