#ifndef CLASS_LP_SDP_VIOLATED_CONSTRAINT_HPP
#define CLASS_LP_SDP_VIOLATED_CONSTRAINT_HPP

#include <vector>
#include <limits>
#include <set>
#include "../../../../myFiles/eigen/Eigen/Dense"
#include "../../../../myFiles/eigen/Eigen/Eigenvalues"
#include "../CPA/ViolatedConstraint.hpp"
#include "../MKCUtil.hpp"

class LP_SDPConstraint
{
private:
    double rhs;
    double EPSLON_EIG = 1e-4;
    double ratioViolationEigen = 2.0;
    bool binary_improvement = true;

public:
    LP_SDPConstraint() : rhs(0.0) {}
    ~LP_SDPConstraint() {}

    void fill_ViolatedConstraint_Eigen(const double &coeff_lp_to_sdp, //(double)K / (K - 1); 1.0 = null
                                       const double &const_lp_to_sdp, // (1.0 / (K - 1)); 0.0= null
                                       const std::vector<std::vector<const Variable *>> &sym_matr_variables,
                                       std::set<ViolatedConstraint *, CompViolatedConstraint> *violated_constraints)
    {
        int dim = sym_matr_variables.size();
        int number_variables_constraint = (dim * (dim - 1)) / 2;
        double eigVal, val_ij;

        Eigen::MatrixXd mat_X(dim, dim);
        Eigen::VectorXd vecXd(dim);
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver;

        std::vector<const Variable *> variables(number_variables_constraint);
        std::vector<double> coefficients(number_variables_constraint, 0.0);

        //set variables and MatrixXd;
        int counter = 0;
        for (int i = 0; i < dim; ++i)
        {
            for (int j = i + 1; j < dim; ++j)
            {
                if (sym_matr_variables[i][j] == nullptr)
                {
                    throw Exception("NonExistent Variable in fill_ViolatedConstraint_Eigen",
                                    ExceptionType::STOP_EXECUTION);
                }
                //set variables
                variables[counter++] = sym_matr_variables[i][j];

                //set symetric matrix
                val_ij = sym_matr_variables[i][j]->get_solution() * coeff_lp_to_sdp - const_lp_to_sdp;
                mat_X(i, j) = val_ij;
                mat_X(j, i) = val_ij;
            }

            mat_X(i, i) = 1.0;
        }

        //Compute the eigenvalue and eigenvector
        eigen_solver.compute(mat_X);
        for (int i = 0; i < dim; ++i)
        {
            eigVal = eigen_solver.eigenvalues()[i];
            // eigenvalue is negatif, so there is a violation of the Semidefiniteness
            if (eigVal < -EPSLON_EIG)
            {
                this->set_coefficients_and_rhs_eigen(coeff_lp_to_sdp,
                                                     const_lp_to_sdp,
                                                     &coefficients,
                                                     eigen_solver.eigenvectors().col(i));

                violated_constraints->insert(new ViolatedConstraint(variables,
                                                                    coefficients,
                                                                    this->rhs,
                                                                    (double)dim,
                                                                    ConstraintType::SUPERIOR_EQUAL,
                                                                    ratioViolationEigen * (-1.0) * eigVal));
            }
        } //end for
    }

    void set_coefficients_and_rhs_eigen(const double &coeff_lp_to_sdp, //(double)K / (K - 1)
                                        const double &const_lp_to_sdp, //(1.0 / (K - 1))
                                        std::vector<double> *coefficients,
                                        const Eigen::VectorXd &eig_vec)
    {
        this->rhs = 0.0;
        int counter = 0;
        double valvp, val_coef;
        double min_coef = std::numeric_limits<double>::max();

        for (int i = 0; i < eig_vec.size(); ++i)
        {
            for (int j = i + 1; j < eig_vec.size(); ++j, ++counter)
            {
                valvp = 2.0 * eig_vec[i] * eig_vec[j];
                val_coef = valvp * coeff_lp_to_sdp;

                (*coefficients)[counter] = val_coef;  //coefficient
                this->rhs += valvp * const_lp_to_sdp; //it is minus

                //to improve rhs
                if (binary_improvement && val_coef < min_coef)
                {
                    min_coef = val_coef;
                }
            }

            //set in rhs the (i, i)
            this->rhs -= eig_vec[i] * eig_vec[i] * (1.0);
        } //end j

        //improve rhs if bynary and min coef. of a zero variable is greater than rhs
        if (min_coef >= this->EPSLON_EIG &&
            this->rhs >= this->EPSLON_EIG &&
            this->rhs < min_coef)
        {
            this->rhs = min_coef;
        }
    }
};

#endif