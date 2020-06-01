#ifndef PROBLEM_PARAM_HPP
#define PROBLEM_PARAM_HPP

#include <string>
#include <iostream> // std::cout
#include <fstream>  // std::ifstream
#include <vector>   // std::ifstream

#include "../Utils/UtilsString.hpp"
#include "../Utils/Logger.hpp"

enum Parameters_max_k_cut_problem
{
    name_file_graph,
    partition_number,
    solver_type,
    formulation,
    has_triangle_inequalities,
    has_clique_inequalities,
    has_wheel_inequalities,
    NO_PROBLEM_PARAMETER
};

enum Problem_Solver_type //Select next branch
{
    SDP_SOLVER_TYPE = 1,        // SDP
    LP_SOLVER_TYPE = 2,         // LP
    LP_SDP_EIG_SOLVER_TYPE = 4, // LP with SDP based constraints
};

enum Problem_Formulation_type //Select next branch
{
    edge_only = 1,               // Edge formulatin proposed by Chopra
    node_edge = 2,               // Node and edge formulation proposed by Chopra
    extended_representative = 4, // Extended formulations proposed in http://dx.doi.org/10.1016/j.endm.2016.03.044*/
};

class MKC_ProblemParam
{
private:
    std::string input_graph_file_path = "instance7.txt";
    const std::string file_path_parameters;
    int partition_number = 3;
    Problem_Solver_type solver_type = Problem_Solver_type::LP_SDP_EIG_SOLVER_TYPE;
    Problem_Formulation_type formulation = Problem_Formulation_type::edge_only;
    bool consider_triangle_inequalities = true;
    bool consider_clique_inequalities = true;
    bool consider_wheel_inequalities = true;


public:
    MKC_ProblemParam(const std::string path) : file_path_parameters(path)
    {
        set_parameters_from_file(path);
    }

    MKC_ProblemParam *set_input_file_path(const std::string &path)
    {
        input_graph_file_path = path;
        validation();
        return this;
    }

    MKC_ProblemParam *set_number_of_partitions(const int &K)
    {
        partition_number = K;
        validation();
        return this;
    }

    const auto &get_input_graph_file() const
    {
        return input_graph_file_path;
    }

    const auto &get_number_partitions() const
    {
        return partition_number;
    }

    const auto &get_solver_type() const
    {
        return solver_type;
    }

    const auto &get_problem_formulation() const
    {
        return formulation;
    }

    const bool &has_triangle_inequalities() const
    {
        return consider_triangle_inequalities;
    }

    const bool &has_clique_inequalities() const
    {
        return consider_clique_inequalities;
    }

    const bool &has_wheel_inequalities() const
    {
        return consider_wheel_inequalities;
    }

    std::string get_solver_str() const 
    {
        switch (get_solver_type())
        {
        case Problem_Solver_type::SDP_SOLVER_TYPE :
            return "SDP";
        case Problem_Solver_type::LP_SOLVER_TYPE :
            return "LP";
        case Problem_Solver_type::LP_SDP_EIG_SOLVER_TYPE :
            return "LP_SDP";
        default:
            return "NONE";
        }
    }

    MKC_ProblemParam *set_parameters_from_file(const std::string file_name)
    {
        std::ifstream file(file_name.c_str());
        if (file.is_open())
        {
            std::string line;
            std::string delimiter = " ";
            while (std::getline(file, line))
            {
                // using printf() in all tests for consistency

                std::vector<std::string> splited = utils::split_string(line, delimiter);

                if ((splited.size() >= 3) && (splited[0].compare("#") != 0))
                {
                    switch (get_parameters_problem(splited[0]))
                    {
                    case Parameters_max_k_cut_problem::name_file_graph:
                        this->input_graph_file_path = splited[2];
                        break;
                    case Parameters_max_k_cut_problem::partition_number:
                        this->partition_number = std::stoi(splited[2]);
                        break;
                    case Parameters_max_k_cut_problem::solver_type:
                        this->solver_type = get_parameter_Solver_type(splited[2]);
                        break;
                    case Parameters_max_k_cut_problem::formulation:
                        this->formulation = get_parameter_Formulation_type(splited[2]);
                        break;
                    case Parameters_max_k_cut_problem::has_triangle_inequalities:
                        this->consider_triangle_inequalities = splited[2] == "true" ? true : false;
                        break;
                    case Parameters_max_k_cut_problem::has_clique_inequalities:
                        this->consider_clique_inequalities = splited[2] == "true" ? true : false;
                        break;
                    case Parameters_max_k_cut_problem::has_wheel_inequalities:
                        this->consider_wheel_inequalities = splited[2] == "true" ? true : false;
                        break;

                    default:
                        break;
                    }
                }
            }
            file.close();
        }
        else
        {
            Log::INFO("Parameter file of BranchBound not found, using default parameters \n " + file_name);
            return this;
        }

        validation();

        return this;
    }

    void validation()
    {
        switch (this->solver_type)
        {
        case Problem_Solver_type::SDP_SOLVER_TYPE:
            if (this->formulation != Problem_Formulation_type::edge_only)
            {
                Log::WARN("For SDP, only edge_only formulation is implemented, so fomulation change do edge_only");
                this->formulation = Problem_Formulation_type::edge_only;
            }
            break;
        case Problem_Solver_type::LP_SDP_EIG_SOLVER_TYPE:
            if (this->formulation == Problem_Formulation_type::extended_representative)
            {
                Log::WARN("For extended_representative formulation, only LP solver is implemented, so solver = LP");
                this->solver_type = Problem_Solver_type::LP_SOLVER_TYPE;
            }
            break;
        default:
            break;
        }

        if (this->formulation == Problem_Formulation_type::edge_only &&
            !consider_clique_inequalities)
        {
            Log::WARN("Clique must be activated in edge only formulation");
            this->consider_clique_inequalities = true;
        }

        if (partition_number < 2)
        {
            Log::ERROR("Number of partitions should be greater than 1");
        }
    }

    std::string to_string() const
    {
        std::string str = "";

        return str;
    }

    ~MKC_ProblemParam() {}

private:
    Parameters_max_k_cut_problem get_parameters_problem(std::string input)
    {
        if (input == "file_name")
            return Parameters_max_k_cut_problem::name_file_graph;
        if (input == "partition_number")
            return Parameters_max_k_cut_problem::partition_number;
        if (input == "solver_type")
            return Parameters_max_k_cut_problem::solver_type;
        if (input == "formulation")
            return Parameters_max_k_cut_problem::formulation;
        if (input == "has_triangle_inequalities")
            return Parameters_max_k_cut_problem::has_triangle_inequalities;
        if (input == "has_clique_inequalities")
            return Parameters_max_k_cut_problem::has_clique_inequalities;
        if (input == "has_wheel_inequalities")
            return Parameters_max_k_cut_problem::has_wheel_inequalities;

        return Parameters_max_k_cut_problem::NO_PROBLEM_PARAMETER;
    }

    Problem_Solver_type get_parameter_Solver_type(std::string input)
    {
        if (input == "SDP")
            return Problem_Solver_type::SDP_SOLVER_TYPE;
        if (input == "LP")
            return Problem_Solver_type::LP_SOLVER_TYPE;
        if (input == "LP_SDP_EIG")
            return Problem_Solver_type::LP_SDP_EIG_SOLVER_TYPE;

        Log::WARN("Solver type " + input + " not considered, default = LP_SDP_EIG");

        return Problem_Solver_type::LP_SDP_EIG_SOLVER_TYPE;
    }

    Problem_Formulation_type get_parameter_Formulation_type(std::string input)
    {
        if (input == "edge_only")
            return Problem_Formulation_type::edge_only;
        if (input == "node_edge")
            return Problem_Formulation_type::node_edge;
        if (input == "extended_representative")
            return Problem_Formulation_type::extended_representative;

        Log::WARN("The formulation type " + input + " not yet implemented, set to edge_only");

        return Problem_Formulation_type::edge_only;
    }
};

#endif