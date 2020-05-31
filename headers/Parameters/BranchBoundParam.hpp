#ifndef BB_PARAM_HPP
#define BB_PARAM_HPP

#include <string>
#include <iostream> // std::cout
#include <fstream>  // std::ifstream
#include <vector>   // std::ifstream

#include "../Utils/UtilsString.hpp"
#include "../Utils/Logger.hpp"

enum Parameters_type_BB
{
    selection_strategy,
    partition_strategy,
    branch_rule_strategy,
    solution_strategy,
    number_iterations_to_execute_cutting_plane,
    bb_max_time_seconds,
    number_iterations_to_compute_heuristic,
    verbose_bb,
    output_solution_file_bb,
    initial_feasible_solution,
    NOT_BB_PARAMETER
};

enum BB_Verbose_Type //Select next branch
{
    log_iterations_in_file = 1,           // log bb iteration in file (automaticaly created in Target)
    log_iterations_in_terminal = 2,       // log bb iterations in terminal only
    log_iterations_terminal_and_file = 3, // log iteration in terminal and file
    skip = 4,                             // Do not log iterations
};

enum BB_Selection_Strategy //Select next branch
{
    best_first = 1,   //best  rst search strategy
    worse_first = 2,  //Worse first search strategy
    breath_first = 3, //Breath first selection (all same level first)
    depth_first = 4,  // depth rst search (Goes deep in search tree)
};

enum BB_Rule_Strategy //Select next branch
{
    R1_MostDecided = 1,     // Most decided first
    R2_ArticleMG = 2,       //Best in MG
    R3_LeastDecided = 3,    //Least decided first
    R5_EdgeWeight = 5,      // Edge with biggest weight
    R6_StrongBrahching = 6, // Strong Branching
    R7_PseudoCost = 7,      // PseudoCost
};

enum BB_Solution_Strategy //Select next branch
{
    EAGER_BB = 1, //best  rst search strategy
    LAZY_BB = 2,  // Compute before
};

enum BB_Partition_Strategy //Select next branch
{
    DICHOTOMIC = 1, // Each branch generates 2 branches
    K_CHOTOMIC = 2  // Each branch generates k branches
};

class BranchBoundParam
{
private:
    BB_Selection_Strategy selection_strategy = BB_Selection_Strategy::best_first;
    BB_Partition_Strategy partition_strategy = BB_Partition_Strategy::DICHOTOMIC;
    BB_Rule_Strategy branch_rule_strategy = BB_Rule_Strategy::R5_EdgeWeight;
    BB_Solution_Strategy solution_strategy = BB_Solution_Strategy::LAZY_BB;
    int number_iterations_to_execute_cutting_plane = 1;
    const std::string file_path;
    double max_time_seconds = 10;
    int number_iterations_to_compute_heuristic = 20;
    BB_Verbose_Type verbose = false;
    std::string get_output_solution_file_bb = "";

public:
    BranchBoundParam(const std::string file_name) : file_path(file_name)
    {
        set_parameters_from_file(file_name);
    }

    const double &get_max_time_seconds()
    {
        return this->max_time_seconds;
    }

    const int &get_number_iterations_to_compute_heuristic() const
    {
        return this->number_iterations_to_compute_heuristic;
    }

    const BB_Partition_Strategy &get_partition_strategy() const
    {
        return this->partition_strategy;
    }

    const BB_Rule_Strategy &get_branch_rule_strategy() const
    {
        return this->branch_rule_strategy;
    }

    const BB_Selection_Strategy &get_selection_strategy() const
    {
        return this->selection_strategy;
    }

    const auto &get_solution_strategy() const
    {
        return this->solution_strategy;
    }

    const int &get_number_iterations_execute_CPA() const
    {
        return this->number_iterations_to_execute_cutting_plane;
    }

    const auto &get_verbose_type() const
    {
        return this->verbose;
    }

    const auto &get_output_file_name_bb() const
    {
        return this->get_output_solution_file_bb;
    }

    bool is_verbose_terminal() const
    {
        return get_verbose_type() == BB_Verbose_Type::log_iterations_in_terminal ||
               get_verbose_type() == BB_Verbose_Type::log_iterations_terminal_and_file;
    }

    bool is_iteration_in_file() const
    {
        return get_verbose_type() == BB_Verbose_Type::log_iterations_in_file ||
               get_verbose_type() == BB_Verbose_Type::log_iterations_terminal_and_file;
    }

    void set_parameters_from_file(const std::string file_name)
    {
        std::ifstream fichier(file_name.c_str(), std::ios::in); // reading open

        if (!fichier) //si le fichier n'est existe pas
        {
            Log::INFO("Parameter file of BranchBound not found, using default parameters \n " + file_name);
            return;
        }

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
                    switch (get_parameter_type(splited[0]))
                    {
                    case Parameters_type_BB::selection_strategy:
                        this->selection_strategy = this->get_selection_strategy_type(splited[2]);
                        break;
                    case Parameters_type_BB::partition_strategy:
                        this->partition_strategy = this->get_partition_strategy_type(splited[2]);
                        break;
                    case Parameters_type_BB::branch_rule_strategy:
                        this->branch_rule_strategy = this->get_rule_strategy_type(splited[2]);
                        break;
                    case Parameters_type_BB::solution_strategy:
                        this->solution_strategy = this->get_solution_strategy_type(splited[2]);
                        break;
                    case Parameters_type_BB::number_iterations_to_execute_cutting_plane:
                        this->number_iterations_to_execute_cutting_plane = std::stoi(splited[2]);
                        break;
                    case Parameters_type_BB::bb_max_time_seconds:
                        this->max_time_seconds = std::stod(splited[2]);
                        break;
                    case Parameters_type_BB::number_iterations_to_compute_heuristic:
                        this->number_iterations_to_compute_heuristic = std::stoi(splited[2]);
                        break;
                    case Parameters_type_BB::verbose_bb:
                        this->verbose = get_verbose_type(splited[2]);
                        break;
                    case Parameters_type_BB::output_solution_file_bb:
                        this->output_solution_file_bb = splited[2];
                        break;
                    case Parameters_type_BB::initial_feasible_solution:
                        this->initial_feasible_solution = std::stod(splited[2]);
                        break;
                    default:
                        break;
                    }
                }
            }
            file.close();
        }
    }

    std::string to_string() const
    {
        std::string str = "";
        str += "file_path = " + file_path + "\n";
        str += "partition_strategy = " + std::to_string(partition_strategy) + "\n";
        str += "branch_rule_strategy = " + std::to_string(branch_rule_strategy) + "\n";
        str += "number_iterations_to_execute_cutting_plane = " + std::to_string(number_iterations_to_execute_cutting_plane) + "\n";
        str += "selection_strategy = " + std::to_string(selection_strategy) + "\n";
        str += "solution_strategy = " + std::to_string(solution_strategy) + "\n";

        return str;
    }

    ~BranchBoundParam() {}

private:
    Parameters_type_BB get_parameter_type(std::string input)
    {
        if (input == "selection_strategy")
            return Parameters_type_BB::selection_strategy;
        if (input == "partition_strategy")
            return Parameters_type_BB::partition_strategy;
        if (input == "branch_rule_strategy")
            return Parameters_type_BB::branch_rule_strategy;
        if (input == "solution_strategy")
            return Parameters_type_BB::solution_strategy;
        if (input == "number_iterations_to_execute_cutting_plane")
            return Parameters_type_BB::number_iterations_to_execute_cutting_plane;
        if (input == "max_time_seconds")
            return Parameters_type_BB::bb_max_time_seconds;
        if (input == "number_iterations_to_compute_heuristic")
            return Parameters_type_BB::number_iterations_to_compute_heuristic;
        if (input == "verbose")
            return Parameters_type_BB::verbose_bb;
        if (input == "output_file_name")
            return Parameters_type_BB::output_solution_file_bb;
        if (input == "initial_feasible_solution")
            return Parameters_type_BB::initial_feasible_solution;

        return Parameters_type_BB::NOT_BB_PARAMETER;
    }

    BB_Selection_Strategy get_selection_strategy_type(std::string input)
    {
        if (input == "best_first")
            return BB_Selection_Strategy::best_first;
        if (input == "worse_first")
            return BB_Selection_Strategy::worse_first;
        if (input == "breath_first")
            return BB_Selection_Strategy::breath_first;
        if (input == "depth_first")
            return BB_Selection_Strategy::depth_first;

        Log::WARN("Selection type of branch and bound (" + input + ") not considered, set as default = best_first");

        return BB_Selection_Strategy::best_first;
    }

    BB_Partition_Strategy get_partition_strategy_type(std::string input)
    {
        if (input == "DICHOTOMIC")
            return BB_Partition_Strategy::DICHOTOMIC;
        if (input == "K_CHOTOMIC")
            return BB_Partition_Strategy::K_CHOTOMIC;

        Log::WARN("Partition type of branch and bound (" + input + ") not considered, set to default = DICHOTOMIC");

        return BB_Partition_Strategy::DICHOTOMIC;
    }

    BB_Rule_Strategy get_rule_strategy_type(std::string input)
    {
        if (input == "R1_MostDecided")
            return BB_Rule_Strategy::R1_MostDecided;
        if (input == "R2_ArticleMG")
            return BB_Rule_Strategy::R2_ArticleMG;
        if (input == "R3_LeastDecided")
            return BB_Rule_Strategy::R3_LeastDecided;
        if (input == "R5_EdgeWeight")
            return BB_Rule_Strategy::R5_EdgeWeight;
        if (input == "R6_StrongBrahching")
            return BB_Rule_Strategy::R6_StrongBrahching;
        if (input == "R7_PseudoCost")
            return BB_Rule_Strategy::R7_PseudoCost;

        Log::WARN("Rule type of branch and bound (" + input + ") not considered, set as default = R1_MostDecided");

        return BB_Rule_Strategy::R1_MostDecided;
    }

    BB_Solution_Strategy get_solution_strategy_type(std::string input)
    {
        if (input == "EAGER_BB")
            return BB_Solution_Strategy::EAGER_BB;
        if (input == "LAZY_BB")
            return BB_Solution_Strategy::LAZY_BB;

        Log::WARN("Solution selection type of branch and bound (" + input + ") not considered, set default = LAZY_BB");

        return BB_Solution_Strategy::LAZY_BB;
    }

    BB_Verbose_Type get_verbose_type(std::string input)
    {
        if (input == "log_iterations_in_file")
            return BB_Verbose_Type::log_iterations_in_file;
        if (input == "log_iterations_in_terminal")
            return BB_Verbose_Type::log_iterations_in_terminal;
        if (input == "log_iterations_terminal_and_file")
            return BB_Verbose_Type::log_iterations_terminal_and_file;
        if (input == "skip")
            return BB_Verbose_Type::skip;
        if (input == "no_log_of_iterations")
            return BB_Verbose_Type::skip;

        Log::WARN("Verbose type of branch and bound (" + input + ") not considered, set default = skip");

        return BB_Verbose_Type::skip;
    }
};

#endif