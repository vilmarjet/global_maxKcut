#ifndef HEURISTIC_PARAM_HPP
#define HEURISTIC_PARAM_HPP

#include <string>
#include <iostream> // std::cout
#include <fstream>  // std::ifstream
#include <vector>   // std::ifstream

#include "../Utils/UtilsString.hpp"
#include "../Utils/Logger.hpp"


enum Parameters_heuristic
{
    heuristic,
    heuristic_max_time_seconds,
    NO_HEURISTIC_PARAMETER
};

enum Heuristic_type //Select next branch
{
    vns = 1,   //variable_neighborhood_search
    grasp = 2,  //Greedy Randomized Adaptive Search Procedure
    moh = 3, //MultipleSearch_Heuristic
    ich = 4,  // Iterative Clustering Heuristic.
};

class MKC_HeuristicParam
{
private:
    Heuristic_type heuristic = Heuristic_type::vns;;
    double heuristic_max_time_seconds = 10;
    bool verbose = false;

public:
    MKC_HeuristicParam(const std::string file_name): file_path(file_name)
    {
        set_parameters_from_file(file_name);
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
                    case Parameters_heuristic::heuristic:
                        this->heuristic = this->get_heuristic_type(splited[2]);
                        break;
                    case Parameters_heuristic::heuristic_max_time_seconds :
                        this->heuristic_max_time_seconds = std::stod(splited[2]);
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

        return str;
    }

    ~MKC_HeuristicParam() {}

private:
    Parameters_heuristic get_parameter_type(std::string input)
    {
        if (input == "heuristic")
            return Parameters_heuristic::heuristic;
        if (input == "max_time_seconds")
            return Parameters_heuristic::heuristic_max_time_seconds;

        return Parameters_heuristic::NO_HEURISTIC_PARAMETER;
    }

    Heuristic_type get_heuristic_type(std::string input)
    {
        if (input == "variable_neighborhood_search")
            return Heuristic_type::vns;
        if (input == "grasp_metaheuristic")
            return Heuristic_type::grasp;
        if (input == "multiple_search_Heuristic")
            return Heuristic_type::moh;
        if (input == "iterative_clustering_heuristic")
            return Heuristic_type::ich;

        Log::WARN("The heuristic type " + input + " not considered, default = VNS");
        
        return Heuristic_type::vns;
    }

    
};

#endif