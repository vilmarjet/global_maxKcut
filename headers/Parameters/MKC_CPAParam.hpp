#ifndef MKC_CPA_PARAM_HPP
#define MKC_CPA_PARAM_HPP

#include <string>
#include <iostream> // std::cout
#include <fstream>  // std::ifstream
#include <vector>   // std::ifstream

#include "../Utils/UtilsString.hpp"

enum Parameters_CPA
{
    max_number_iterations,
    max_time_seconds,
    max_time_per_iteration_seconds,
    max_number_violated_inequalities_par_iteration,
    number_iterations_without_clean_inequalities,
    verbose_cpa,
    is_early_termination,
    Default
};

class MKC_CPAParam
{
private:
    int max_number_iterations = 100;
    double max_time_seconds = 10;
    double max_time_per_iteration_seconds = 10;
    int max_number_violated_inequalities_par_iteration = 1000;
    int number_iterations_without_clean_inequalities = 0;
    std::string file_path;
    bool verbose = false;
    bool is_early_termination = false;

public:
    MKC_CPAParam(const std::string file_name)
    {
        this->file_path = file_name;
        set_parameters_from_file(file_name);
    }

    const int &get_number_iterations_to_clean() const
    {
        return this->number_iterations_without_clean_inequalities;
    }

    const int &get_max_number_iterations() const
    {
        return this->max_number_iterations;
    }

    const double &get_max_time_seconds() const
    {
        return this->max_time_seconds;
    }

    const double &get_max_time_per_iteration_seconds() const
    {
        return this->max_time_per_iteration_seconds;
    }

    const int &get_max_number_violated_inequalities_par_iteration() const
    {
        return this->max_number_violated_inequalities_par_iteration;
    }

    const bool & is_verbose() const 
    {
        return this->verbose;
    }

    void set_parameters_from_file(const std::string file_name)
    {
        std::ifstream fichier(file_name.c_str(), std::ios::in); // reading open

        if (!fichier) //si le fichier n'est existe pas
        {
            std::cout << "Parameter file not found \n " + file_name;
            std::cin.get();
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
                    switch (get_parameter(splited[0]))
                    {
                    case Parameters_CPA::max_number_iterations:
                        this->max_number_iterations = std::stoi(splited[2]);
                        break;
                    case Parameters_CPA::max_time_seconds:
                        this->max_time_seconds = std::stod(splited[2]);
                        break;
                    case Parameters_CPA::max_time_per_iteration_seconds:
                        this->max_time_per_iteration_seconds = std::stod(splited[2]);
                        break;
                    case Parameters_CPA::max_number_violated_inequalities_par_iteration:
                        this->max_number_violated_inequalities_par_iteration = std::stoi(splited[2]);
                        break;
                    case Parameters_CPA::number_iterations_without_clean_inequalities:
                        this->number_iterations_without_clean_inequalities = std::stoi(splited[2]);
                        break;
                    case Parameters_CPA::verbose_cpa:
                        this->verbose =  splited[2] == "true" ? true : false;
                        break;
                    case Parameters_CPA::is_early_termination:
                        this->is_early_termination =  splited[2] == "true" ? true : false;
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
        str += "max_number_iterations = " + std::to_string(max_number_iterations) + "\n";
        str += "max_time_seconds = " + std::to_string(max_time_seconds) + "\n";
        str += "max_time_per_iteration_seconds = " + std::to_string(max_time_per_iteration_seconds) + "\n";
        str += "max_number_violated_inequalities_par_iteration = " + std::to_string(max_number_violated_inequalities_par_iteration) + "\n";
        str += "number_iterations_without_clean_inequalities = " + std::to_string(number_iterations_without_clean_inequalities) + "\n";

        return str;
    }

    ~MKC_CPAParam() {}

private:
    Parameters_CPA get_parameter(std::string input) const
    {
        if (input == "max_number_iterations")
            return Parameters_CPA::max_number_iterations;
        if (input == "max_time_seconds")
            return Parameters_CPA::max_time_seconds;
        if (input == "max_time_per_iteration_seconds")
            return Parameters_CPA::max_time_per_iteration_seconds;
        if (input == "max_number_violated_inequalities_par_iteration")
            return Parameters_CPA::max_number_violated_inequalities_par_iteration;
        if (input == "number_iterations_without_clean_inequalities")
            return Parameters_CPA::number_iterations_without_clean_inequalities;
        if (input == "verbose")
            return Parameters_CPA::verbose_cpa;
        if (input == "is_early_termination")
            return Parameters_CPA::is_early_termination;
        

        return Parameters_CPA::Default;
    }
};

#endif