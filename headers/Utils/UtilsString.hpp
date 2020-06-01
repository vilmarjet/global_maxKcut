#ifndef UTILS_STRING_HPP
#define UTILS_STRING_HPP

#include <vector>
#include <string>

namespace utils
{
    static std::vector<std::string> split_string(const std::string &str, const std::string &delim)
    {
        std::vector<std::string> tokens;
        size_t prev = 0, pos = 0;
        do
        {
            pos = str.find(delim, prev);
            if (pos == std::string::npos)
                pos = str.length();
            std::string token = str.substr(prev, pos - prev);
            if (!token.empty())
                tokens.push_back(token);
            prev = pos + delim.length();
        } while (pos < str.length() && prev < str.length());

        return tokens;
    }
} // namespace utils

#endif