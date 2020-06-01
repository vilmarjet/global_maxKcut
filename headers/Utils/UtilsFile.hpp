#ifndef UTILS_FILE_HPP
#define UTILS_FILE_HPP

#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>

namespace utils
{
static bool is_empty(std::ifstream* pFile)
{
    return pFile->peek() == std::ifstream::traits_type::eof();
}

static int dirExists(const char *path)
{
    struct stat info;

    if(stat( path, &info ) != 0)
        return 0;
    else if(info.st_mode & S_IFDIR)
        return 1;
    else
        return 0;
}

static void make_dir(std::string path)
{
    mkdir(path.c_str(), 0777);
}
}

#endif