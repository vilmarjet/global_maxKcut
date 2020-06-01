#ifndef CONSTANTS_SOLVER_HPPP
#define CONSTANTS_SOLVER_HPPP

#include <chrono>
#include <ctime>   

#define ISDEBUG true

#define DEBUG_MSG(x) do { \
  std::time_t end_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()); \
  if (ISDEBUG) { std::cout <<std::ctime(&end_time) <<  x << std::endl; } \
} while (0)

namespace CONSTANTS
{
static const double ZERO = 1e-6;
static const double EPSILON = 1e-3;
static const double INFINITY_DOUBLE = std::numeric_limits<double>::max();
static const double INFINITY_INT = std::numeric_limits<int>::max();

static bool is_zero(const double &d)
{
    return std::abs(d) < ZERO ? true : false;
}

} // namespace CONSTANTS
#endif
