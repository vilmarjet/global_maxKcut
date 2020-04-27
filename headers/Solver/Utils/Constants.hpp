#ifndef CONSTANTS_SOLVER_HPPP
#define CONSTANTS_SOLVER_HPPP

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
