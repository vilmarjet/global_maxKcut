#ifndef MKC_CPA_HPP
#define MKC_CPA_HPP

#include "./CPA/CuttingPlaneAlgo.hpp"

namespace maxkcut
{

class MKC_CPA : public CuttingPlaneAlgorithm
{
private:
    MKCInstance *instance;

public:
    MKC_CPA(Solver *solver_, MKCInstance *instance_) : CuttingPlaneAlgorithm(solver_),
                                                       instance(instance_) {}
    ~MKC_CPA() {}

    // From CuttingPlaneAlgorithm
    void find_violated_inequality()
    {
        
    }
};

} // namespace maxkcut

#endif
