#ifndef MKC_RANKVERTICES
#define MKC_RANKVERTICES

#include <time.h>       /* time */
#include <stdlib.h>     /* srand, rand */


///CLASSE rank vertex
class MKC_InstanceRankVertices
{
  public:
    int 		vertex;
    double		weight; //can be degree or total weigth of incident edges
    
    MKC_InstanceRankVertices(){};
    MKC_InstanceRankVertices(const int & _vec,const double & _weight);
    bool operator< (const MKC_InstanceRankVertices&) const;
    void print() const ;
};

#endif 