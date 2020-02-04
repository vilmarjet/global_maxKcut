#include "../headers/MKC_InstanceRankVertices.hpp"

MKC_InstanceRankVertices::MKC_InstanceRankVertices(const int & _vec,const double & _weight)
{
  vertex= _vec;
  weight= _weight;
}

bool MKC_InstanceRankVertices::operator<(const MKC_InstanceRankVertices& c1) const{
  return (weight>= c1.weight);	
}