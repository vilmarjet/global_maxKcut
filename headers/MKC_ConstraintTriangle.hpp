#ifndef TRITRI
#define TRITRI

#include <set>
#include <vector>

class MKC_ConstraintTriangle
{
public:
  int _i,
      _j,
      _h,
      _xij,
      _xhj,
      _xhi;
  double _val;

  MKC_ConstraintTriangle(){};
  MKC_ConstraintTriangle(const int &i, const int &j, const int &h,
                         const int &xij, const int &xhj, const int &xhi,
                         const double &val);
  bool operator<(const MKC_ConstraintTriangle &) const;
  void printSimple() const;
};

////////////////// Population of triangles *************/////

class MKC_ConstraintTrianglePopulate
{
public:
  std::set<MKC_ConstraintTriangle> set_Ineq;
  std::set<MKC_ConstraintTriangle>::iterator it;

  MKC_ConstraintTrianglePopulate() { set_Ineq.clear(); };
  void addIneq(const int &i, const int &j, const int &h,
               const int &xij, const int &xhj, const int &xhi, const double &val);
  int size();
  MKC_ConstraintTriangle get_Inequality(const int &);
  std::vector<int> get_Vertices(const int &);
  void printAll();
  void clearAll();

  MKC_ConstraintTrianglePopulate operator=(const MKC_ConstraintTrianglePopulate &b)
  {
    MKC_ConstraintTrianglePopulate box;
    box.set_Ineq = b.set_Ineq;
    return box;
  }

  //void clearAll();
};


#endif
