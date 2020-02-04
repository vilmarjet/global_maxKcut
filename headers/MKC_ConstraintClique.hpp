#ifndef CLICLI
#define CLICLI

#include <set>
#include <vector>

template <typename T>
inline void printvector_incliaque(std::vector<T> v);

class MKC_ConstraintClique
{
public:
  int Q;
  std::vector<int> vec_vertx;
  double sum, //right side of violated Inequality
      b;

  MKC_ConstraintClique(){};
  MKC_ConstraintClique(int *, int, double &);
  MKC_ConstraintClique(const std::vector<int> &vec, const int &q, const double &s);
  bool operator<(const MKC_ConstraintClique &) const;
  std::vector<int> GetVector() const;
  inline void set_b(const double &bb) { b = bb; }
};

/// POPULATION class --- this is class tha we will use in the main

class MKC_ConstraintCliquePopulate
{
public:
  std::set<MKC_ConstraintClique> set_Ineq;
  std::set<MKC_ConstraintClique>::iterator it;

  MKC_ConstraintCliquePopulate() { set_Ineq.clear(); };
  void addIneq(const std::vector<int> vec_vertices, const int &q, const double &s, const double &cst);
  MKC_ConstraintClique get_Inequality(const int &);
  std::vector<int> get_vecVertx_Inequality(const int &);
  void printAll();
  void clearAll();
  int size();
  //void clearAll();

  MKC_ConstraintCliquePopulate operator=(const MKC_ConstraintCliquePopulate &b)
  {
    MKC_ConstraintCliquePopulate box;
    box.set_Ineq = b.set_Ineq;
    return box;
  }
};

#endif
