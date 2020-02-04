#ifndef WHEELWHEEL
#define WHEELWHEEL

#include <time.h>   /* time */
#include <stdlib.h> /* srand, rand */
#include <set>
#include <vector>

///RCL
class MKK_ConstraintRCLWheel
{
public:
  int vertx;
  double val;

  MKK_ConstraintRCLWheel(){};
  MKK_ConstraintRCLWheel(const int &ver, const double &va)
  {
    vertx = ver;
    val = va;
  }
  inline void set(const int &ver, const double &va)
  {
    vertx = ver;
    val = va;
  }
  bool operator<(const MKK_ConstraintRCLWheel &) const;
};


class MKC_ConstraintRCLWheelSupport
{
public:
  std::set<MKK_ConstraintRCLWheel> set_rcl;
  std::set<MKK_ConstraintRCLWheel>::iterator it;

  MKC_ConstraintRCLWheelSupport(){};
  void addElement(const int &ver, const double &va);
  int getVertex(const int &i);
  void clear() { set_rcl.clear(); }
  int size() { return set_rcl.size(); }
  int getRandomVertex(const double &prop, const long int &rand);
  void printAll();
  void printElement(const int &i);
};


///CLASSE INEQUALTY WHEEL
class MKC_ConstraintWheel
{
public:
  std::vector<int> myVect;
  double sumTotal;
  double violation;
  double b;

  MKC_ConstraintWheel(){};
  MKC_ConstraintWheel(std::vector<int> vec, double &, double &);
  MKC_ConstraintWheel(std::vector<int> vec, const double &, const double &);
  void set_b(const double &bb) { b = bb; }
  bool operator<(const MKC_ConstraintWheel &) const;
  void print() const;
};


/** cLASSE TO CREAT POPULATION OF wheel VIOLATED */

class MKC_ConstraintWheelPopulate
{
public:
  std::set<MKC_ConstraintWheel> myset;
  std::vector<int> key_Ineq;
  std::set<MKC_ConstraintWheel>::iterator it;
  const static int sizeMaxKey = 1000;

  MKC_ConstraintWheelPopulate() { key_Ineq.resize(sizeMaxKey, 0); }
  bool SetNewWheel(MKC_ConstraintWheel);
  bool SetIneq(std::vector<int> &vec, const double &suT, const double &vio);
  int size();
  double GetsumTotal(int);
  std::vector<int> GetVectorWheel(const int &i);
  void EraseElement(int);
  void clearAll();
  MKC_ConstraintWheel GetInequalityClass(int ii);
  void printAll();
  bool SetIneq_NoVerif(std::vector<int> &vec, const double &suT, const double &vio);
  ~MKC_ConstraintWheelPopulate() { key_Ineq.clear(); }

  MKC_ConstraintWheelPopulate operator=(const MKC_ConstraintWheelPopulate &b)
  {
    MKC_ConstraintWheelPopulate box;
    box.myset = b.myset;
    box.key_Ineq = b.key_Ineq;
    return box;
  }
};

#endif