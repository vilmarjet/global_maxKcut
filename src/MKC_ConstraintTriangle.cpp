
#include "../headers/MKC_ConstraintTriangle.hpp"
#include <iostream>

MKC_ConstraintTriangle::MKC_ConstraintTriangle(const int &i, const int &j, const int &h,
                                               const int &xij, const int &xhj, const int &xhi,
                                               const double &val)
{
  _i = i;
  _j = j;
  _h = h;
  _xij = xij;
  _xhj = xhj;
  _xhi = xhi;
  _val = val;
}

bool MKC_ConstraintTriangle::operator<(const MKC_ConstraintTriangle &c1) const
{
  return (_val >= c1._val);
}

void MKC_ConstraintTriangle::printSimple() const
{
  std::cout << "Print Triangle inequality: ";
  std::cout << _xij << "*x_{" << _i << "," << _j << "} ";
  std::cout << _xhj << "*x_{" << _h << "," << _j << "} ";
  std::cout << _xhi << "*x_{" << _h << "," << _i << "} ";
  std::cout << " = " << _val;
}

void MKC_ConstraintTrianglePopulate::addIneq(const int &i, const int &j, const int &h,
                                const int &xij, const int &xhj, const int &xhi, const double &val)
{
  MKC_ConstraintTriangle Inq(i, j, h, xij, xhj, xhi, val);
  set_Ineq.insert(Inq);
}

int MKC_ConstraintTrianglePopulate::size()
{
  return set_Ineq.size();
}

std::vector<int> MKC_ConstraintTrianglePopulate::get_Vertices(const int &i)
{
  std::vector<int> aux;
  int counter = 0;
  if ((size() > 0) && (i <= size()))
    for (it = set_Ineq.begin(); it != set_Ineq.end(); ++it)
    {
      if (counter == i)
      {
        aux.push_back((*it)._i);
        aux.push_back((*it)._j);
        aux.push_back((*it)._h);
        return aux;
      }
      counter++;
    }

  { //erro
    std::cout << "Erro classe MKC_ConstraintTrianglePopulate is empty or trying to acess inexistente inequality in MKC_ConstraintTrianglePopulate::get_Inequality(const int & i)" << std::endl;
    exit(1);
  }
}
MKC_ConstraintTriangle MKC_ConstraintTrianglePopulate::get_Inequality(const int &i)
{
  int counter = 0;
  if (size() > 0)
    for (it = set_Ineq.begin(); it != set_Ineq.end(); ++it)
    {
      if (counter == i)
        return (*it);

      counter++;
    }

  { //erro
    std::cout << "Erro classe MKC_ConstraintTrianglePopulate is empty or trying to acess inexistente inequality in MKC_ConstraintTrianglePopulate::get_Inequality(const int & i)" << std::endl;
    exit(1);
  }
}

void MKC_ConstraintTrianglePopulate::printAll()
{
  int counter = 1;
  std::cout << std::endl
            << "Print the elements of the class MKC_ConstraintTrianglePopulate:" << std::endl;
  for (it = set_Ineq.begin(); it != set_Ineq.end(); ++it)
  {
    std::cout << counter << ": ";
    (*it).printSimple();
    std::cout << std::endl;
    counter++;
  }
}

void MKC_ConstraintTrianglePopulate::clearAll()
{
  set_Ineq.clear();
}