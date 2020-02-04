
#include "../headers/MKC_ConstraintWheel.hpp"
#include <iostream>

// Rcl to build wheel constraint
bool MKK_ConstraintRCLWheel::operator<(const MKK_ConstraintRCLWheel &c1) const
{
  return (val >= c1.val);
}

void MKC_ConstraintRCLWheelSupport::addElement(const int &ver, const double &va)
{
  MKK_ConstraintRCLWheel ele(ver, va);
  set_rcl.insert(ele);
}

int MKC_ConstraintRCLWheelSupport::getVertex(const int &i)
{
  int counter = 0;

  if (size() > 0)
    for (it = set_rcl.begin(); it != set_rcl.end(); ++it)
    {
      if (counter == i)
        return (*it).vertx;
      counter++;
    }

  { //erro
    std::cout << "Erro classe MKK_ConstraintRCLWheel is empty OR trying to acess inexistente element in MKK_ConstraintRCLWheel::getVertex(const int & i)" << std::endl;
    exit(1);
  }
}

int MKC_ConstraintRCLWheelSupport::getRandomVertex(const double &prop, const long int &rand)
{
  double proR = prop;
  if (size() > 0)
  {
    if (size() == 1)
      return getVertex(0);
    int counter = 0;
    double random;

    while (true)
    {
      random = ((double)(rand % 11) / 10);
      if (random <= proR)
        return getVertex(counter);
      counter++;

      if (counter >= size())
      {
        counter = 0;
        proR *= 1.2; // aumenta 20% the proportion
      }
    }
  } //end if size_t

  return -1; // there is no vertex
}

void MKC_ConstraintRCLWheelSupport::printAll()
{
  int counter = 1;
  std::cout << "Print vector RCL" << std::endl;
  if (size() > 0)
    for (it = set_rcl.begin(); it != set_rcl.end(); ++it)
    {
      std::cout << counter << ": " << (*it).vertx << " val=" << (*it).val << std::endl;
      counter++;
    }
}

void MKC_ConstraintRCLWheelSupport::printElement(const int &i)
{
  int counter = 0;
  std::cout << "RCL Print Elemento " << i << ", vertex =  ";
  if (size() > 0)
    for (it = set_rcl.begin(); it != set_rcl.end(); ++it)
    {
      if (counter == i)
      {
        std::cout << (*it).vertx << ", val=" << (*it).val << std::endl;
        it = set_rcl.end();
      }
      counter++;
    }
}

MKC_ConstraintWheel::MKC_ConstraintWheel(std::vector<int> vec, double &sT, double &vio)
{
  for (std::size_t i = 0; i < vec.size(); i++)
  {
    myVect.push_back(vec[i]);
  }
  sumTotal = sT;
  violation = vio;
}

MKC_ConstraintWheel::MKC_ConstraintWheel(std::vector<int> vec, const double &sT, const double &vio)
{
  myVect = vec;
  sumTotal = sT;
  violation = vio;
}

void MKC_ConstraintWheel::print() const
{
  std::cout << "{ ";
  for (std::size_t i = 0; i < myVect.size(); i++)
    std::cout << myVect[i] << " ";

  std::cout << "} "
            << ", sum =" << sumTotal << ", viol = " << violation << std::endl;
}

bool MKC_ConstraintWheel::operator<(const MKC_ConstraintWheel &c1) const
{
  return (violation >= c1.violation);
}



bool MKC_ConstraintWheelPopulate::SetNewWheel(MKC_ConstraintWheel wheel)
{
  int count = 0;

  for (it = myset.begin(); it != myset.end(); ++it)
  {
    count = 0;
    for (std::size_t j = 0; j < (*it).myVect.size(); j++)
    {
      if ((*it).myVect[j] == wheel.myVect[j])
        count++;
    }

    if (std::size_t(count) == (*it).myVect.size()) //find some equal
      return false;
  } //end for it

  myset.insert(wheel); // inserting
  return true;
}

bool MKC_ConstraintWheelPopulate::SetIneq_NoVerif(std::vector<int> &vec, const double &suT, const double &vio)
{
  int count = 0;
  MKC_ConstraintWheel Ineq(vec, suT, vio);

  myset.insert(Ineq); // inserting
  return true;
}

bool MKC_ConstraintWheelPopulate::SetIneq(std::vector<int> &vec, const double &suT, const double &vio)
{
  int count = 0;

  //Verifing if vec it is repeated, if yes dont include it (Maybe errase it)
  int key_ine = 0;
  for (std::size_t j = 0; j < vec.size(); j++)
  {
    if (j % 2 == 0)
      key_ine += (int)vec[j] / (j + 1);
    else
      key_ine += (int)vec[j] * (j);
  }

  if (key_ine >= key_Ineq.size())
    key_ine = key_ine % key_Ineq.size();

  if (key_Ineq[key_ine] == 1) // found same key
    return false;

  //else
  key_Ineq[key_ine] = 1;
  MKC_ConstraintWheel Ineq(vec, suT, vio);

  myset.insert(Ineq); // inserting
  return true;
}

int MKC_ConstraintWheelPopulate::size()
{
  return myset.size();
}

double MKC_ConstraintWheelPopulate::GetsumTotal(int i)
{
  int count = 0;
  for (it = myset.begin(); it != myset.end(); ++it)
  {
    if (count == i)
    {
      return (*it).sumTotal;
      it = myset.end();
    }
    count++;
  }

  {
    std::cout << "Erro in GetsumTotal(const int &i) of MKC_ConstraintWheelPopulate, access a element that out of range of this population " << std::endl;
    exit(0);
  }

  return 0.0;
}

std::vector<int> MKC_ConstraintWheelPopulate::GetVectorWheel(const int &i)
{

  int count = 0;
  if (i < size())
  {
    for (it = myset.begin(); it != myset.end(); ++it)
    {
      if (count == i)
        return (*it).myVect;
      count++;
    }
  }
  else
  {
    std::cout << "Erro in GetVectorWheel(const int &i) of MKC_ConstraintWheelPopulate, access a element that out of range of this population " << std::endl;
    exit(0);
  }

  { // not found
    std::cout << "Erro in GetVectorWheel(const int &i) of MKC_ConstraintWheelPopulate, access a element that out of range of this population " << std::endl;
    exit(0);
  }
  return GetVectorWheel(i);
}

void MKC_ConstraintWheelPopulate::EraseElement(int i)
{
  int count = 0;
  for (it = myset.begin(); it != myset.end(); ++it)
  {
    if (count == i)
    {
      myset.erase(it);
      it = myset.end();
    }
    count++;
  }
}

void MKC_ConstraintWheelPopulate::clearAll()
{
  myset.clear();

  for (unsigned i = 0; i < key_Ineq.size(); i++)
    key_Ineq[i] = 0;
}

MKC_ConstraintWheel MKC_ConstraintWheelPopulate::GetInequalityClass(int ii)
{
  it = myset.begin();

  for (int i = 0; i < ii; i++)
    ++it;

  return (*it);
}

void MKC_ConstraintWheelPopulate::printAll()
{
  std::cout << std::endl
            << "Imprimindo  elementos da  'class MKC_ConstraintWheelPopulate' :" << std::endl;
  std::cout << "seq | vec |  sum | viol" << std::endl;
  int count = 0;
  for (it = myset.begin(); it != myset.end() /* && count<50 */; ++it)
  {
    std::cout << count + 1 << "| ";
    (*it).print();
    count++;
  }
}

