#include "../headers/MKC_ConstraintClique.hpp"
#include <iostream>


MKC_ConstraintClique::MKC_ConstraintClique(const std::vector<int> &vec,const int &q ,const double &s)
{
  sum = s;
  Q = vec.size();
  //new vector
  vec_vertx = vec;
}

bool MKC_ConstraintClique::operator<(const MKC_ConstraintClique& c1) const{
  return (sum <= c1.sum);	
}

inline std::vector<int> MKC_ConstraintClique::GetVector() const
{
  return vec_vertx;
}


bool operator==(MKC_ConstraintClique const& a, MKC_ConstraintClique const& b)
{
  int count = 0;
  if ((a.Q == b.Q)&&(a.sum == b.sum)){
    for (int i=0;i<a.Q;i++){
      if (a.vec_vertx[i] == b.vec_vertx[i])
	count ++;
    }//end for i;
  
    if (count == a.Q)
      return true;
    
  }//END if
  
  return false;
}


//Population of clique constraints


void MKC_ConstraintCliquePopulate::addIneq(const std::vector<int> vec_vertices, const int &q, const double &s, const double &cst)
{
  MKC_ConstraintClique Inq(vec_vertices, q, s);
  Inq.set_b(cst);

  set_Ineq.insert(Inq);
}


std::vector<int> MKC_ConstraintCliquePopulate::get_vecVertx_Inequality(const int & i) 
{
  return get_Inequality(i).GetVector();
}


MKC_ConstraintClique MKC_ConstraintCliquePopulate::get_Inequality(const int & i)
{
  int counter = 0;
  if (size() > 0)
    for (it=set_Ineq.begin(); it!=set_Ineq.end() ; ++it){
      if(counter == i)
	return (*it);
      
      counter ++;
    }
  
  {//erro
  std::cout <<  "Erro classe MKC_ConstraintCliquePopulate is empty or trying to acess inexistente inequality in MKC_ConstraintCliquePopulate::get_Inequality(const int & i)" <<std::endl;
  exit(1);
  }
}
  

int MKC_ConstraintCliquePopulate::size()
{
  return set_Ineq.size();
}

void MKC_ConstraintCliquePopulate::clearAll()
{
  set_Ineq.clear();
}

void MKC_ConstraintCliquePopulate::printAll()
{
  int counter = 0;
  std::cout<< std::endl << "Print the elements of the class MKC_ConstraintCliquePopulate: ... " << size() << std::endl;
  
  for (it=set_Ineq.begin(); it!=set_Ineq.end() ; ++it){
    std::cout << counter << " ";
    printvector_incliaque((*it).GetVector());
    std::cout << " sum = " << (*it).sum ;
    std::cout << std::endl;
    counter ++;
  }
}


template <typename T>
void printvector_incliaque(std::vector<T> v) 
{
    std::cout << " : {";
        for(std::size_t j = 0; j < v.size(); j++) {
           std:: cout << v[j];
	    if (j <v.size()-1)
	      std::cout << ", ";
        }
        std::cout << "};";
}

