#include "../headers/MKC_ConstraintLPtoSDP.hpp"

#include <iostream>

MKC_ConstraintLPtoSDP::MKC_ConstraintLPtoSDP(const std::vector<double> &vec, const double &vio, const bool & real)
{
  eigVect = vec;
  eigVal = vio;
  NEG = real;
}

MKC_ConstraintLPtoSDP::MKC_ConstraintLPtoSDP(const Eigen::VectorXd &vec, const int &size, const double &vio, const bool & real)
{
  eigVect.resize(size);
  for (int i=0; i< size; i++ )
  	eigVect[i] = vec[i];
  eigVal = vio;
  NEG = real;
}

void MKC_ConstraintLPtoSDP::print() const
{
  std::cout << "{ ";
  for (std::size_t i=0; i< eigVect.size(); i++)
    std::cout << eigVect[i] << " ";
    
  std::cout << "} "<< "| eigVal = " << eigVal << std::endl;
}

bool MKC_ConstraintLPtoSDP::operator<(const MKC_ConstraintLPtoSDP& c1) const{
  return (eigVal <= c1.eigVal);	
}


//Populate
void MKC_ConstraintLPtoSDPPopulate::GetMatrixAllEigenvalue(std::vector<T_eign_Matrix> &MatrixVec, const double &tol)
{
  if (size() > 0){
    
    MatrixVec.resize(size());
    
    int count = 0;
    
    for (it=myset.begin(); it!=myset.end() ; ++it){
      if ((*it).eigVal < -tol){
	MatrixVec[count].eigVec = (*it).eigVect;
	MatrixVec[count].eigVal = (*it).eigVal; // inserting eigenvalue first position
	count++;
      }else{
	break;
      }
    }//end FOR
    MatrixVec.resize(count);
  }//end of IF 
}



bool MKC_ConstraintLPtoSDPPopulate::SetIneq(const std::vector<double> &vec,const double &vio, const bool & NEG)
{
  int count =0;
  MKC_ConstraintLPtoSDP Ineq (vec, vio, NEG);
  
  myset.insert(Ineq); // inserting 
  return true;
}

bool MKC_ConstraintLPtoSDPPopulate::SetIneq(const Eigen::VectorXd &vec, const int &size,const double &vio, const bool & NEG)
{
  int count =0;
  MKC_ConstraintLPtoSDP Ineq (vec, size, vio, NEG);
  
  myset.insert(Ineq); // inserting 
  return true;
}

bool MKC_ConstraintLPtoSDPPopulate::Getreal(const int &i)
{
  int count = 0;
   for (it=myset.begin(); it!=myset.end() ; ++it){
     if (count ==i){
       return (*it).NEG;
     }
     count++;
   }
  
  return false;
}
double MKC_ConstraintLPtoSDPPopulate::Get_eigVal(const int &i)
{
  int count = 0;
   for (it=myset.begin(); it!=myset.end() ; ++it){
     if (count ==i){
       return (*it).eigVal;
     }
     count++;
   }
   
  {
    std::cout << "Erro in GetsumTotal(const int &i) of Pop_Ineq_Wheel, access a element that out of range of this population " << std::endl;
    exit(0);
  }

  return 0.0;
}

void MKC_ConstraintLPtoSDPPopulate::Get_Vector_and_Value(const int &i, std::vector<double> &vec, double & eval)
{
  int count = 0;
   if (i < size()){
    for (it=myset.begin(); it!=myset.end() ; ++it){
      if (count ==i){
	vec = (*it).eigVect;
	eval = (*it).eigVal;
	break;
      }
      count++;
    }
  }else{
    std::cout << "Erro in GetVectorWheel(const int &i) of Pop_Ineq_Wheel, access a element that out of range of this population " << std::endl;
    exit(0);
  } 
}

std::vector<double> MKC_ConstraintLPtoSDPPopulate::GetVector(const int &i)
{
  
    int count = 0;
   if (i < size()){
    for (it=myset.begin(); it!=myset.end() ; ++it){
      if (count ==i)
	return (*it).eigVect;      
      count++;
    }
  }else{
    std::cout << "Erro in GetVectorWheel(const int &i) of Pop_Ineq_Wheel, access a element that out of range of this population " << std::endl;
    exit(0);
  }
  
  return GetVector(i);
}

void MKC_ConstraintLPtoSDPPopulate::printAll()
{
  std::cout << std::endl << "Imprimindo  elementos da  'class MKC_ConstraintLPtoSDPPopulate' :" << std::endl;
  std::cout << "seq | vec | eigenvalue"<<std::endl; 
  int count = 0;
  for (it=myset.begin(); it!=myset.end()/* && count<50 */; ++it){
    std::cout << count+1 << "| ";
    (*it).print();
    count++;
  }
}