#include "../headers/MKC_ConstraintSelection.hpp"
#include <iostream>

MKC_ConstraintSelection::MKC_ConstraintSelection(const int & oS,const int &po,const double & g,const double & violation, const double &bb)
{
  orPos = po;
  orSep = oS; 
  gap = g; // Normalized gap = violation/b
  viol = violation;
  b = bb;
  
  if (gap < 0)
  {
    std::cout << "Erro in MKC_ConstraintSelectionPopulate: GAP from a Inequality is iferior than Zero (0)"<< std::endl;
    exit(1);
  }
}

bool MKC_ConstraintSelection::operator<(const MKC_ConstraintSelection& c1) const{
  return (gap >= c1.gap);	
}



bool MKC_ConstraintSelectionPopulate::set_newElement(MKC_ConstraintSelection elem)
{
  myset.insert(elem);
  
  if (get_CurrentSize() > sizeMax)
  {
    it = myset.end();
    --it;
    myset.erase(it); //Erase last element os Data
  }
  
  return true;
}

void  MKC_ConstraintSelectionPopulate::set_MaxSize( const int &s)
{
  sizeMax = s;
}

int  MKC_ConstraintSelectionPopulate::get_CurrentSize()
{
  return myset.size();
}

void MKC_ConstraintSelectionPopulate::clearAll()
{
  myset.clear();
  sizeMax = 0;
}

void MKC_ConstraintSelectionPopulate::print_top(const int & qt)
{
  	
	std::cout<< "Print elements of classe MKC_ConstraintSelectionPopulate, ";
	std::cout<< "myset.size() = "<< myset.size()<< std::endl;
	std::cout<< "Orig. Sep | Orig. Pos | gap | viol | b  "<< std::endl;
	
	int cont = 0;
   //for (it=myset.begin(); it!=myset.end(); ++it)
   //for (it=myset.end(); it!=myset.begin(); --it)
   for (it=myset.begin(); it!=myset.end(); ++it){
     std::cout << (*it).orSep << " | " << (*it).orPos << " | "<< (*it).gap<<  " | "  << (*it).viol<< " | " << (*it).b  <<std::endl;
     if (cont == qt )
       break;
     cont ++;
   }
}

void MKC_ConstraintSelectionPopulate::print_all()
{	
	std::cout<< "Print elements of classe MKC_ConstraintSelectionPopulate, ";
	std::cout<< "myset.size() = "<< myset.size()<< std::endl;
	std::cout<< "Orig. Sep | Orig. Pos | gap | viol | b  "<< std::endl;
	
   //for (it=myset.begin(); it!=myset.end(); ++it)
   //for (it=myset.end(); it!=myset.begin(); --it)
   for (it=myset.begin(); it!=myset.end(); ++it)
     std::cout << (*it).orSep << " | " << (*it).orPos << " | "<< (*it).gap<<  " | "  << (*it).viol<< " | " << (*it).b  <<std::endl;
}

void MKC_ConstraintSelectionPopulate::set_IneqData_in_SepPop(const int &orS,const int &orP, const double &violation, const double & bb)
{
  
  double aux_b = bb;
  
  if (bb <0)
    aux_b = aux_b*(-1.0);
  
  double gap = violation/aux_b;
  MKC_ConstraintSelection data(orS, orP,gap, violation, bb);
  
  myset.insert(data);
  
}


MKC_ConstraintSelection MKC_ConstraintSelectionPopulate::get_Data(const int & ii)
{
   it = myset.begin();
    for (int i =0; i<ii; i++)
	++it;                  
  
  return (*it);

}
