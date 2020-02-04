#ifndef POPPOP3
#define POPPOP3

#include <vector>
#include <set>


class MKC_ConstraintSelection
{
  public:
    int	 	orSep,orPos; // Original Separation(Inequality) and Original position 
    double 	gap; // Normalized gap = violation/b
    double	viol, b; //Violationa
    
    MKC_ConstraintSelection(const int &,const int &,const double &,const double &,const double &);
    bool operator< (const MKC_ConstraintSelection&) const;
};


/* cLASSE TO CREAT POPULATION OF */

class MKC_ConstraintSelectionPopulate
{
  public:
    std::set<MKC_ConstraintSelection> 		myset;
    std::set<MKC_ConstraintSelection>::iterator 	it;
    int					sizeMax;
    
    
    MKC_ConstraintSelectionPopulate(){}
    bool  set_newElement(MKC_ConstraintSelection);
    void  set_MaxSize(const int &);
    int   get_CurrentSize();
    void  clearAll();
    void  print_all();
    void print_top(const int &);
    int	  size(){return myset.size();}
    void  set_IneqData_in_SepPop(const int &orS,const int &orP, const double &violation, const double & bb);
    
    MKC_ConstraintSelection	get_Data(const int &);
    
    
    MKC_ConstraintSelectionPopulate operator=(const MKC_ConstraintSelectionPopulate& b) {
         MKC_ConstraintSelectionPopulate box;
         box.myset = b.myset;
	 box.sizeMax = b.sizeMax;
	 return box;
      }
};


#endif 