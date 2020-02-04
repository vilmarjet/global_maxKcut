#ifndef VECPROPREINEQ
#define VECPROPREINEQ

#include "../../myFiles/eigen/Eigen/Dense"

#include <vector>
#include <set>

typedef struct T_eign_Matrix
{
  double	eigVal;
  std::vector<double>	eigVec ;
}T_eign_Matrix;

///CLASSE INEQUALTY WHEEL 
class MKC_ConstraintLPtoSDP
{
  public:
    std::vector<double> eigVect; //eigenvector
    double 		sumTotal;
    double		eigVal; // eigenvalue 
    double		right_cst;
    bool 		NEG; // if eigenvalue is negative = TRUE
    
    MKC_ConstraintLPtoSDP(){};
    MKC_ConstraintLPtoSDP(const std::vector<double> &vec, const double &, const bool &);
    MKC_ConstraintLPtoSDP(const Eigen::VectorXd &vec, const int &size, const double &, const bool &);
    bool operator< (const MKC_ConstraintLPtoSDP&) const;
    void print() const ;
};


/* cLASSE TO CREAT POPULATION OF vector prop VIOLATED */
class MKC_ConstraintLPtoSDPPopulate
{
  public:
    std::set<MKC_ConstraintLPtoSDP> 			myset;
    std::set<MKC_ConstraintLPtoSDP>::iterator 	it;
    
    
    MKC_ConstraintLPtoSDPPopulate(){}
    bool SetIneq(const std::vector<double> &vec,const double &vio, const bool &);
    bool SetIneq(const Eigen::VectorXd &vec, const int &size,const double &vio, const bool &);


    double Get_eigVal(const int &);
    std::vector<double> GetVector(const int &i);
    void Get_Vector_and_Value(const int &i, std::vector<double> &vec, double & eval);
    bool Getreal(const int &);
    bool setIneqIneq (MKC_ConstraintLPtoSDP *);
    void GetMatrixAllEigenvalue(std::vector<T_eign_Matrix> &MatrixVec, const double &tol);
    
    int  size(){return myset.size();}
    void clearAll(){myset.clear();}
    void printAll();    
};

#endif 