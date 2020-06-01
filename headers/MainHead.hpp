#ifndef HEADHEAD
#define HEADHEAD



#include "mosek.h"    /* Include the MOSEK definition file.  */


#include <iostream>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <cstdlib>
#include <ctime>
#include <vector>
#include <set>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <string>	//string
#include <iomanip>      // std::setprecision
#include <math.h> // use pow and sqrt 



#include "MKC_ConstraintTriangle.hpp"
#include "MKC_ConstraintClique.hpp"
#include "MKC_ConstraintSelection.hpp"
#include "MKC_ConstraintWheel.hpp"
#include "MKC_ConstraintLPtoSDP.hpp"
#include "MKC_InstanceRankVertices.hpp"

#include "MKCGraph.hpp"
#include "MKCGraphBuilder.hpp"
#include "MKCInstance.hpp"
#include "MKCInstanceBuilder.hpp"

#include "MKC_ModelEdgeLP.hpp"
#include "MKC_ModelEdgeSDP.hpp"

#include "VariablesEdge.hpp"

#include "./CPA/CuttingPlaneAlgo.hpp"

#include "./Solver/Abstract/Solver.hpp"
#include "./Solver/Mosek/SolverMosekLp.hpp"
#include "./Solver/Factory/SolverFactory.hpp"

#include "MKC_CuttingPlane.hpp"

#include "../../../myFiles/eigen/Eigen/Dense"
#include "../../../myFiles/eigen/Eigen/Eigenvalues" 


#include "./Parameters/MKC_CPAParam.hpp"
#include "./Parameters/BranchBoundParam.hpp"
#include "./Parameters/MKC_ProblemParam.hpp"
#include "./Parameters/MKC_HeuristicParam.hpp"

#include "./Utils/UtilsFile.hpp"

//#include <ilcplex/ilocplex.h>


typedef struct T_cost_Matrix
{
  std::vector<int>      barc_i ;
  std::vector<int>	   barc_j ;
  std::vector<double>   barc_v ;
  
  
  inline T_cost_Matrix operator=(T_cost_Matrix a) {
        barc_i = a.barc_i;
        barc_j = a.barc_j;
	barc_v = a.barc_v;
	
        return a;
    }

    
}T_cost_Matrix;


typedef struct idx_ij{
  int idx_i;
  int idx_j;
}idx_ij;

typedef struct T_constraint
{
  int			Origem; //1 = Triangle, 2=Clique ....
  double		blc; // Numerical value of Lowwer bound 
  double		buc; // Numerical value of Upper bound
  MSKboundkeye		bkc; // Bound key
  
  std::vector<int> 	vars ; // vector of non-zero variables in the constraint (for LP)

  std::vector<double>	aval ; // value 
  
  
  inline T_constraint operator=(T_constraint a) {
        Origem = a.Origem;
        blc = a.blc;
	buc = a.buc;
	bkc = a.bkc;
	
	vars = a.vars;
	aval = a.aval;
	
        return a;
    }
  
}T_constraint;


typedef struct T_Instance
{
  //string 	nom;
  int    		DIM ;		/*Dimention = number of vertices*/
  long     		LENBARVAR;	/* Number of scalar SD variables  */
  T_cost_Matrix		cij;
  
  std::vector<T_constraint> CONST;     // set of inequalities 
  long 			    lastConst;
  
  //T_bounds_Cons	bound;
  double		sum_cost;	/*Constant that put in the cost matrix x(00)*/
  double		cst;		/*  =  valeur de (-(k-1)/k) from F.O*/
  long			edge_nb;
  long 		totalVars;
  
  std::vector<int>	indice_cij; 	//indice of where we can find the cost 
  std::vector<idx_ij>	ant_indice_cij; // variables i and j of indice_cij 	
  std::vector<double>	varS;
  double		ObSol;
  double 		valFeasibleSol;
  
  
  
  inline T_Instance operator=(T_Instance a) {
        DIM = a.DIM;
        LENBARVAR = a.LENBARVAR;
	cij = a.cij;
  
	CONST = a.CONST;     // set of inequalities 
  
  //T_bounds_Cons	bound;
	sum_cost = a.sum_cost;	/*Constant that put in the cost matrix x(00)*/
	cst = a.cst;		/*  =  valeur de (-(k-1)/k) from F.O*/
	edge_nb = a.edge_nb;
	totalVars = a.totalVars;
  
	indice_cij = a.indice_cij; 	//indice of where we can find the cost 
	ant_indice_cij = a.ant_indice_cij; // variables i and j of indice_cij 	
  
	varS = a.varS;
	ObSol = a.ObSol;
	valFeasibleSol = a.valFeasibleSol;
	
        return a;
    }
    
    
    void clear_Constraint(){
      CONST.clear();
    }
    
    
}T_Instance;

typedef struct T_fixVar
{
  long var;
  double val;

  inline T_fixVar operator=(T_fixVar a) {
	var = a.var;
	val = a.val;
  }

  bool operator<(const T_fixVar& a) const{
      return (val < a.val); //Attention, some variables can not be considered in "set",  since it is just (<) ... if want all change to (<=))	
    }

}T_fixVar;

typedef struct T_Branch
{
  double	lowerboud; // father lb
  double	upperbound;// father ub 
  double      	seq;  //to rank in the select strategy 
  int      	Level_tree;
  long 		lastVarFix;
  long 		lastPartFix;  
  
  std::vector <T_fixVar> fixvar;	//vector with fixed edges (dictomic bb)
  std::vector< std::vector <int> > Partitions; // fixed partitions
  std::vector<T_constraint> ExtraCONST;     // set of inequalities 
  
  
  T_Instance   *ptxInst;
  


   bool operator<(const T_Branch& a) const{
      return (seq <= a.seq);	
    }
    
  inline T_Branch operator=(T_Branch a) {
        lowerboud = a.lowerboud;
        upperbound = a.upperbound;
	seq = a.seq;
	Level_tree = a.Level_tree;
  	lastVarFix = a.lastVarFix;
  	lastPartFix =a.lastPartFix;  

  
	Partitions = a.Partitions;     // partition 
	fixvar = a.fixvar;	//vector with fixed edges (dictomic bb)
	ExtraCONST = a.ExtraCONST;

	ptxInst = a.ptxInst;		/*  =  valeur de (-(k-1)/k) from F.O*/
	
        return a;
    }
    
}T_Branch;

typedef struct T_PseudCost_BB
{
 std::vector <double> vec_cost;//sum of all gain when do decision k
 std::vector <int> vec_nbAces; // number of times that this decision is taken into acont 
 
 void Set_NbPart(const int &nb){
   vec_cost.resize(nb,0.0);
   vec_nbAces.resize(nb,0);
 }
 void SetGain(const double &gain, const int &part){
	vec_cost[part] += gain;
	vec_nbAces[part] ++;
 }
 int MinNbAccess () const {
	int min= 100000;
	for (int i=0; i<vec_nbAces.size(); ++i)
		if (vec_nbAces[i] < min)
			min = vec_nbAces[i] ;
	return min;
  } 

 double Max_avg () const {
   double val= 0.0;
   for (int i=0; i<vec_cost.size(); ++i){
	if (vec_nbAces[i] > 0){
		if (val < ((double)(vec_cost[i])/((double) vec_nbAces[i])) )
			val = ((double)(vec_cost[i])/((double) vec_nbAces[i]));
	}
   }//end for 
   return val;
 }

 double Min_avg () const {
   double val= 100000000.0;
   bool flag = false;
   for (int i=0; i<vec_cost.size(); ++i){
	if (vec_nbAces[i] > 0){
		if (val >((double)(vec_cost[i])/((double) vec_nbAces[i]))){
			val = ((double)(vec_cost[i])/((double) vec_nbAces[i]));
			flag = true;
		}
	}
   }//end for 
   if (flag)
  	return val;
   else 
	return 0.0;
 }

 double Get_Score(const double &Mu) const{
	return ((1.0 - Mu)*Min_avg() + Mu*Max_avg());
 }


}T_PseudCost_BB;



typedef struct T_SDP_EdgeConstraint
{
  int size;
  int newadd; //new ineq added last iteration
  
  std::vector<int>	varI; 	//var i
  std::vector<int>	varJ; 	//var j

  void clear(){
	varI.clear();	
	varJ.clear();
	newadd = 0;
	size = 0;
  }
  
  
  inline T_SDP_EdgeConstraint operator=(T_SDP_EdgeConstraint a) {
        size = a.size;
        newadd = a.newadd;
	
	varI = a.varI;
	varJ = a.varJ;
        return a;
    }
    
}T_SDP_EdgeConstraint;



typedef struct RankPartition
{
    int 	Part_origem;
    double 	sumEdge;
    
    bool operator<(const RankPartition& a) const{
      return (sumEdge <= a.sumEdge);	
    }
}RankPartition;


typedef struct ICH_Var
{
    std::vector<unsigned>	vertices; 	//var i
    double 	sumEdge;
    
    bool operator<(const ICH_Var& a) const{
      return (sumEdge >= a.sumEdge);	
    }
}ICH_Var;



void MakeSparseEig (T_Instance &instance, std::vector<double> &eigvect, const double &eigval );
void MakeSparseEig (T_Instance &instance, std::vector<double> &eigvect, const double &eigval, const int &sizeSetZero );
double  CalculateViolation_SDP (T_Instance &instance, std::vector<double> &vecProp);


// Printing 
inline void PrintSDPtypeSolution (const T_Instance& instance );
template <typename T>
inline void printvector(const std::vector<T> &v) ;
template <typename T>
inline void printMatrix( const std::vector<T> &m,const int &dim);
template <typename T>
inline void printDoubleVector( const std::vector<std::vector<T> > &m);

void WriteFinalResult_File (char *fileResult,const T_Instance & instance,clock_t &time,const int ite);
void WriteFinalResult_File (const std::string &fileResult,const T_Instance &instance, clock_t &time, const int ite);
inline void print_Inequality_Tconstraint(const T_constraint &Const);
inline void  print_Inequality_TconstraintIndex(const T_constraint &Const, const T_Instance & instance);
inline void WriteIteration_FILE(const T_Instance &instance, std::ofstream &file_ITE, const char*FileLecture, const int &i);
inline void WriteIteration_FILE(const T_Instance &instance, std::ofstream &file_ITE, const std::string FileLecture, const int &i);
inline void Print_Iterations_screen(const T_Instance &instance, const char*FileLecture, const int &i);
inline void Print_Iterations_screen(const T_Instance &instance, const std::string &FileLecture, const int &i);
inline void printSolution_screen(const T_Instance &instance);
inline void printCostMatrix_CompleteGraph (const T_Instance& instance);
inline void printCostMatrix_SparseGraph (const T_Instance& instance);
inline void PrintPartitionICH(const std::vector< std::vector<int> > &Partitions);
inline void Print_ListBB_allActive (const std::set <T_Branch> &ListBB);
inline void PrintScreen_BB(const int &Ite, const int &Size_list, const double &Lb, const double &Ub, const double &gap, const double &time);


//Set Parameters 
void set_FileNames(char *argv[],std::string &FileResults);
void set_FileNamesITE(char *argv[],std::string &FileResults);
void Inserting_Parameters(char *argv[]);

inline void clear_AllPopSeparation();


//General use 
//void Lire_Instance(char* nomFichier,T_Instance& instance);
void Lire_Instance(const std::string nomFichier,T_Instance& instance);
inline bool FindViolation(T_Instance &instance);
inline double ValueXij (const int &i, const int &j,const T_Instance &instance);
inline void Selective_Separation(T_Instance &instance);
inline double sampleNormal() ;
inline double getCurrentTime_Double(clock_t &time);
inline void DynamicIneqSize(const T_Instance &instance, const double &gapImp, int *NBMAX );
double CuttingPlane_Optimization (T_Instance &instance);
//double CuttingPlane_Optimization (T_Instance &instance, const bool &PRINT_ITERATIONS);
double CuttingPlane_Optimization (T_Instance &instance, const bool &PRINT_ITERATIONS, clock_t  Start = std::clock());
char* Get_char_from_String ( const std::string &stri);
inline void setInitial_LP_Solution_justVar(T_Instance &instance);
double UpperBoundSmallestEigenvalue(T_Instance& instance);
double UpperBoundSmallestEigenvalue_UnweightedGraph(T_Instance& instance);
void Set_IneqNuberOfEdgesCut (T_Instance &instance, const double &rghtValue, const int &qtEdges);



//Graph simplification
inline void K_coreElimination(T_Instance& instance);
void Block_Optimization(T_Instance &instance);
inline void Chordal_extention (T_Instance& instance);
inline void Set_Neighbourvertices(const T_Instance& instance, const int &vertex, std::vector <bool>  select_v, std::vector <int> *neighb_vertice );
inline int  MinimumDegree (const T_Instance& instance,   std::vector <int>  deegre_vertice,  std::vector <bool>  select_v );
inline bool ExistEdge (const T_Instance&instance, const int &n_j, const int &n_i );
inline bool Set_Clque_in_MaximalClique (const int &vertex, std::vector <int>neighb_vertice, std::vector <std::vector <int> > *maximalCliques);
inline void Add_edgeInstance (T_Instance& instance, const int & J, const int & I);


//Heuristic
void DoAllHeuristicFiles(char *argv[], T_Instance &instance);
double execute_heuristic_to_feasible_solution(const T_Instance &instance);


//MOSEK
// inline void setObjFunction_andVarBounds__LPmosek(const T_Instance &instance, MSKrescodee  &r, MSKtask_t    &task, MSKenv_t &env);
inline void setObjFunction_andVarBounds__LPmosek(const T_Instance &instance, MSKrescodee  &r, MSKtask_t    &task, MSKenv_t &env, int Form_Type=0);;
inline void setConstraint_LPmosek(T_Instance &instance, MSKrescodee  &r, MSKtask_t    &task, int start=0);
inline void Add_NewConstraintLP_in_taskMosek(T_Instance &instance, MSKrescodee  &r, MSKtask_t    &task);
//inline void SolveMosek(T_Instance &instance, MSKrescodee  &r, MSKtask_t    &task, MSKenv_t &env);
inline void SolveMosek(T_Instance &instance, MSKrescodee  &r, MSKtask_t    &task, MSKenv_t &env, int type=0);
inline void SolveMosek2(T_Instance &instance, MSKrescodee  &r, MSKtask_t    &task, MSKenv_t &env);
inline void Set_REDUNDANT_Ineq_of_FuncObj(T_Instance &instance, MSKrescodee  &r, MSKtask_t    &task);
inline void setSolver(T_Instance &instance, MSKrescodee  &r, MSKtask_t    &task, MSKenv_t &env, const int & Option);
inline  double setInitial_LP_Solution(T_Instance &instance);



inline void DynamicIneqActivation(const T_Instance &instance, const double &gapImp);
inline void ActiveAllIneq();
inline void ActiveJustTriWhell();
inline void DesactivateAll();

// ********************
//		Triangle Inequality
// **************
void TriangleInequalitySelective( T_Instance &instance, int &NbIneqMax, const int &type, MKC_ConstraintTrianglePopulate &PopTri);
void DesignTriangleIneq_Mosek ( T_Instance &instance,const MKC_ConstraintTriangle &IneqT, const double &cstT);
inline bool  get_exceptionsVector_vertex(const T_Instance &instance, std::vector<int> &vec_vertices, const int &h);
void AddAll_TriangleInequality( T_Instance &instance,const int &NbIneqMax, const int &type, MKC_ConstraintTrianglePopulate &PopTri);

// ********************
//		Clique Inequality
// **************
void Clique_InequalitySelective_Heuristic ( T_Instance &instance, int &NbIneqMax, const int &SizeClique, const double &cst, const int &type, MKC_ConstraintCliquePopulate &Population );
void DesignCliqueIneq_MOSEK ( T_Instance &instance,const std::vector<int>  &vec_vertices, const double &cstCliq);
inline double Evaluation_Clique(const T_Instance &instance, const std::vector<int> &vec_vertices );
void addFbdnEdges(const T_Instance &instance, std::vector<int> &vec_vertices,std::set<int> &Fbd_edges);
void LocalSearch_CLIQUE(const T_Instance &instance, std::vector<int> &vec_vertices, const int &Size_Clique, const std::set<int> &fbdn_edges  );
void Clique_Heuristique_Selective2 (const T_Instance &instance, std::vector<int> &vec_vertices, const int &Size_Clique, const std::set<int> &fbdn_edges );
inline bool  get_exceptionsClique(const T_Instance &instance, std::vector<int> &vec_vertices, std::set<int> &frbdn_vertex,const int &h, const std::set<int> &fbdn_edges);



// ********************
//		General clique Inequality
// **************
void GenClique_Inequality( T_Instance &instance, int &NbIneqMax, const int &type, MKC_ConstraintCliquePopulate &PopGen);


// ********************
//		Wheel Inequality
// **************
void Wheel_InequalitySelective_Heuristics( T_Instance &instance,int &NbIneqMax,  const int &type, MKC_ConstraintWheelPopulate &PopWh);
inline bool  get_exceptionsWheel(const T_Instance &instance, std::vector<int> &vec_vertices, std::set<int> &frbdn_vertex,const int &typeWheel ,
				 const int &h, const int & PosInVec);
inline bool validationWheel(const T_Instance &instance, std::vector<int> &vec_vertices, const int &typeWheel);
inline double Evaluation_Wheel (const T_Instance &instance, std::vector<int> &vec_vertices,const int &typeWheel);
void DesignWheelIneq_MOSEK( T_Instance &instance,const std::vector<int> &vec_vertices, const double &cstW,  const int &typeWheel);

//grasp
void  GRASP_Wheel(const T_Instance &instance, std::vector<int> &vec_vertices,const int &typeWheel, const int &SizeCycleInWheel,
		  MKC_ConstraintWheelPopulate & PopWh,const double &rh_cst, const int & NbItGrasp );
bool Construction_GRASP_Wheel (const T_Instance &instance, std::vector<int> &vec_vertices,const int &typeWheel, const int &SizeCycleInWheel);
double LocalSearch_Wheel(const T_Instance &instance, std::vector<int> &vec_vertices,const int &typeWheel);

//genetic
void  GENETIC_Wheel(const T_Instance &instance, std::vector<int> &vec_vertices,const int &typeWheel, const int &SizeCycleInWheel, MKC_ConstraintWheelPopulate & PopWh);
void Construction_FirstPopulation_Genetic_Wheel(const T_Instance &instance, std::vector<int> &vec_vertices,const int &typeWheel, const int &SizeCycleInWheel,
					      MKC_ConstraintWheelPopulate & PopWh,const double &rh_cst, const int & SizePop );
inline void CreatRandomWheelIneq(const T_Instance &instance, std::vector<int> &vec_vertices,const int &typeWheel, const int &SizeCycleInWheel,
					      MKC_ConstraintWheelPopulate & PopWh,const double &rh_cst, const int & SizePop );
void CrossFitting_Wheel(const T_Instance &instance,const int &typeWheel, const int &SizeCycleInWheel, MKC_ConstraintWheelPopulate & PopWh,const double &rh_cst, const int & SizePop );
inline void SelectMama_and_Papa(std::vector<int> &VecPapa, std::vector<int>&VecMama, MKC_ConstraintWheelPopulate &PopWh,
			  const double &minPapa, const double &maxPapa,
			  const double &minMama, const double &maxMama,const int & SizePop);
inline void GeneticALgo_MutationInWheel(const T_Instance &instance, std::vector<int> &vec_vertices,const int &typeWheel, const int &SizeCycleInWheel);
void Genetic_LocalSearch_Wheel(const T_Instance &instance,const int &typeWheel, const int &SizeCycleInWheel,
			       MKC_ConstraintWheelPopulate & PopWh,const double &rh_cst, const int & SizePop );

//multiple size
bool Wheel_Heuristic_MultipleSize (const T_Instance &instance, std::vector<int> &vec_vertices,const int &typeWheel, MKC_ConstraintWheelPopulate & PopWh );



// ********************
//		Bicycle wheel Inequality
// **************
void Bi_Wheel_InequalitySelective(T_Instance &instance,int &NbIneqMax, const int &type, MKC_ConstraintWheelPopulate &PopWh);








/*******
 * 		Eigenvalue constraint
 */
inline void Find_edgeViolated_in_SDP (const T_Instance &instance, double* barx);
inline void Clear_edgeNotViolated_in_SDP (const T_Instance &instance);
void SDP_violations_in_LPSOlution(T_Instance &instance, int &NbIneqMax,MKC_ConstraintLPtoSDPPopulate &PopSDP, const int & type);
void FindNegatifEigenvaleu(T_Instance &instance,MKC_ConstraintLPtoSDPPopulate &PopVecProp);
void FindNeg_EigFromSDP(double *barx,  T_Instance &instance);
void Design_SDPIneq(T_Instance &instance,const std::vector<double> &vecProp);
inline void BuildCompleteGraph(T_Instance& instance);
void Heuristic_to_ValidEigenvalues(T_Instance &instance,MKC_ConstraintLPtoSDPPopulate &PopVecProp);
void Method_to_ValidEigenvalues_Miguel(T_Instance &instance,MKC_ConstraintLPtoSDPPopulate &PopVecProp);
void set_pre_inequalities_VP (T_Instance &instance);
void set_pre_inequalities_VP2 (T_Instance &instance);
void set_pre_inequalities_VP3(T_Instance &instance);
void Eigenvaleu_ForSomeVertices(T_Instance &instance,MKC_ConstraintLPtoSDPPopulate &PopVecProp, const  std::vector<int> &Vec_Vertices, const double &counterMaxNeg );
void FindNegatifEigenvaleu_Chordal(T_Instance &instance,MKC_ConstraintLPtoSDPPopulate &PopVecProp);

void FindGapIneqNegatifEigenvalue(T_Instance &instance, MKC_ConstraintLPtoSDPPopulate &PopVecProp);



void Design_SDPIneq2( T_Instance &instance,const std::vector<double> &vecProp);

void Set_IneqFuncObj (T_Instance &instance, const double &rghtValue);
void Method_to_ValidEigenvalues_Vilmar(T_Instance &instance,MKC_ConstraintLPtoSDPPopulate &PopVecProp);

void FindNegatifEigenvaleuSubGraph_triangle(T_Instance &instance,MKC_ConstraintLPtoSDPPopulate &PopVecProp);
void Design_SDPIneq_Triangle( T_Instance &instance,Eigen::VectorXcd &vecProp,  const std::vector<int> &vertex);

void FindNegatifEigenvaleuSubGraph(T_Instance &instance,MKC_ConstraintLPtoSDPPopulate &PopVecProp);
void Design_SDPIneq_SubGraph ( T_Instance &instance,Eigen::VectorXcd &vecProp,  const int begin, const int endd);


void Gershgorin_Circle(T_Instance &instance,MKC_ConstraintLPtoSDPPopulate &PopVecProp);

//New ideas insert sdp in LP 
void FindNegatifEigenvaleu_Constraint_WithPositiveEig(T_Instance &instance,MKC_ConstraintLPtoSDPPopulate &PopVecProp);



//Pre treatement subgraph
inline void SetPretreatmen(T_Instance & instance);
inline void Optmize_subGraph(T_Instance & instance);
inline void Copy_instance(const T_Instance &instance, T_Instance &destination, const int &size_max, const int &start, std::vector<int> &origen_indice);
void PreTrea_Div2Conq(T_Instance &instance);
inline void ClearInstance(T_Instance & instance);
void Set_validIneq_in_MainInstance(T_Instance & destination, T_Instance & source,  std::vector<int> &origen_indice);
inline void copy_constraint_i_FromsubGraph(T_Instance &destination, const T_Instance & source, const int &idx, std::vector<int> &origen_indice);
void Clean_uselessInequalities (T_Instance & instance);
void Rank_by_degree(std::set<MKC_InstanceRankVertices> &ranked, const T_Instance &instance );
inline void Rank_by_incidentWeight(std::set<MKC_InstanceRankVertices> &ranked, const T_Instance &instance );
inline void Copy_by_rank (const T_Instance &instance, T_Instance &destination, const int &size_max, const int &start, 
                          std::vector<int> &origen_indice, const std::vector<int> &ranked_v);

inline void Optmize_subGraph_onlyEig(T_Instance & instance);

void PreTrea_Div2Conq_Eigenvalue(T_Instance &instance);
inline void Set_validIneq_in_MainInstance_justEig (T_Instance & destination, const T_Instance & source,  std::vector<int> &origen_indice);
void PreTrea_Div2Conq_rank_exploreAll(T_Instance &instance);
inline void Copy_by_two_SubGraphs(const T_Instance &instance, T_Instance &destination,
				 const std::vector<int> &begin_v, const std::vector<int> &end_v, const int & frst, const int & scond, 
				  std::vector<int> &origen_indice);
inline void select_vertices_fromMOstviolatedIneq(const T_Instance &instance, std::vector<int> &ranked_v, const int & max_rand, const int & qt_Ineq, const int & max_size,
						  MKC_ConstraintSelectionPopulate &PopBestIneq_origem, MKC_ConstraintTrianglePopulate 	&PopTriangle_origem, MKC_ConstraintCliquePopulate &PopClique_origem ,MKC_ConstraintCliquePopulate &PopGenClique_origem,
						  MKC_ConstraintWheelPopulate &PopWheell_origem, MKC_ConstraintWheelPopulate &PopBiWheell_origem);
void SubGraphTreat_by_ViolatedIneq(T_Instance &instance);
inline bool Find_ViolatedTri_in_wheel(const T_Instance & instance, const int &h,const int & j, const int & i);
inline double PreTrea_Div2Conq_forUpperbound(T_Instance & instance);






// SDP
void SolveMosekSDP (T_Instance &instance);
inline void Transforme_LpIneq01_in_SDPineq(const int &nb,const double &LBsdp, const double &UBsdp, double *vl, double &bound );
inline double getValueXij_SDP ( const int &i, const int & j,  double* barx, const int &DIM );
inline void Transform_SDPsol_2_LPsol(T_Instance &instance, double*barx, const double &LBsdp, const double &UBsdp);
inline double ManualCalculeOfObjFunc(T_Instance &instance);
inline void Set_ObFunction_Mosek_SDP(const T_Instance &instance,MSKrescodee &r, MSKtask_t &task, const double &LBsdp);
inline void Set_CombinatorialConstraints_MOSEK_SDP(const T_Instance &instance,MSKrescodee &r, MSKtask_t &task, const double &LBsdp, const double &UBsdp);
inline void Set_OriginalConstraints_MOSEK_SDP(const T_Instance &instance,MSKrescodee &r, MSKtask_t &task, const double &LBsdp);
void Design_SDPIneq3( T_Instance &instance,const std::vector<double> &vecProp);
inline bool Find_ViolatedTri_in_eigenvalue(const T_Instance &instance, std::vector<int> &aux_vec,const int  &counter,const int  &h );


//CPLEX
// void solveCPLEX_LP(T_Instance &instance);
// inline void setObjFunction_cplex(IloNumVarArray &vars, T_Instance &instance, IloModel &model, IloEnv &env);
// inline void setVars_cplex(IloNumVarArray &vars, T_Instance &instance, IloEnv &env);
// inline void setParam_cplex(IloCplex &cplex, std::stringstream &logfile);
// void setConstraints_cplex(IloNumVarArray &vars, T_Instance &instance, IloModel &model, IloEnv &env);


//Representative formulation
inline void Set_RepresentativeVar(T_Instance& instance);
void Design_Representative_OnePerCluster(T_Instance &instance);
void Design_RepresentativeIneq_MaxOnePerCluster(T_Instance &instance);
void Design_RepresentativeIneq_SumRepresentative(T_Instance &instance);

//Extended representative 
inline void Set_ExtendedRepresentativeVar(T_Instance& instance);
void Design_ExtendLimitEdge(T_Instance &instance);
void Design_ExtendLimitRepresentative(T_Instance &instance);
void Design_ExtendWithTwoVars(T_Instance &instance);
void Design_ExtendEqualIneq(T_Instance &instance);


//Node and Edge formulation
inline void set_NodeEdge_Formulation(T_Instance&  instance);
inline void Design_SumNodes_equal_One(T_Instance &instance);
void Design_Impose_EdgeEqualOne(T_Instance &instance) ;//Ineq 2.7 in thesis
void Design_Impose_EdgeCut_if_DifferentPartition(T_Instance &instance); //Ineq 2.8 and 2.9 together in thesis



//New formulation
void set_newFormulation(T_Instance &instance);
inline double calcule_IncidenteEdges(const T_Instance &instance, std::vector<double> *posM, 	std::vector<double> *negM, const std::vector<int>  &ind_cij, const T_cost_Matrix &costMatrix);
void Design_NewFormulation_bigIneq (T_Instance *instance, const T_cost_Matrix &costMatrix, const std::vector<int> &ind_cij, const std::vector<double> &negM);
void Design_NewFormulation_upperboundpartition(T_Instance *instance, const std::vector<double> &posM);
void Design_NewFormulation_sumEqualOne(T_Instance *instance);


//Heuristics solution
inline double Get_valueOfPartition (const T_Instance &instance,const std::vector< std::vector<int> > &Partitions);
inline void GenerateRandomSolution (const T_Instance &instance, std::vector< std::vector<int> > &Partitions);
void LocalSearch_Solution (const T_Instance &instance,std::vector< std::vector<int> > &Partitions);
inline double AnalyzeNewPartition(const T_Instance &instance, const std::vector< std::vector<int> > &Partitions, std::vector< std::vector<int> > &BestPartitions, double *BestPartitionsValue );
inline int  TotalNbVertices_Partition (const std::vector< std::vector<int> > &Partitions);
double gain_movePartition(const T_Instance &instance,  const std::vector< std::vector<int> > &Partitions, const int &vert, const int &Par_or, const int &Par_target);


//ICH
double ICH_Heuristique(const T_Instance &instance, const  double &epslonICH = 0.4 );
double ICH_Heuristique_Ghaddar(const T_Instance &instance, const  double &epslonICH  = 2.6);
double ICH_Heuristique_Ghaddar_main(const T_Instance &instance);
void set_newInstance_forICH (T_Instance *instance, std::vector<std::vector<int> > Partitions);
double ICH_Heuristique_Ghaddar(const T_Instance &instance,  std::vector<int> &OriginVertices , const T_Instance &instanceCONST, const  double &epslonICH = 2.6);
void Change_VecVertexOrigem_by_Partition_ICH ( const std::vector< std::vector<int> > &Partitions, std::vector<int> &OriginVertices);
void Change_Partition_by_VecVertexOrigem_ICH (const std::vector<int> &OriginVertices, std::vector< std::vector<int> > &Partitions);


//Grasp
double Grasp_Heuristic_FeasibleSoltion (const T_Instance &instance);
void Initial_solution_to_GraspFeasible (const T_Instance &instance,const int &type_val, const double &RCL_prop, std::vector< std::vector<int> > &Partitions);


//Tabu
double TabuOptimum_Heuristic_FeasibleSoltion (const T_Instance &instance);
bool Tabu_ModifyOneVertex_best(const T_Instance &instance, std::vector< std::vector<int> > &Partitions, std::vector<int> &TabuList, const int &TabuListSize );
inline void ChangeTabu_decrease(std::vector<int> & tabuList);


//VNS
double VNS_Heuristic_FeasibleSoltion (const T_Instance &instance);


//Hybrid
double  Hybrid_GraspAndTabu(const T_Instance &instance);

//MOH
double MultipleSearch_Heuristic_FeasibleSoltion (const T_Instance &instance);
double Set_gama (const int &Ori_p_i ,const int &target_i, const int &Ori_p_j ,const int &target_j  );
void LocalSearch_TwoMoves_Solution  (const T_Instance &instance,std::vector< std::vector<int> > &Partitions);


///////
// BRANCH AND BOUND
///////// 
double BranchAndBound(T_Instance &instance);
void Inserting_Parameters_BB(char *argv[]);
bool CuttingPlane_Simple_for_BB (T_Instance &instance_orig,   std::vector< std::vector<int> > *Partitions,   std::vector <T_fixVar> &fixvar, 
const bool  &BY_PARTITION_BB, double  *bestLowerBound ,const int &cleanIneq, const std::vector<T_constraint> &CONST_branch  );
double Calculate_BoundNikiforov(const T_Instance &instance, const std::vector< std::vector<int> > &Partitions);
inline bool Solve_SubProblem_BB (T_Instance &instance,   std::vector< std::vector<int> > *Partitions,   std::vector <T_fixVar> &fixvar,
  const bool  &BY_PARTITION_BB,  const int &TYPE_SOLVER_BB,  const bool &NEW_TASK, double *bestLowerBound,const  bool  &sdp_edge_type=false);
inline void WriteIteration_FILE_BB(std::ofstream &file_ITE, const int &Ite, const int &Size_list, const double &Lb, 
	const double &Ub, const double &gap, const double &time);
void SaveFinalSolution(const int &Ite, const double &Lb, const double &Ub, const double &gap, const double &time);
void set_FileNamesITE_BB (std::string &FileResults);
inline bool Branching_BB  (std::set <T_Branch> &ListBB, T_Instance &instance,   std::vector< std::vector<int> > &Partitions,   std::vector <T_fixVar> &fixvar,
  const bool  &BY_PARTITION_BB,  const int &TYPE_SOLVER_BB, double *bestLowerBound);
inline bool Strategy_selecting_ActiveNodeTree_BB(const T_Instance &instance, double *val_Change);
inline bool  CheckAndClean_ListBB (std::set <T_Branch> &ListBB, const double &bestLowerBound, double *biggestUpper ) ;
void  Clean_uselessInequalities_BB (const T_Instance &instance, std::vector<T_constraint> &CONST_BB);

bool SolveMosek_LP_for_Branch(T_Instance &instance, MSKrescodee  &r, MSKtask_t    &task, MSKenv_t &env, const int TYPE, const std::vector<T_fixVar> &fixvar, const bool &NEW_TASK);
inline long ChooseEdge_BB (const T_Instance &instance);
long ChooseEdge_PseudoCost_BB( T_Instance &instance, std::vector<T_fixVar> *fixvar ,std::vector<T_PseudCost_BB > *PseudoCcost );
long FindBestEdge_score ( std::set<int>UsedEdges,const std::vector<T_PseudCost_BB > &PseudoCcost , const double &Mu);
inline long ChooseEdge_ConsiderPartition_BB(const T_Instance &instance, const std::vector< std::vector<int> > &Partitions);
inline void Set_BranchingRule(const T_Instance &instance, const bool &BY_PARTITION_BB);
inline void Set_NewValPseudoCost (const double  &Ub_before,  const double &UB_new, const int &var,const int &part , std::vector<T_PseudCost_BB > *PseudoCcost );
int FindBestVertex_score (const std::vector<bool> &validVertex,const std::vector<T_PseudCost_BB > &PseudoCcost , const double &Mu);
inline int ChooseVertexPseudoCost_by_Partition_BB(T_Instance &instance_orig,const std::vector< std::vector<int> > &Partitions_origem,
std::vector<T_PseudCost_BB > *PseudoCcost );

inline long Branch_Rule_ChooseVariable(T_Instance &instance,std::vector<T_fixVar> *fixvar,  const std::vector< std::vector<int> > *Partitions, int *Vertex );
inline long ChooseByFirstViolated_BB (const T_Instance &instance);
inline long ChooseEdge_by_LargestSmallest_Cijweight_BB(const T_Instance &instance, const int &mult);
inline long ChooseEdge_by_LargestSmallest_Cijweight_BB(const T_Instance &instance, const int &mult, const std::vector< std::vector<int> > &Partitions);
inline long ChooseEdge_by_NearestToFixVal_BB  (const T_Instance &instance, const double & fixVal);
inline long ChooseEdge_by_NearestToFixVal_BB (const T_Instance &instance, const double & fixVal,const  std::vector< std::vector<int> > &Partitions);
inline bool IsBinary(const double &val);
long ChooseEdge_StrongBranching_BB( T_Instance &instance, std::vector<T_fixVar> *fixvar);
inline int ChooseVertex_by_Weight_Partition_BB(const T_Instance &instance, const std::vector< std::vector<int> > &Partitions);
inline int  ChooseVertex_by_ClosestFeasible_BB  (const T_Instance &instance, const double & fixVal,const  std::vector< std::vector<int> > &Partitions);
inline bool IsValidEdge(const long &aux_edge,const std::vector<T_fixVar> *fixvar );
inline int ChooseVertexStrongBranching_by_Partition_BB(const T_Instance &instance, const std::vector< std::vector<int> > &Partitions);
inline long ChooseEdge_by_ClosestFeasible_BB (const T_Instance &instance, const double & fixVal,const std::vector<T_fixVar> *fixvar);
inline long Pick_FirstEdgeAvailable(const  T_Instance &instance, const std::vector<T_fixVar> &fixvar);

void CreatNewCandidates_FixVariable_BB(std::set <T_Branch> &ListBB, T_Instance &instance,std::vector<T_fixVar> &fixvar, double &bestLB);
void Creat_TwoCandidates_BB (std::set <T_Branch> &ListBB, T_Instance &instance, const std::vector<T_fixVar> &fixvarOrig, double &bestLB, const double &selectedEdge);
void  fixMoreVar_Dichtomic_triangle (const T_Instance &instance,std::vector <T_fixVar> *fixvar, const T_fixVar &lastVar );
void Fix_reduced_cost_dichotomic ( T_Instance & instance, std::vector<T_fixVar> *fixvar , const double &bestLowerBound);


bool GetExceptionTriangle(const T_Instance &instnace, const int & i, const int &j, const int &h);

double ICH_Heuristique_Ghaddar_oneIteration(const T_Instance &instance);
void CreatNewCandidates_FixPatitions_RedCost_BB(std::set <T_Branch> &ListBB, T_Instance &instance, std::vector< std::vector<int> > &Partitions, double *bestLowerBound);
void CreatNewCandidates_FixPatitions_BB(std::set <T_Branch> &ListBB, T_Instance &instance, std::vector< std::vector<int> > &Partitions, double *bestLowerBound);
inline bool Select_validVertex_BB (T_Instance &instance, const std::vector< std::vector<int> > &Partitions ,double *bestLowerBound, int *selectedVertex);
inline bool Select_validVertex_BB_zeroAllPartition (T_Instance &instance, const std::vector< std::vector<int> > &Partitions ,double *bestLowerBound, int *selectedVertex);
inline int Find_firstVertexNotInPartition_BB (const std::vector<bool> &verfVertex, const int &size);
void Set_FixVar_fromPartition_BB(const T_Instance &instance,const std::vector< std::vector<int> > &Partitions, std::vector <T_fixVar> &fixvar);
void Creat_Candidate_Partition_BB(std::set <T_Branch> &ListBB, T_Instance &instance, const std::vector< std::vector<int> > &Partitions , const int &selectedVertex, const int & part_Dest, double *bestLowerBound);
void Creat_Candidate_Partition_RedCost_BB(std::set <T_Branch> &ListBB, T_Instance &instance, const std::vector< std::vector<int> > &Partitions , const int &selectedVertex, const int & part_Dest, double *bestLowerBound, double*redcost);
void FixVarible_to_ONE_RCisBig_BB (const T_Instance &instance, std::vector< std::vector<int> > &Partitions, double *bestLowerBound,double*redcost);

void ImprovePartition_FixMoreVertices(const T_Instance &instance, std::vector< std::vector<int> > &Partitions, const double  &bestLB);
bool  Set_TrivialVertices_inPartition_BB(const T_Instance &instance, std::vector< std::vector<int> > &Partitions, double  *bestLowerBound );

//sdp in BB
bool SolveMosek_SDP_for_BB (T_Instance &instance, const std::vector<T_fixVar> &fixvar);
inline  void Set_FixVar_MOSEK_SDP_BB(const T_Instance &instance,MSKrescodee &r, MSKtask_t &task, const double &LBsdp,const std::vector<T_fixVar> &fixvar);
inline void Transform_SDPsol_2_LPsol_for_BB(T_Instance &instance, double*barx, const double &LBsdp, const double &UBsdp);

//fix intial vertex 
void Fix_N_Vertices_and_CreatPartitions(T_Instance &instance, std::set <T_Branch> &ListBB_Orig, const int & qtNb);
bool Select_validVertex_from_RankedVector_BB(const std::vector< std::vector<int> > &Partitions, const std::vector<int> &ranked_v, int *selVer);
void SetEdges_from_PartitionCandidates_BB(T_Instance &instance, std::set <T_Branch> &ListBB);


//fix more vertices 
double Val_newVertex_inPartition( const T_Instance &instance, const std::vector< std::vector<int> > &Partitions, const int &P_in, const int &v_out );
// void Set_vertices_byLPSolution(const T_Instance &instance, std::vector< std::vector<int> > &Partitions, double  *bestLowerBound );
bool Set_FixMoreVertex_bySolution_inPartition_BB(const T_Instance &instance, std::vector< std::vector<int> > &Partitions, double  *bestLowerBound ) ;
void Set_vertices_byLPSolution_allZeros(const T_Instance &instance, std::vector< std::vector<int> > &Partitions, double  *bestLowerBound ) ;
void Set_vertices_byLPSolution_Extreme(const T_Instance &instance, std::vector< std::vector<int> > &Partitions, double  *bestLowerBound ) ;
void Set_vertices_byLPSolution(const T_Instance &instance, std::vector< std::vector<int> > &Partitions, double  *bestLowerBound ) ;
double UpperBoundSmallestEigenvalue_Partition(const T_Instance& instance, const std::vector<bool> VerInPartition);
double UpperBoundSmallestEigenvalue_Partition_AllVerticesToo(const T_Instance& instance, std::vector< std::vector<int> > &Partitions);
double LocalSearch_WithFixedVertices(const T_Instance &instance,const std::vector< std::vector<int> > &Partitions_old);
bool ImprovePartition_FixMoreVertices2(const T_Instance &instance, std::vector< std::vector<int> > &Partitions, const double  &bestLB);

#endif 
