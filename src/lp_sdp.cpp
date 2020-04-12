/*


		vars.add(SelectMama_and_PapaNumVar(env, 0.0, 40.0));
		vars.add(IloNumVar(env));
		vars.add(IloNumVar(env));

		IloExpr xpr(env);
		xpr += 1.0*vars[0];
		xpr += 2*vars[1];
		xpr += 3*vars[2];

		//IloObjective obj; = IloMinimize(env, x1 + 2*x2 + 3*x3);
		IloObjective obj = IloMaximize(env);
		obj(0)=1;
		obj(1)=2;
		obj(2)=3;

		model.add(obj);

		http://www.iems.ucf.edu/qzheng/grpmbr/seminar/Daniel_Using_C++_with_CPLEX.pdf

		for(i=0;i<4;i++){
		IloExpr xpr(env);
		for(j=0;j<3;j++){
		  xpr += (i+1)*var_x[j];
		}
		sprintf(temp,"cst_a(%d)",i+1);
		cst_a.add(IloRange(env,0,xpr,+IloInfinity,temp));
		xpr.end();
		}

		model.add(IloMaximize(env, xpr));
		model.add( - vars[0] +     vars[1] + vars[2] <= 20);
		model.add(   vars[0] - 3 * vars[1] + vars[2] <= 30);

		// Solve
		cplex.solve();

*/

#include "../headers/MainHead.hpp"

//just to test git Novo mudanca
//nova mul ultimo test
//Essa es a mudanca
/* This function prints log output from MOSEK to the terminal. */
static void MSKAPI printstr(void *handle,
                            const char str[])
{
  printf("%s", str);
} /* printstr */

/******* Global variables **************/
const int num_of_threads = 1;
//MSKint32t NUMCON 	= 0; //antes era 9

const double epslon = 0.0001; //
const double epslon_s = 0.000001;
const double D_INFINITY = 10000000.00;
int NBMAXINEQ = 1000;
int K = 3;
double Obj1;
int nb_ITE = 1000;
bool SepMethod2 = true;   // false = not actived
double MinIteImpro = 0.0; //improvement at each iteration //before was .00001
double MinIteImpro_subGraph = 0.01;
int SDP_SEP = 0; /* 0 = Not activated (Just LP optimization)
				1 = Just negatives eigenvalues inequalities
				2 = Negatives and positive eigenvalues
				3 = Randon selection/combinations of negatives eigenvectors
				4 = Miguel combinations of negatives eigenvectors
				5 = Vilmar new vectors based in X solutin /It is not working well

				*/
double time_IPM_iteration;
int PreVP = 0; /*  0 = none
			    6 = Pre-treatment using householder matrix
			    7 = pre-treatment using vectors based in objective function
			    8 = both are activated
			*/

bool newMethod = false;

int ICH = 0;
//Separation method for separation
int TypeTriangle = 1; //0 or 1
int TypeHeurClique = 1;
int TypeHeurGeClique = 1;
int TypeHeurWheel = 4;   // 0 Without wheel Inequality
                         // 1 Grasp
                         // 2 Genetic
                         // 4 My heuristic multiple size
int TypeHeurBiWheel = 2; // same than TypeHeurWheel
bool ISCOMPLETE_GRAPH = false;

//Calcule qtd of inequalities
int TOTAL_TRI = 0;
int TOTAL_CLI = 0;
int TOTAL_GCL = 0;
int TOTAL_WHE = 0;
int TOTAL_BWH = 0;
int TOTAL_SDPCUT = 0;

//Calcule qtd of inequalities
double Time_TRI = 0.0;
double Time_CLI = 0.0;
double Time_GCL = 0.0;
double Time_WHE = 0.0;
double Time_BWH = 0.0;
double Time_SDPCUT = 0.0;
double Time_IPM = 0.0;
double MAXTIME = 100000.0; // standard max time
double MAXTIME_subG = 20.0;
double MAXTIME_ITE = 300;
clock_t start; // mesure the time

// Population of inequalities
MKC_ConstraintSelectionPopulate PopBestIneq;

//Select the most violated inequalities
MKC_ConstraintCliquePopulate PopClique;
MKC_ConstraintCliquePopulate PopGenClique;
MKC_ConstraintTrianglePopulate PopTriangle;
MKC_ConstraintWheelPopulate PopWheell;
MKC_ConstraintWheelPopulate PopBiWheell;
MKC_ConstraintLPtoSDPPopulate PopVecProp;

double MAXVALUE;
double MinVP = 0.0;
std::vector<double> PrintEigPos;
std::vector<std::vector<int>> maximalCliques;

//Stuy of IPM
double TOL;

int NBINEQSDP = 400; // empirical test (it is the maximum of inequalities that we can add in the sdp)
bool DYNAMIC_INEQ = true;
bool DYNAMIC_ACTIV_INEQ = true;
bool ALL_INEQ_ACTIV = false;
bool DO_EIG_INEQ_FROM_SDP = false;
double MINGAP = 0.0250;
double RATIOGAP = 0.5; // should be 0,1
double gapImp = 1.0;
MSKrealt gapTolRel = 75;
double gapPrimal = 0.0;
double bestSol;

//iterative way of add edge constraints
bool SDP_EdGE_TYPE = true; //global parameter: if true the program set bounds inequalities of SDP in cutting plane algo ("true" gives better results)
bool SDP_EdGE_TYPE_BB = false;
T_SDP_EdgeConstraint sdpEdgeConst;

int NB_SDPineq;
int QT_DIV_EIG = 10; // It is going to take just (QT_DIV_EIG)% of all the violated Eigenvalue
                     // bool NEW_TASK_LP = true;

//file lecture
std::string FileResults = "",
            fileResult, // name file to writhe summary of final result
    FileLecture,        // name File to read instance
    fileResult_ITE,     //name of FIle with iteation information; //name of FIle with iteation information
    fileResult_ITE_BB;
std::ofstream file_ITE,
    file_ITE_BB; //file to write iterations

int CLEAN = 2; // it was equals to 2

//Cplex global vars

int SIMPLEXITE = 0;
int LastConstraint = 0;
bool COMPLET_EIG = false;

////For BRANCH AND BOUND
#define TYPE_IPM_BB 1
#define TYPE_SIMPLEX_BB 2
#define TYPE_SDP_BB 3

//strategy of solution
#define EAGER_BB 1             //best  rst search strategy
#define LAZY_BB 2              //Worse first search strategy
int STRATEGY_SOL_BB = LAZY_BB; //EAGER_BB; //LAZY_BB;

//trategy for selecting next subproblem (How the List is ranked)
#define BeFS 1 //best  rst search strategy
#define WFS 2  //Worse first search strategy
#define BFS 3  //Breath first selection (all same level first)
#define DFS 4  // depth rst search
int SELEC_STRATEGY_BB = DFS;

//Branch rule
#define R1 1 // Most decided first
#define R2 2 //Best in MG
#define R3 3 //Least decided first
#define R5 5 // Edge with biggest weight
#define R6 6 // Strong Branching
#define R7 7 // PseudoCost
std::vector<T_PseudCost_BB> PseudoCcost;
int BRANCHING_RULE_BB = R7;

//If it perfromane separation by partition vertex or by edge
bool BY_PARTITION_BB = true; //true= By vertex (polytomic = K), false= by edge (dichotomic)

//Branch and cut
#define BnB 1 // (cut) Branch and boud
#define BnC 2 //Branch and cut
int TYPE_BRANCH = BnB;
std::vector<T_constraint> ExtraCONST_BB;

double seqqq_BB = 0.0;
double *redcost_BB;
MSKrescodee r_BB; //mosek Parameters
MSKtask_t task_BB = NULL;
MSKtask_t task_BB_2 = NULL;
MSKenv_t env_BB = NULL;
double SDP_FirstSol;
int TYPE_SOLVER_BB = 3;
double Last_parameter_BB;

std::vector<std::vector<int>> bestPartitions_BB;

bool JUST_TRIandCLI = false;

/******* END of Global variables **************/

using namespace Eigen;
using namespace std;
using namespace maxkcut;

int main(int argc, char *argv[])
{
  int a = 0;
  MKCInstance* new_instance = MKCInstanceBuilder<std::nullptr_t>::create()
                                 ->set_graph()
                                 ->set_graph_input_file(argv[1])
                                 ->set_type_graph(GraphType::CHORDAL)
                                 ->end_graph()
                                 ->set_K(3)
                                 ->build();

  cout << new_instance->get_graph()->to_string();

  SolverParam solverParm;
  solverParm.set_gap_primal(0.75)
      ->set_gap_tolerance(0.9)
      ->set_gap_relative_tolerance(0.75);

  Solver *solver = SolverFactory::create_solver(TypeSolver::LP_EARLY_MOSEK, solverParm);
  solver->create_environnement();
  MKC_ModelEdgeLP model = MKC_ModelEdgeLP(new_instance, solver);

  
  Solver *solverSDP = SolverFactory::create_solver(TypeSolver::SDP_MOSEK, solverParm);
  MKC_ModelEdgeSDP modelSDP = MKC_ModelEdgeSDP(new_instance, solverSDP);
  modelSDP.solve();
  cout << "After first ";

  cin.get();

  modelSDP.add_type_inequality(new MKC_InequalityTriangle());
  modelSDP.add_type_inequality(new MKC_InequalityClique(new_instance->get_K() + 1));
  modelSDP.add_type_inequality(new MKC_InequalityClique(new_instance->get_K() + 2));
  modelSDP.add_type_inequality(new MKC_InequalityWheel(3, 1));
  modelSDP.add_type_inequality(new MKC_InequalityWheel(3, 2));
  modelSDP.add_type_inequality(new MKC_InequalityLpSdp());


  model.add_type_inequality(new MKC_InequalityTriangle());
  model.add_type_inequality(new MKC_InequalityClique(new_instance->get_K() + 1));
  model.add_type_inequality(new MKC_InequalityClique(new_instance->get_K() + 2));
  model.add_type_inequality(new MKC_InequalityWheel(3, 1));
  model.add_type_inequality(new MKC_InequalityWheel(3, 2));
  model.add_type_inequality(new MKC_InequalityLpSdp());


  for (int ite = 0; ite < 21; ite++)
  {
    cout << "\n ---> Iteration = " << ite << "\n";
    model.solve();
    model.find_violated_constraints(100);

  }

  solver->to_string();
  cin.get();

  sdpEdgeConst.size = 0;
  sdpEdgeConst.varI.clear();
  sdpEdgeConst.varJ.clear();

  double time3;
  T_Instance instance; //Store information of instance

  srand(time(NULL)); //to use the srand

  //  Inserting_Parameters(argv);
  Inserting_Parameters_BB(argv);

  TOL = (double)NBMAXINEQ;
  //if (SDP_SEP == -2)
  gapPrimal = 0.75;

  bestSol = 100000000000000000.0;

  //File to read !
  string FileLecture = "";
  for (int i = 0; argv[1][i]; i++)
    FileLecture += argv[1][i];

  //Read Instance
  Lire_Instance(FileLecture, instance); //Function to read

  // printvector(instance.cij.barc_i);

  //******* Set FILE of final results
  // *****
  set_FileNames(argv, fileResult);

  //FIle with iteation information
  set_FileNamesITE(argv, fileResult_ITE);
  file_ITE.open(fileResult_ITE.c_str());

  WriteIteration_FILE(instance, file_ITE, FileLecture, -1);

  //file Iteration BB
  set_FileNamesITE_BB(argv, fileResult_ITE_BB);
  file_ITE_BB.open(fileResult_ITE_BB.c_str());
  WriteIteration_FILE_BB(file_ITE_BB, -1, 0, 0.0, 0.0, 0.0, 0.0);

  // ***** end of files
  //  cout << "d =" <<  ((double)( instance.edge_nb ))/((instance.DIM*(instance.DIM-1))/2) << endl;

  bool IS_SPARSE = instance.edge_nb < 0.4 * (instance.DIM * (instance.DIM - 1) * 0.5);
  bool PRINT_ITERATIONS = true;

  //  IS_SPARSE = false;
  if (IS_SPARSE)
  {
    ISCOMPLETE_GRAPH = false;
    K_coreElimination(instance); //Eliminate trivial vertex
    Chordal_extention(instance); //Make it chordal
    //Block_Optimization2_SameBlock(instance); With chordal graphs we dont do Block optimization (for real world applications it must be interesting)
    if (((double)(instance.edge_nb)) / ((instance.DIM * (instance.DIM - 1)) / 2) > 0.75)
    {
      BuildCompleteGraph(instance);
      ISCOMPLETE_GRAPH = true;
      DO_EIG_INEQ_FROM_SDP = true;
    }
  }
  else
  {
    //Make the graph complete
    BuildCompleteGraph(instance);
    ISCOMPLETE_GRAPH = true;
    DO_EIG_INEQ_FROM_SDP = true;
  }

  /********* Special types of Solver (LP mostely)*********/
  //Study before and after
  if (SDP_SEP == 99)
  {
    JUST_TRIandCLI = true;
    SDP_SEP = 0;
  }
  else if (SDP_SEP == -99)
  {
    JUST_TRIandCLI = true;
    SDP_SEP = -1;
  }
  else
  {
    JUST_TRIandCLI = false;

    /*Two formulations that uses Node variables for LP formulations*/
    if (SDP_SEP == 10)
    {
      set_NodeEdge_Formulation(instance); /*Edge-node formulation see My thesis*/
      SDP_SEP = 0;                        // return to LP type of solver
    }
    else if (SDP_SEP == 11)
    {
      /*It is working just for complete graph (error formulations or code ?!)*/
      BuildCompleteGraph(instance);
      ISCOMPLETE_GRAPH = true;
      //Set_RepresentativeVar(instance); /*Old extended*/
      Set_ExtendedRepresentativeVar(instance); /*Extended formulations proposed in http://dx.doi.org/10.1016/j.endm.2016.03.044*/
      SDP_SEP = 0;                             // return to LP type of solver
    }
  }
  /****************************/

  if (instance.DIM > 0)
  {
    CuttingPlane_Optimization(instance, PRINT_ITERATIONS);
  }
  else
  {
    //     cout << "The solution is trivial with solution = " << instance.sum_cost << endl;
    WriteIteration_FILE_BB(file_ITE_BB, 1, 0, instance.sum_cost, instance.sum_cost, 0.0, getCurrentTime_Double(start));
    return 0;
  }

  //   cout <<"Fin do 1 Cutting plane"; cin.get();

  BranchAndBound(instance);

  //  bool DOHEURISTIC = true;
  //  if (false){
  //  	 clock_t start2 = std::clock();
  //  	double LB = VNS_Heuristic_FeasibleSoltion (instance);
  //  	instance.ObSol = LB;
  //  	time_IPM_iteration  = getCurrentTime_Double(start2);
  //  	WriteIteration_FILE(instance, file_ITE,FileLecture, -2); //Writing last iteration
  //  }
  //  if (DOHEURISTIC){

  //    clock_t start2 = std::clock();
  //    ICH_Heuristique(instance); cout << ", time = " << getCurrentTime_Double(start2)<< endl;

  //    start2 = std::clock(); std::vector<int> OriginVertices2;
  //   cout << "\nVal ICH NOVO =" << ICH_Heuristique_Ghaddar_main(instance) <<", time = " << getCurrentTime_Double(start2)<< endl;
  //    cout << "Val ICH =" << ICH_Heuristique_Ghaddar(instance, OriginVertices2, instance) <<", time = " << getCurrentTime_Double(start2)<< endl;
  //    cout << "Val MOH=" << MultipleSearch_Heuristic_FeasibleSoltion(instance);
  //     cout << "Val VNS=" << VNS_Heuristic_FeasibleSoltion (instance);
  //    TabuOptimum_Heuristic_FeasibleSoltion (instance);
  //    Grasp_Heuristic_FeasibleSoltion (instance);
  //    Hybrid_GraspAndTabu(instance);
  //  }//end if DOHEURISTIC

  //   MAXTIME = 10;
  //   CuttingPlane_Optimization(instance, PRINT_ITERATIONS);

  //  bool DOHEURISTIC_d = true;
  //     if (DOHEURISTIC_d){
  //    clock_t start2 = std::clock();
  //	double valll; int nbRound = 5;
  ////    ICH_Heuristique(instance); cout << ", time = " << getCurrentTime_Double(start2)<< endl;

  //    start2 = std::clock(); std::vector<int> OriginVertices2;
  //    ICH_Heuristique_Ghaddar(instance, OriginVertices2, instance);cout << ", time = " << getCurrentTime_Double(start2)<< endl;
  //	start2 = std::clock();   valll = 0.0;
  //	for (int i=0; i< nbRound; ++i )
  //    valll += MultipleSearch_Heuristic_FeasibleSoltion(instance);
  //	cout << "MOH Value = " << valll/nbRound << ", time = " << getCurrentTime_Double(start2)/nbRound<<  endl;

  //	start2 = std::clock();   valll = 0.0;
  //	for (int i=0; i< nbRound; ++i )c
  //    valll +=VNS_Heuristic_FeasibleSoltion (instance);
  //	cout << "VNS Value = " << valll/nbRound << ", time = " << getCurrentTime_Double(start2)/nbRound<<  endl;
  ////    TabuOptimum_Heuristic_FeasibleSoltion (instance);
  //	start2 = std::clock();   valll = 0.0;
  //	for (int i=0; i< nbRound; ++i )
  //    valll += Grasp_Heuristic_FeasibleSoltion (instance);

  //	cout << "GRASP Value = " << valll/nbRound << ", time = " << getCurrentTime_Double(start2)/nbRound<<  endl;
  ////    Hybrid_GraspAndTabu(instance);
  //  }//end if DOHEURISTIC

  return 0;
}

double BranchAndBound(T_Instance &instance)
{

  //	cout << "*************************" << endl ;
  //	cout << "***Branch and Bound ********" << endl;
  //	cout << "*************************" << endl ;
  //New start
  clock_t start_BB = std::clock();
  start_BB = start;

  bool RESUME = true;
  bool NEW_TASK = false;
  bool R, IsOK = true, NotOK = false;
  int counter = 0;
  int freqLB = 20, freqLB_BIG = 500; //if (SDP_SEP == -2)freqLB = 150;

  double MAXTIME_BB = 4800;
  double lb, bestLowerBound, previousLB = 0;      // heuristic methodc
  double bestUpperBound, LocalUb, previousUB = 0; // highest in B&B list

  Set_BranchingRule(instance, BY_PARTITION_BB);

  //For Branch and cut
  if (TYPE_BRANCH == BnC)
    ExtraCONST_BB = instance.CONST;
  else
    ExtraCONST_BB.clear();

  T_Branch *pt_branch;
  std::vector<T_fixVar> fixvar; //fixed variables
  std::set<T_Branch> ListBB;
  std::set<T_Branch>::iterator it_branch;
  std::vector<std::vector<int>> Partitions;

  //    cout << SDP_SEP; cin.get();

  //    if (SDP_SEP == -2)
  //      Set_IneqFuncObj(instance, 6669); //This way we limit our next solutions to be inferior than first solution. Impotant when we use Upperbound from Nikiforov

  if ((SDP_SEP == -1) || (SDP_SEP == -3))
    TYPE_SOLVER_BB = TYPE_SDP_BB;
  else
    TYPE_SOLVER_BB = TYPE_SIMPLEX_BB; //  TYPE_IPM_BB

  int nbVer = 2;
  if (SDP_SEP == -2)
    nbVer = 2;

  Fix_N_Vertices_and_CreatPartitions(instance, ListBB, nbVer); //creat first branches by fixing "nbVer" Vertices.

  if (BY_PARTITION_BB == false)
    SetEdges_from_PartitionCandidates_BB(instance, ListBB);

  //Initial LowerBound
  bestLowerBound = ICH_Heuristique_Ghaddar_main(instance); // a inital lower bound from ICH method.
  R = CheckAndClean_ListBB(ListBB, bestLowerBound, &bestUpperBound);
  if (Last_parameter_BB > bestLowerBound && Last_parameter_BB <= bestUpperBound)
  {
    bestLowerBound = Last_parameter_BB;
  }
  //   cout << bestLowerBound;

  /* ********* MAIN LOOP ****************** */
  while (ListBB.size() > 0 && RESUME)
  {
    //      Print_ListBB_allActive (ListBB); cin.get();
    //calculate a lower bound  (not for all candidates)
    if ((counter % freqLB == 0) || ListBB.size() == 1)
    {

      lb = VNS_Heuristic_FeasibleSoltion(instance);
      if (lb > bestLowerBound)
      {
        bestLowerBound = lb;
        // 	      bestLowerBound = 18512;
      }

      R = CheckAndClean_ListBB(ListBB, bestLowerBound, &bestUpperBound); //check all candidades if UP>LB (fathom by bound)
      if (R == false)
        goto END_OF_BB; // Signal of end of BB

      //Print screen
      if (bestLowerBound > previousLB + epslon || bestUpperBound < previousUB - epslon || counter % freqLB_BIG == 0)
      {
        // 		PrintScreen_BB(counter,ListBB.size() ,bestLowerBound,bestUpperBound,100*(bestUpperBound-bestLowerBound)/bestLowerBound, getCurrentTime_Double(start_BB));
        WriteIteration_FILE_BB(file_ITE_BB, counter, ListBB.size(), bestLowerBound, bestUpperBound, 100 * (bestUpperBound - bestLowerBound) / bestLowerBound, getCurrentTime_Double(start_BB));
        previousUB = bestUpperBound;
        previousLB = bestLowerBound;
      }
    }

    it_branch = ListBB.begin();
    if ((*it_branch).upperbound > bestLowerBound + 0.99)
      R = IsOK; //Resume
    else
      R = NotOK; //fathom by bound
    LocalUb = (*it_branch).upperbound;

    if (BY_PARTITION_BB)
      Partitions = (*it_branch).Partitions;
    else
      fixvar = (*it_branch).fixvar;

    if (counter > 1)
      ExtraCONST_BB.clear(); //extra constraints of each branch (clean to receive new in cutting plane)

    //solve the relaxation (if Lazy strategy is used)
    if (R == IsOK && STRATEGY_SOL_BB == LAZY_BB)
    {

      if (TYPE_BRANCH == BnB)
      {
        R = Solve_SubProblem_BB(*it_branch->ptxInst, &Partitions, fixvar, BY_PARTITION_BB, TYPE_SOLVER_BB, NEW_TASK, &bestLowerBound);
      }
      else
      { //branch and cut  (BnC)
        R = CuttingPlane_Simple_for_BB(*it_branch->ptxInst, &Partitions, fixvar, BY_PARTITION_BB, &bestLowerBound, 1, (*it_branch).ExtraCONST);
      }

      if (R == IsOK)
      {
        //	cout << (*it_branch).Level_tree << endl; cin.get();

        if (BRANCHING_RULE_BB == R7 && (*it_branch).Level_tree > 0)
          Set_NewValPseudoCost(LocalUb, (*it_branch).ptxInst->ObSol, (*it_branch).lastVarFix, (*it_branch).lastPartFix, &PseudoCcost);

        LocalUb = (*it_branch).ptxInst->ObSol; //actualization of LocalUB (used to see if we are going to continue )
      }
    } //end of Solve_subProblem

    //Fathom by bound ??
    if (LocalUb >= bestLowerBound + 0.99 && R == IsOK)
    {                                         //Fathom by Bound
      seqqq_BB = (*it_branch).Level_tree + 1; //Used in the branching
      R = Branching_BB(ListBB, *it_branch->ptxInst, Partitions, fixvar, BY_PARTITION_BB, TYPE_SOLVER_BB, &bestLowerBound);
    }
    //delete (parent) from list
    ListBB.erase(it_branch);

    //	   Print_ListBB_allActive(ListBB); cin.get();
    counter++;
    if (getCurrentTime_Double(start_BB) > MAXTIME_BB)
      RESUME = false;
  } //end WHILE

END_OF_BB:
  if (ListBB.size() == 0)
    WriteIteration_FILE_BB(file_ITE_BB, counter, 0, bestLowerBound, bestLowerBound, 0, getCurrentTime_Double(start_BB));
  else
    WriteIteration_FILE_BB(file_ITE_BB, counter, ListBB.size(), bestLowerBound, bestUpperBound, 100 * (bestUpperBound - bestLowerBound) / bestLowerBound, getCurrentTime_Double(start_BB));

  //	cout << endl << endl << "********End of Branch and Bound*************" << endl << endl ;
  //	cout << "Optimal Solution Value = " << bestLowerBound << endl << endl ;
  //	cout << "Number of visited nodes = " << counter << endl ;
  //	cout << "Total Time = " << getCurrentTime_Double(start_BB)<< endl << endl ;
  //	cout << "*********************" << endl ;

  // free memory (mosek_BB)
  MSK_deletetask(&task_BB);
  MSK_deleteenv(&env_BB);

  return bestLowerBound; //end o function
}

inline void Set_BranchingRule(const T_Instance &instance, const bool &BY_PARTITION_BB)
{

  if (BRANCHING_RULE_BB == R7)
  { //PseudoCost Branching
    if (BY_PARTITION_BB)
    {
      PseudoCcost.resize(instance.DIM);
      for (int i = 0; i < instance.DIM; ++i)
        PseudoCcost[i].Set_NbPart(K);
    }
    else
    {
      PseudoCcost.resize(instance.edge_nb);
      for (int i = 0; i < instance.edge_nb; ++i)
        PseudoCcost[i].Set_NbPart(2); //{D-, D+}
    }
  } //End IF R7
}

inline void Set_NewValPseudoCost(const double &Ub_before, const double &UB_new, const int &var, const int &part, std::vector<T_PseudCost_BB> *PseudoCcost)
{
  (*PseudoCcost)[var].SetGain(Ub_before - UB_new, part);
}

inline void Print_ListBB_allActive(const std::set<T_Branch> &ListBB)
{
  std::set<T_Branch>::iterator it_branch;
  int counter = 0;
  cout << "Printing Lista of active candidates with = " << ListBB.size() << endl;

  for (it_branch = ListBB.begin(); it_branch != ListBB.end(); ++it_branch)
  {
    cout << counter << ": UB=" << (*it_branch).upperbound << ", seq=" << (*it_branch).seq << ", level=" << (*it_branch).Level_tree;
    cout << "sizeConst=" << (*it_branch).ExtraCONST.size() << endl;
    counter++;
  }
}

//This function check all the candidade in list to see if their upper bound are bigger than lowerbound
inline bool CheckAndClean_ListBB(std::set<T_Branch> &ListBB, const double &bestLowerBound, double *biggestUpper)
{
  *biggestUpper = 0.0;
  std::set<T_Branch>::iterator it, aux_it;

  for (it = ListBB.begin(); it != ListBB.end();)
  {
    if ((*it).upperbound > *biggestUpper)
      *biggestUpper = (*it).upperbound;
    //aux to delete
    aux_it = it;
    ++it;
    if ((*aux_it).upperbound < bestLowerBound + 0.9)
      ListBB.erase(aux_it); //delete from list
  }                         //end of FOR IT

  if (*biggestUpper < bestLowerBound + 0.9 || ListBB.size() == 0) // Lb is optimal
    return false;                                                 // Print optimal value

  return true;
}

inline bool Branching_BB(std::set<T_Branch> &ListBB, T_Instance &instance, std::vector<std::vector<int>> &Partitions, std::vector<T_fixVar> &fixvar,
                         const bool &BY_PARTITION_BB, const int &TYPE_SOLVER_BB, double *bestLowerBound)
{
  bool R = true;
  if (BY_PARTITION_BB == true)
  {

    if (TYPE_SOLVER_BB != TYPE_SDP_BB)
      CreatNewCandidates_FixPatitions_RedCost_BB(ListBB, instance, Partitions, bestLowerBound);
    else
      CreatNewCandidates_FixPatitions_BB(ListBB, instance, Partitions, bestLowerBound);
  }
  else
  {
    CreatNewCandidates_FixVariable_BB(ListBB, instance, fixvar, *bestLowerBound);
  }
  return R;
} //end branching

inline bool Solve_SubProblem_BB(T_Instance &instance, std::vector<std::vector<int>> *Partitions, std::vector<T_fixVar> &fixvar,
                                const bool &BY_PARTITION_BB, const int &TYPE_SOLVER_BB, const bool &NEW_TASK, double *bestLowerBound, const bool &sdp_edge_type)
{
  SDP_EdGE_TYPE_BB = sdp_edge_type; //for bound separtion in SDP
  double UB_NIK;
  bool BOUND_NIKI = true;
  bool R = true;

  if (BY_PARTITION_BB == true)
  { // it is the old way (have to change it)

    // 	    if (TYPE_SOLVER_BB != TYPE_SDP_BB)
    //	  if (Partitions != NULL)
    //      R =  Set_TrivialVertices_inPartition_BB(instance,*Partitions,bestLowerBound); //If R not true ---> Prunne by Integer solution (not helping)

    if (Partitions != NULL)
      Set_FixVar_fromPartition_BB(instance, *Partitions, fixvar); //Set vector with all edges fixed from partition

    if (TYPE_SOLVER_BB == TYPE_SDP_BB)
      R = SolveMosek_SDP_for_BB(instance, fixvar); //solve using SDP solver
    else
      R = SolveMosek_LP_for_Branch(instance, r_BB, task_BB, env_BB, TYPE_SOLVER_BB, fixvar, NEW_TASK); //TYPE_SIMPLEX_BB
                                                                                                       // 	      PrintPartitionICH(Partitions); //cin.get();
  }
  else
  {

    if (TYPE_SOLVER_BB == TYPE_SDP_BB)
      R = SolveMosek_SDP_for_BB(instance, fixvar); //solve using SDP solver
    else
      R = SolveMosek_LP_for_Branch(instance, r_BB, task_BB, env_BB, TYPE_SOLVER_BB, fixvar, NEW_TASK); //TYPE_SIMPLEX_BB
  }

  return R;
} //end of Solve_subproblem

//Based on triangle inequality we can fix more variables
//see section 4.1.1 of paper of branch-and-bound (implementation of splitting)
void fixMoreVar_Dichtomic_triangle(const T_Instance &instance, std::vector<T_fixVar> *fixvar, const T_fixVar &lastVar)
{
  int v_I, v_J,
      n_H, n_Z;
  int var = lastVar.var,
      n_var, sumVal;
  T_fixVar newFixVal;
  double val_v = lastVar.val,
         val_n;
  long aux_edge;

  //from last variable (get vertices of the  edge  (lastVar.var)  )
  v_I = instance.cij.barc_i[var]; //remember that it start with 1 (not zero)
  v_J = instance.cij.barc_j[var];

  //remember a triangle (i, j ,h ) has the following edges
  //(i,j) --> from lastVar ; (i,h) and (j,h)

  int size = (int)(*fixvar).size(); //size of list of fixed variables

  //	 cout << "size =" << size << "v_I=" << v_I << ", v_J=" << v_J << ", val_v=" << val_v<<endl;

  //	 cout << "printing all exectep the last"<< endl;
  //	 	for (int i=0; i< size-1; ++i )
  //	{
  //		cout << i << ": " << instance.cij.barc_i[(*fixvar)[i].var]; //remember that it start with 1 (not zero)
  //		cout << " - " << instance.cij.barc_j[(*fixvar)[i].var] ;
  //		cout << "... val = " << (*fixvar)[i].val <<  endl;

  //	}

  for (int i = 0; i < size - 1; ++i)
  {
    n_H = -1;
    n_var = (*fixvar)[i].var;

    if (instance.cij.barc_i[n_var] == v_I)
    { //it means the exit a a h,i... thus we have to fix h,j
      n_Z = v_J;
      n_H = instance.cij.barc_j[n_var];
    }
    else if (instance.cij.barc_i[n_var] == v_J)
    { //it means the exit a a h,i... thus we have to fix h,j
      n_Z = v_I;
      n_H = instance.cij.barc_j[n_var];
    }
    else if (instance.cij.barc_j[n_var] == v_I)
    { //it means the exit a a h,i... thus we have to fix h,j
      n_Z = v_J;
      n_H = instance.cij.barc_i[n_var];
    }
    else if (instance.cij.barc_j[n_var] == v_J)
    { //it means the exit a a h,i... thus we have to fix h,j
      n_Z = v_I;
      n_H = instance.cij.barc_i[n_var];
    }

    if (n_H != -1)
    {
      aux_edge = instance.indice_cij[n_H + n_Z * (instance.DIM + 1)];
      if (aux_edge == -1) //not exit this edge on graph
        n_H = -1;
    }

    //found a valid edge
    if (n_H != -1)
    {

      val_n = (*fixvar)[i].val;

      //			cout << "size =" << size << "n_H=" << n_H << ", n_Z=" << n_Z << ", val_n=" << val_n; cin.get();
      sumVal = val_n + val_v;
      newFixVal.var = aux_edge;

      switch (sumVal)
      {
      case (1): //Best first
        newFixVal.val = 0;
        (*fixvar).push_back(newFixVal);
        break;
      case (2): //Worse first
        newFixVal.val = 1.0;
        (*fixvar).push_back(newFixVal);
        break;
      default: //Defaut (do nothing in case sum == 0)
        break;
      } //end switch
    }   //end  	 n_H != -1)
  }     //end for all fixed

  //	if ((int)(*fixvar).size() > size){
  //		cout << "Chegoug e incluiu  akii.... " << (int)(*fixvar).size()  - size ;
  //		cin.get();
  //	}
}
void SetEdges_from_PartitionCandidates_BB(T_Instance &instance, std::set<T_Branch> &ListBB)
{
  std::set<T_Branch>::iterator it_branch;
  std::set<T_Branch> newListBB;
  std::vector<T_fixVar> fixvar; //fixed variables
  T_Branch newBranch;

  for (it_branch = ListBB.begin(); it_branch != ListBB.end(); ++it_branch)
  {

    Set_FixVar_fromPartition_BB(*it_branch->ptxInst, (*it_branch).Partitions, fixvar); //Set vector with all edges fixed from partition

    newBranch.lowerboud = (*it_branch).lowerboud;
    newBranch.upperbound = (*it_branch).upperbound;
    newBranch.seq = (*it_branch).seq;
    newBranch.ptxInst = (*it_branch).ptxInst;

    newBranch.fixvar = fixvar;

    newBranch.ExtraCONST = ExtraCONST_BB;

    newListBB.insert(newBranch);

  } //end for iteration all branches

  ListBB.clear();

  for (it_branch = newListBB.begin(); it_branch != newListBB.end(); ++it_branch)
  {
    ListBB.insert((*it_branch));
  } //end for new list
}

/*Function to fix initial vertices and creat all the candidates (nb of candidates depend on qtNb and "K")*/
void Fix_N_Vertices_and_CreatPartitions(T_Instance &instance, std::set<T_Branch> &ListBB_Orig, const int &qtNb)
{

  int counter, selVer;
  set<MKC_InstanceRankVertices> ranked;
  set<MKC_InstanceRankVertices>::iterator it;
  vector<int> ranked_v;

  //var for BB
  bool NEW_TASK = true;
  double lb, bestLowerBound; // heuristic methodc
  bool R = true, flag = true;
  std::vector<T_fixVar> fixvar; //fixed variables
  std::set<T_Branch> ListBB;
  std::set<T_Branch>::iterator it_branch;
  std::vector<std::vector<int>> Partitions;
  T_Branch newBranch;

  int auxStra = STRATEGY_SOL_BB;
  STRATEGY_SOL_BB = LAZY_BB;

  /****** First phase : Rank the vertices ****/

  //rank the vertices
  ranked_v.resize(qtNb);
  //Rank_by_degree(ranked,instance ); // (WARNING) HERE vertices start in 1
  Rank_by_incidentWeight(ranked, instance); // (WARNING) HERE in ranked the vertices start in 1

  counter = 0;
  for (it = ranked.begin(); it != ranked.end() && counter < qtNb; ++it)
  {
    ranked_v[counter] = (*it).vertex - 1;
    counter++;
  }
  //printvector(ranked_v); //cin.get();

  /****** Second phase : Creat our first candidate to include in LIST ****/

  //Set_IneqFuncObj(instance, SDP_FirstSol); //This way we limit our next solutions to initial to be inferior than first solution of SDP

  //    if (SDP_SEP == -1) TYPE_SOLVER_BB = TYPE_SDP_BB;
  //    else 		 TYPE_SOLVER_BB = TYPE_IPM_BB;

  if (TYPE_SOLVER_BB == TYPE_SDP_BB)
    SolveMosek_SDP_for_BB(instance, fixvar);
  else
    SolveMosek_LP_for_Branch(instance, r_BB, task_BB, env_BB, TYPE_IPM_BB, fixvar, NEW_TASK);

  bestLowerBound = newBranch.lowerboud = VNS_Heuristic_FeasibleSoltion(instance);
  newBranch.upperbound = instance.ObSol;
  newBranch.seq = -1.0;
  newBranch.ptxInst = &instance;
  std::vector<int> newPartition(1, ranked_v[0]); // creat new partition with first vertex in partition 0 (can fix this vertex in first partition)
  newBranch.Partitions.push_back(newPartition);

  ListBB.insert(newBranch);

  /****** Third phase : Creat other candidate to include in LIST ****/
  seqqq_BB = 0;

  while (ListBB.size() > 0)
  {

    it_branch = ListBB.begin();

    if (TotalNbVertices_Partition((*it_branch).Partitions) >= qtNb)
    { //good to go
      //insert int the original list
      ListBB_Orig.insert((*it_branch));
    }
    else
    { // generate candidates from (*it_branch)
      Partitions = (*it_branch).Partitions;
      //PrintPartitionICH(Partitions ); cin.get();
      R = Select_validVertex_from_RankedVector_BB(Partitions, ranked_v, &selVer);

      if (R == true)
      {
        if (Partitions.size() < K)
          Partitions.resize(Partitions.size() + 1);
        flag = true;
        for (int part = 0; part < Partitions.size(); part++)
        {
          if (Partitions[part].size() != 0 || flag)
          { //add in just one void partitions
            if (Partitions[part].size() == 0)
              flag = false;

            //set selVer in partition part
            Creat_Candidate_Partition_BB(ListBB, instance, Partitions, selVer, part, &bestLowerBound); //creat candidates
          }

        } //end of FOR partition size
      }
    } //els ELSE generator

    ListBB.erase(it_branch);
  } //end of while

  STRATEGY_SOL_BB = auxStra;

} //end function

bool Select_validVertex_from_RankedVector_BB(const std::vector<std::vector<int>> &Partitions, const std::vector<int> &ranked_v, int *selVer)
{
  bool flag;
  for (int i = 0; i < ranked_v.size(); i++)
  {
    flag = true;
    for (int j = 0; j < Partitions.size() && flag; j++)
      for (int jj = 0; jj < Partitions[j].size() && flag; jj++)
      {
        if (Partitions[j][jj] == ranked_v[i])
          flag = false;
      }
    if (flag)
    {
      *selVer = ranked_v[i];
      return true;
    } //end IF flag

  } //end for i

  cout << "Maybe a problem in Select_validVertex_from_RankedVector_BB since it could not find a valid vertex to be included" << endl;
  cin.get();
  return false;
}
void Set_FixVar_fromPartition_BB(const T_Instance &instance, const std::vector<std::vector<int>> &Partitions, std::vector<T_fixVar> &fixvar)
{
  T_fixVar newVar;
  long aux_edge;
  int I, J;

  fixvar.clear();

  for (unsigned i = 0; i < Partitions.size(); i++)
  {
    for (unsigned ii = 0; ii < Partitions[i].size(); ii++)
    {
      I = Partitions[i][ii] + 1; //start by 0
      //from same partition (fix to 1 )
      for (unsigned iii = ii + 1; iii < Partitions[i].size(); iii++)
      {
        J = Partitions[i][iii] + 1; //start by 0
        aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];

        if (aux_edge != -1)
        { //it is a valid edge
          newVar.var = aux_edge;
          newVar.val = 1.0; //val if vertex are in same partition

          fixvar.push_back(newVar);
        }
      }

      //From other partitions
      for (unsigned j = i + 1; j < Partitions.size(); j++)
        for (unsigned jj = 0; jj < Partitions[j].size(); jj++)
        {
          J = Partitions[j][jj] + 1; //start by 0
          aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
          if (aux_edge != -1)
          { //it is a valid edge
            newVar.var = aux_edge;
            newVar.val = 0.0; //edges in dif partitions

            fixvar.push_back(newVar);
          }
        } //and FOR j
    }
  } //end FOR i

} //end function

void CreatNewCandidates_FixPatitions_BB(std::set<T_Branch> &ListBB, T_Instance &instance, std::vector<std::vector<int>> &Partitions, double *bestLowerBound)
{
  bool R = true;
  int selectedVertex;
  bool flag = true;
  //   std::vector< std::vector<int> > Partitions;
  //
  //    Partitions = Part_Orig;

  /*select a vertex to be fixed*/
  R = Select_validVertex_BB(instance, Partitions, bestLowerBound, &selectedVertex); // If R not true ---> Prunne by Integer solution

  if (R == true)
  {
    if (Partitions.size() < K)
      Partitions.resize(Partitions.size() + 1);

    for (int i = 0; i < Partitions.size(); i++)
    {
      if (Partitions[i].size() != 0 || flag)
      { //add in just one void partitions

        if (Partitions[i].size() == 0)
          flag = false;

        Creat_Candidate_Partition_BB(ListBB, instance, Partitions, selectedVertex, i, bestLowerBound); //creat candidates
      }
    } //end of FOR partition size
  }
}

void CreatNewCandidates_FixPatitions_RedCost_BB(std::set<T_Branch> &ListBB, T_Instance &instance, std::vector<std::vector<int>> &Partitions, double *bestLowerBound)
{
  bool R = true;
  int selectedVertex;
  bool flag = true;
  double val, maxGap = 4.0,
              gap = instance.ObSol - *bestLowerBound; //var with all the reduced cost of our problem
                                                      //   redcost =  (double*) calloc(instance.totalVars,sizeof(double)); //Reduced cost variable  *redcost,
                                                      //   std::vector< std::vector<int> > Partitions;
                                                      //
                                                      //    Partitions = Part_Orig;

  //Get Reduced cost
  //	MSK_getreducedcosts(task_BB_2, MSK_SOL_BAS,0, instance.totalVars,redcost );

  //fixing a vertex a partition by reduced cost

  if (gap <= maxGap)
    FixVarible_to_ONE_RCisBig_BB(instance, Partitions, bestLowerBound, redcost_BB);

  /*select a vertex to be fixed*/
  R = Select_validVertex_BB(instance, Partitions, bestLowerBound, &selectedVertex); // If R not true ---> Prunne by Integer solution
                                                                                    //    R = Select_validVertex_BB_zeroAllPartition (instance, Partitions, bestLowerBound, &selectedVertex);  //(Just time consuming... nothing else)

  if (R == true)
  {
    if (Partitions.size() < K)
      Partitions.resize(Partitions.size() + 1);

    for (int i = 0; i < Partitions.size(); i++)
    {
      if (Partitions[i].size() != 0 || flag)
      { //add in just one void partitions

        if (Partitions[i].size() == 0)
          flag = false;

        Creat_Candidate_Partition_RedCost_BB(ListBB, instance, Partitions, selectedVertex, i, bestLowerBound, redcost_BB); //creat candidates
      }
    } //end of FOR partition size
  }
}

//this function fixes the variables  that have Reduced cost (RC) bigger than gap ... (up - lp)... Therefore, if  RC_e > up - lp we are goint to put a vertex in the suggested partition ;
void FixVarible_to_ONE_RCisBig_BB(const T_Instance &instance, std::vector<std::vector<int>> &Partitions, double *bestLowerBound, double *redcost)
{
  int I, J;
  long aux_edge;
  double val,
      gap = instance.ObSol - (*bestLowerBound + 1.0); //var with all the reduced cost of our problem
  vector<int> InPartition(instance.DIM, -1);          //determine the partition of each vertex (if it is in partition, -1 otherwise)

  for (int part = 0; part < Partitions.size(); ++part)
    for (int j = 0; j < Partitions[part].size(); ++j)
      InPartition[Partitions[part][j]] = part; // part is the partition

  for (int i = 0; i < instance.DIM; ++i)
    if (InPartition[i] == -1)
    {
      for (int part = 0; part < Partitions.size(); ++part)
      {
        val = 0.0;
        for (int j = 0; j < Partitions[part].size(); ++j)
        { //vertex in partition
          I = i + 1;
          J = Partitions[part][j] + 1;
          aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];

          if (aux_edge != -1) //it is a valid edge
            if (instance.varS[aux_edge] > epslon)
            {
              val += redcost[aux_edge];

              if (val > gap)
              {
                Partitions[part].push_back(i);

                part = Partitions.size();
                break;
                //return void (); //normally there is just one vertex
              }
            }
        }
      } // end FOR  Partition
    }   //end of FOR i

} //end of function

//Using reduced cost to fix variables
// Let edge e = {i,j}, and RedCost_e the reduced cost of variable "e".
//Based on the proposition: IF  (RedCost_e < lb - up), then var [e] can be fixed to zero ...
// it means that vertex j and i cannot be in same partition (never)
void Creat_Candidate_Partition_RedCost_BB(std::set<T_Branch> &ListBB, T_Instance &instance, const std::vector<std::vector<int>> &Partitions, const int &selectedVertex, const int &part_Dest, double *bestLowerBound, double *redcost)
{
  int aux_edge, I, J;
  double RC_Sum = 0.0,
         gap = (*bestLowerBound + 1.0) - instance.ObSol,
         val;
  //	std::vector< std::vector<int> > Partitions = Partitions_orig;
  //  cout << " instance.ObSol = " <<  instance.ObSol << ", lb = " <<    *bestLowerBound  << " and gap=" << gap <<endl ;

  //if Redcost < lb - up ... should be fixed to zero
  J = selectedVertex + 1;
  for (int i = 0; i < Partitions[part_Dest].size(); i++)
  {
    I = Partitions[part_Dest][i] + 1;
    aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
    if (aux_edge != -1) //it is a valid edge
      if (instance.varS[aux_edge] <= epslon)
      {
        val = redcost[aux_edge];
        RC_Sum += val;
        if (RC_Sum < gap || val < gap)
        {
          return void(); // end function we dont need to creat this branch
        }
        //cout <<"Rc = " <<   redcost[aux_edge] ; cin.get();
      }
  }

  ///
  ///		Look to fix to 1 (set another vertex I  in same than vertex J is going)
  ///
  //   std::vector<bool> NotinPartition (instance.DIM, true); //easy way to check vertices  NOT in partition

  //  //if vertex is INSIDE partitions (it will have a value true)
  //   for (unsigned j=0; j<Partitions.size(); j++)
  //	for (unsigned jj=0; jj<Partitions[j].size(); jj++){
  //		NotinPartition[Partitions[j][jj]] = false;
  //	}

  //	for (int i=0; i<instance.DIM; ++i){
  //			aux_edge = instance.indice_cij[(i+1)+J*(instance.DIM+1)];
  //			if (aux_edge != -1 && NotinPartition[i]){
  //				val =  instance.varS[aux_edge];
  ////				cout << "val_e = " << val_e;
  ////				cout << " redcost[aux_edge]  = " << redcost[aux_edge] ;  cin.get();
  //				if (val>= 1.0 - epslon){ //val is 1.0
  //					if (-(redcost[aux_edge]) < gap){
  //						cout <<"gap=" << gap << ", -(redcost[aux_edge]) " << -(redcost[aux_edge]) ; cin.get();
  //						Partitions[part_Dest].push_back(i);
  //					}
  //				}
  //			}//end if valid
  //		}//end for i and j

  //creat new branch in enumeration tree
  Creat_Candidate_Partition_BB(ListBB, instance, Partitions, selectedVertex, part_Dest, bestLowerBound);
}

void Creat_Candidate_Partition_BB(std::set<T_Branch> &ListBB, T_Instance &instance, const std::vector<std::vector<int>> &Partitions, const int &selectedVertex, const int &part_Dest, double *bestLowerBound)
{
  //   cout << ",v=" << selectedVertex;
  T_Branch newBranch;
  bool R;

  //setting new branch
  newBranch.Partitions = Partitions;
  newBranch.Partitions[part_Dest].push_back(selectedVertex);

  if (STRATEGY_SOL_BB == EAGER_BB)
  {
    std::vector<T_fixVar> fixvar;
    R = Solve_SubProblem_BB(instance, &newBranch.Partitions, fixvar, true, TYPE_SOLVER_BB, false, bestLowerBound);
    // 		cout << "instance.ObSol" << instance.ObSol; cin.get();
    if (instance.ObSol <= *bestLowerBound + 0.99)
      return void(); //Fathom by bound
  }

  newBranch.lowerboud = *bestLowerBound;
  newBranch.upperbound = instance.ObSol; //from generator solution

  newBranch.Level_tree = seqqq_BB;
  newBranch.lastVarFix = selectedVertex;
  newBranch.lastPartFix = part_Dest;

  newBranch.ExtraCONST = ExtraCONST_BB; //for branch and cut
  //newBranch.seq = -instance.ObSol;      //Others options:  Get_valueOfPartition (instance,newBranch.Partitions); //seqqq_BB ; //
  R = Strategy_selecting_ActiveNodeTree_BB(instance, &newBranch.seq);

  newBranch.ptxInst = &instance;

  //insert in Lista
  ListBB.insert(newBranch);

} //end function

bool Set_FixMoreVertex_bySolution_inPartition_BB(const T_Instance &instance, std::vector<std::vector<int>> &Partitions, double *bestLowerBound) //If R not true ---> Prunne by Integer solution
{

  //   fazer isso aqui ... vai da compilation erro soh para eu saber onde comecar o trabalho !!!
  //Do a heuristic method with fixed partition (VNS or just a Local search), This is the lower bound that we are going to send to next two functions
  //moreover if this heuristc is better than bestlower, than we should change it

  double lb;
  lb = LocalSearch_WithFixedVertices(instance, Partitions);
  cout << "lb = " << lb << endl;
  if (lb > *bestLowerBound)
    cin.get();

  Set_vertices_byLPSolution_Extreme(instance, Partitions, &lb);
  //   Set_vertices_byLPSolution(instance, Partitions,bestLowerBound );

  //  Set_vertices_byLPSolution_allZeros(instance, Partitions,bestLowerBound );

  if (TotalNbVertices_Partition(Partitions) == instance.DIM)
  {
    lb = Get_valueOfPartition(instance, Partitions);
    cout << "new lb in bySolution = " << lb << endl; //cin.get();
    if (lb > *bestLowerBound)
      *bestLowerBound = lb;
    return false; // no need to do nothing else with this candidate cause it has already all vertex in partition
  }

  return true;
}

double LocalSearch_WithFixedVertices(const T_Instance &instance, const std::vector<std::vector<int>> &Partitions_old)
{

  int bestNewPartition, I, J, aux_edge, random,
      counter = 0, max_iteration = 30;
  double sumEdge_SP, sumEdge, bestSumEdge;
  //PrintPartitionICH(Partitions); //cin.get();
  //cout << "Initial value = " << Get_valueOfPartition (instance,Partitions ) << endl;
  //   cout << "Old partition" ;
  //   PrintPartitionICH(Partitions_old);

  if (Partitions_old.size() < K)
    return -1.0; //end of function

  std::vector<std::vector<int>> Partitions;
  Partitions = Partitions_old;

  //Vertices that are in partition
  std::vector<bool> VerInPartition(instance.DIM, false);
  for (unsigned i = 0; i < Partitions.size(); i++)
    for (unsigned ii = 0; ii < Partitions[i].size(); ii++)
      VerInPartition[Partitions[i][ii]] = true;

  //complete paratitions
  for (unsigned i = 0; i < instance.DIM; i++)
    if (!VerInPartition[i])
    {
      random = (rand() % K);           // randonly choose a partition
      Partitions[random].push_back(i); //insert vertex i in partition
    }                                  //end of if

  //     cout << "with random  partition val=" << Get_valueOfPartition (instance,Partitions );
  //     PrintPartitionICH(Partitions);

  bool CHANGES = true;

  while ((CHANGES) && (counter < max_iteration))
  {
    counter++;
    CHANGES = false;
    for (unsigned i = 0; i < Partitions.size(); i++)
    {

      for (unsigned ii = 0; ii < Partitions[i].size(); ii++)
        if (!VerInPartition[Partitions[i][ii]])
        { //can not change the fixed vertices

          //calcule if we take vertex ii from partition i
          sumEdge_SP = 0.0;
          for (unsigned iii = 0; iii < Partitions[i].size(); iii++)
          {
            if (iii == ii)
              continue;

            I = Partitions[i][ii] + 1;  // start by 1
            J = Partitions[i][iii] + 1; // start by 1

            aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
            if (aux_edge != -1) //it is a valid edge
              sumEdge_SP += instance.cij.barc_v[aux_edge];
          }

          //cout << "sumEdge_SP = " << sumEdge_SP << endl;

          { //find best partition to be put

            bestNewPartition = -1;
            bestSumEdge = D_INFINITY;
            I = Partitions[i][ii] + 1; // start by 1

            for (unsigned j = 0; j < Partitions.size(); j++)
            {
              if (i == j)
                continue; // do nothing
              sumEdge = 0.0;
              for (unsigned jj = 0; jj < Partitions[j].size(); jj++)
              {
                J = Partitions[j][jj] + 1; // start by 1
                aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
                if (aux_edge != -1) //it is a valid edge
                  sumEdge += instance.cij.barc_v[aux_edge];

                //cout << "x{"<< I <<","<< J <<  "} -- cost = " << instance.cij.barc_v[aux_edge] << endl; //cin.get();
              } //end 	FOR ii and jj

              if (((sumEdge < sumEdge_SP) && (sumEdge < bestSumEdge)))
              {
                //cout << "Partition " << j << ",  sumEdge_SP ="<<  sumEdge_SP<< ",  sumEdge = " << sumEdge << endl ; cin.get();
                bestSumEdge = sumEdge;
                bestNewPartition = j;
              }
            } //end for J

          } //end find best partition

          if (bestNewPartition != -1)
          {
            Partitions[bestNewPartition].push_back(Partitions[i][ii]); //add element ii in partition "bestNewPartition"
            Partitions[i].erase(Partitions[i].begin() + ii);           //erase ii from i partition
            ii--;
            CHANGES = true;
          } //end of if Bestpartition

        } //end FOR ii
    }     //end FOR i
  }       //end of WHILE

  //    cout << "New  partition val=" << Get_valueOfPartition (instance,Partitions );
  //     PrintPartitionICH(Partitions); cin.get();
  //
  return Get_valueOfPartition(instance, Partitions);
  //PrintPartitionICH(Partitions); cin.get();
}

void Set_vertices_byLPSolution(const T_Instance &instance, std::vector<std::vector<int>> &Partitions, double *bestLowerBound)
{
  int I, J, P_in, v_out;
  double val, val_P, val_Pbst, val_Pin;
  bool flag = false;

  std::vector<int> VerInParti(instance.DIM, -1);

  double Upper = instance.ObSol,
         gapUL = Upper - *bestLowerBound,
         compGap;

  std::vector<std::vector<int>> newPartitions;
  newPartitions = Partitions;

  // 	cout << "The partition " << endl;
  //    PrintPartitionICH(newPartitions); //cin.get();

  for (int i = 0; i < Partitions.size(); i++)
    for (int ii = 0; ii < Partitions[i].size(); ii++)
      VerInParti[Partitions[i][ii]] = i;

  //     cout << "The vector ";
  //     printvector(VerInParti);

  for (long i = 0; i < instance.edge_nb; i++)
  {

    //check partitions of variables in edge i
    I = VerInParti[instance.cij.barc_i[i] - 1]; // remember that in cij vertex start at 1
    J = VerInParti[instance.cij.barc_j[i] - 1];

    val = instance.varS[i];

    if (IsBinary(val))
    { // val is binary
      if ((I == -1 && J != -1) || (I != -1 && J == -1))
      { // one of the partitions should not be in a parttition

        //Check val is 0 or 1.0
        if ((val - 0.9) >= 0.0)
        {
          //v_out is the vertex that is not fixed, yet.
          if (I != -1)
          {
            P_in = I;
            v_out = instance.cij.barc_j[i] - 1;
          }
          else
          {
            P_in = J;
            v_out = instance.cij.barc_i[i] - 1;
          }

          // 	  cout << "Vertex I=" << instance.cij.barc_i[i]-1 << ",J=" <<instance.cij.barc_j[i]-1 << ", v_out=" << v_out << ", P_in=" << P_in << endl;
          val_Pbst = 0.0;
          for (int j = 0; j < Partitions.size(); j++)
          {
            val_P = Val_newVertex_inPartition(instance, Partitions, j, v_out); // VALUE if v_out is in partition j
                                                                               // 	    cout << "val= " << val_P<<endl;
            if (j != P_in)
            {
              if (val_P > val_Pbst)
                val_Pbst = val_P;
            }
            else
            { // Partition is that he is already
              val_Pin = val_P;
            }
          } //end for new partitions J
          compGap = val_Pin - val_Pbst;

          // 	    cout << "val_Pin=" << val_Pin << ", val_Pbst=" << val_Pbst << ", compGap = "<< compGap << ", gapUL = " << gapUL; cin.get();

          if (compGap > gapUL)
          { //thus v_out should be in partition P_in

            // SHould put v_out in partition P_in
            Partitions[P_in].push_back(v_out); // set in Partition P_in
            VerInParti[v_out] = P_in;
            i = -1; //reset i
            flag = true;
          } // end IF compGap
        }
        else
        {           // val is approx 0.0
          continue; // I can do nothing, I guess
        }
      } //end if I and J are accepted
    }   //end  if binary
  }     //end of for iteration all edges

  /*
  if (flag){
	int bf;
       bf = TotalNbVertices_Partition (newPartitions ) ;
	cout << endl << "Set_vertices_byLPSolution has included : (" << TotalNbVertices_Partition (Partitions ) - bf << ") Vertices... now it has " << TotalNbVertices_Partition (Partitions )  << endl; cin.get();
// 	PrintPartitionICH(Partitions ); cin.get();
  }
  */
}

void Set_vertices_byLPSolution_Extreme(const T_Instance &instance, std::vector<std::vector<int>> &Partitions, double *bestLowerBound)
{
  int I, J, P_in, aux_edge;
  double val,
      val_bst, val_SeconBest;
  bool flag = false;
  double Upper = instance.ObSol,
         gapUL = Upper - *bestLowerBound,
         compGap;

  if (Partitions.size() < K)
    return void(); // end of function

  std::vector<int> VerInParti(instance.DIM, -1); // Easily identify vertex that are already fixed in partition
  for (int i = 0; i < Partitions.size(); i++)
  {
    if (Partitions[i].size() == 0)
      return void(); // end of function
    else
    {
      for (int ii = 0; ii < Partitions[i].size(); ii++)
        VerInParti[Partitions[i][ii]] = i;
    } //end of ELSE
  }

  std::vector<short> set_partition; // set partitons
  std::vector<int> set_vertex;      // to set vertices to be included in Partitions

  //   std::vector< std::vector<int> > newPartitions;
  // 	newPartitions = Partitions;

  for (int i = 0; i < instance.DIM; i++)
  {
    if (VerInParti[i] == -1)
    { // it is not in a partition

      cout << "***** v=" << i << endl;
      val_bst = 0.0;
      for (int j = 0; j < Partitions.size(); j++)
      {
        val = Val_newVertex_inPartition(instance, Partitions, j, i); // VALUE if vertex i is in partition j

        cout << "P_" << j << ", val= " << val << endl;

        if (val >= val_bst)
        {
          P_in = j;
          val_SeconBest = val_bst;
          val_bst = val;
        }
        else if (val > val_SeconBest)
          val_SeconBest = val;
      } //END FOR j

      compGap = val_bst - val_SeconBest;

      if (compGap > gapUL)
      { //thus v_out should be in partition P_in
        PrintPartitionICH(Partitions);

        cout << "gapUL=" << gapUL << ", compGap=" << compGap << ", val_bst=" << val_bst << ", val_SeconBest=" << val_SeconBest;
        cin.get();
        // SHould put v_out in partition P_in
        // 	  Partitions[P_in].push_back(i);
        set_partition.push_back(P_in);
        set_vertex.push_back(i);
        //newPartitions[P_in].push_back(i); // set in Partition P_in
        flag = true;
      } // end IF compGap
    }   //end IF vertex not in partition
  }     //end FOR instance.DIM

  if (flag)
  {
    int bf;
    bf = TotalNbVertices_Partition(Partitions);
    for (int i = 0; i < set_partition.size(); i++)
      Partitions[set_partition[i]].push_back(set_vertex[i]);
    cout << endl
         << "Set_vertices_byLPSolution has included : (" << TotalNbVertices_Partition(Partitions) - bf << ") Vertices... now it has " << TotalNbVertices_Partition(Partitions) << endl; // cin.get();
                                                                                                                                                                                        // 	PrintPartitionICH(Partitions ); cin.get();
  }
}

void Set_vertices_byLPSolution_allZeros(const T_Instance &instance, std::vector<std::vector<int>> &Partitions, double *bestLowerBound)
{
  int I, J, P_in, aux_edge;
  double val,
      epslon_val = 0.9,
      val_bst, val_SeconBest;
  bool flag = false;

  std::vector<int> VerInParti(instance.DIM, -1);

  double Upper = instance.ObSol,
         gapUL = Upper - *bestLowerBound,
         compGap;

  std::vector<std::vector<int>> newPartitions;
  newPartitions = Partitions;

  if (Partitions.size() < K)
    return void(); // end of function

  for (int i = 0; i < Partitions.size(); i++)
  {
    if (Partitions[i].size() == 0)
      return void(); // end of function
    else
    {
      for (int ii = 0; ii < Partitions[i].size(); ii++)
        VerInParti[Partitions[i][ii]] = i;
    } //end of ELSE
  }

  // 	cout << "The partition " << endl;
  //    PrintPartitionICH(newPartitions); //cin.get();

  for (int i = 0; i < instance.DIM; i++)
  {
    if (VerInParti[i] == -1)
    { // it is not in a partition

      I = i + 1;
      val = 0.0;
      //look in all the partitions
      for (int j = 0; j < Partitions.size(); j++)
        for (int jj = 0; jj < Partitions[j].size(); jj++)
        {
          J = Partitions[j][jj] + 1;
          aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];

          if (aux_edge != -1)                  //it is a valid edge
            if (instance.varS[aux_edge] > 0.0) // not be influencied by negatives values from SDP formulation
              val += instance.varS[aux_edge];
        } //end of FOR jj and j too

      //       cout << "chegou val= " << val ;
      if (val < epslon_val)
      { // val is close to 0

        val_bst = 0.0;
        for (int j = 0; j < Partitions.size(); j++)
        {
          val = Val_newVertex_inPartition(instance, Partitions, j, i); // VALUE if vertex i is in partition j
                                                                       // 	    cout << "val= " << val_P<<endl;

          if (val > val_bst)
          {
            P_in = j;
            val_SeconBest = val_bst;
            val_bst = val;
          }
          else if (val > val_SeconBest)
            val_SeconBest = val;
        } //END FOR j

        compGap = val_bst - val_SeconBest;

        if (compGap > gapUL - 1.0)
        { //thus v_out should be in partition P_in

          // 	  cout << endl <<  "i=" << i << ", P_in" << P_in;
          // 	  cout << endl <<  "val_bst=" << val_bst << ", val_SeconBest=" << val_SeconBest << ", compGap=" << compGap ;
          // 	  cout << ", gapUL=" << gapUL ; cin.get();

          // SHould put v_out in partition P_in
          Partitions[P_in].push_back(i); // set in Partition P_in
          VerInParti[i] = P_in;
          flag = true;
        } // end IF compGap

      } //we have found a violation

    } //end IF vertex not in partition

  } //end FOR instance.DIM

  /*
  if (flag){
	int bf;
       bf = TotalNbVertices_Partition (newPartitions ) ;
	cout << endl << "Set_vertices_byLPSolution has included : (" << TotalNbVertices_Partition (Partitions ) - bf << ") Vertices... now it has " << TotalNbVertices_Partition (Partitions )  << endl; cin.get();
// 	PrintPartitionICH(Partitions ); cin.get();
  }
  */
}

//Simulate the total value if we put v_out in partition P_in
double Val_newVertex_inPartition(const T_Instance &instance, const std::vector<std::vector<int>> &Partitions, const int &P_in, const int &v_out)
{
  long aux_edge, I, J = v_out + 1;
  double val = 0.0;

  for (int i = 0; i < Partitions.size(); i++)
    if (i != P_in)
    {
      for (int ii = 0; ii < Partitions[i].size(); ii++)
      {
        I = Partitions[i][ii] + 1;
        aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];

        if (aux_edge != -1) //it is a valid edge
          val += instance.cij.barc_v[aux_edge];
      } //end FOR ii
    }   //end FOR i

  return val;
}

//Calculate objective function by bound of Nikiforov
double Calculate_BoundNikiforov(const T_Instance &instance, const std::vector<std::vector<int>> &Partitions)
{
  double LargestVal, sumVal,
      Val_OtherVert_byPartition = 0.0,
      Val_OtherVert_Outside = 0.0,
      ValPartition = 0.0;
  std::vector<bool> NotinPartition(instance.DIM, true); //easy way to check vertices  NOT in partition

  //if vertex is INSIDE partitions (it will have a value true)
  int counter = 0;
  for (unsigned j = 0; j < Partitions.size(); j++)
    for (unsigned jj = 0; jj < Partitions[j].size(); jj++)
    {
      NotinPartition[Partitions[j][jj]] = false;
      counter++;
    }

  //	PrintPartitionICH(Partitions );  cin.get();
  //Calcule actual val of fixed partition
  ValPartition = Get_valueOfPartition(instance, Partitions) + instance.sum_cost; // actual val of partitions (verified)

  if (counter < instance.DIM)
  {
    //2nd part Calculate the Upper bound of maxKcut for edge not in partition
    Val_OtherVert_Outside = UpperBoundSmallestEigenvalue_Partition(instance, NotinPartition); //Upper bound based on Nikiforov for vertex not in partition

    //Calcule all the not in partition with
    Val_OtherVert_byPartition = 0.0;
    for (int i = 0; i < instance.DIM; i++)
      if (NotinPartition[i])
      {
        LargestVal = 0.0;
        for (int p = 0; p < Partitions.size(); p++)
        {
          sumVal = Val_newVertex_inPartition(instance, Partitions, p, i); // VALUE when vertex i is in partition j

          if (sumVal > LargestVal)
            LargestVal = sumVal;
        } //end FOR patition
        Val_OtherVert_byPartition += LargestVal;
      } //end FOR J
  }

  //  cout << "ValPartition=" << ValPartition << ", Val_OtherVert_byPartition = " << Val_OtherVert_byPartition << ", Val_OtherVert_Outside = " << Val_OtherVert_Outside; cin.get();
  return Val_OtherVert_byPartition + Val_OtherVert_Outside + ValPartition;
}

//main function to fix trivial vertices in partition
bool Set_TrivialVertices_inPartition_BB(const T_Instance &instance, std::vector<std::vector<int>> &Partitions, double *bestLowerBound)
{
  bool R = true;
  //ImprovePartition_FixMoreVertices(instance ,Partitions , *bestLowerBound);
  R = ImprovePartition_FixMoreVertices2(instance, Partitions, *bestLowerBound);

  //	PrintPartitionICH(Partitions );  cin.get();
  if (TotalNbVertices_Partition(Partitions) == instance.DIM && R)
  {
    double lb;
    lb = Get_valueOfPartition(instance, Partitions);

    if (lb > *bestLowerBound)
    {
      cout << "new lbb= " << lb << endl; //cin.get();
      *bestLowerBound = lb;
    }
    return false;
  }
  return R;
} //end of function

//akiSim
bool ImprovePartition_FixMoreVertices2(const T_Instance &instance, std::vector<std::vector<int>> &Partitions, const double &bestLB)
{

  bool flag = false; //controls changes in partition
  double LargestVal, secondLrg, sumVal, UtopicUB, ValEdge, LargestDif_ALL;
  int part_j, I, J;
  double gap; //= ub - bestLB;

  //Possible include new
  if (Partitions.size() < K)
  {
    return true;
  }
  else
  {
    for (int j = 0; j < Partitions.size(); j++)
      if (Partitions[j].size() == 0)
        return true;
  }

  //std::vector< std::vector<int> > newPartitions (Partitions);  //newPartitions = Partitions;
  std::vector<bool> NotinPartition(instance.DIM, true); //easy way to check vertices  NOT in partition
  std::vector<int> VerticesToBeFixed;
  std::vector<int> Partition_VectorToBeFixed;

  //if vertex is INSIDE partitions (it will have a value true)
  int counter = 0;
  for (unsigned j = 0; j < Partitions.size(); j++)
    for (unsigned jj = 0; jj < Partitions[j].size(); jj++)
    {
      NotinPartition[Partitions[j][jj]] = false;
      counter++;
    }

  if (counter >= instance.DIM - 1 || counter < instance.DIM * 0.3)
    return true; //avoid segmentation error UpperBoundSmallestEigenvalue_Partition

  double Val_OtherVert_byPartition;

  //Calcule actual val of fixed partition
  double ValPartition = Get_valueOfPartition(instance, Partitions) + instance.sum_cost; // actual val of partitions (verified)

  //2nd part Calculate the Upper bound of maxKcut for edge not in partition
  double Val_OtherVert_Outside = UpperBoundSmallestEigenvalue_Partition(instance, NotinPartition); //Upper bound based on Nikiforov for vertex not in partition
  bool FirstPart = false;
  bool ScndPart = false;

  flag = false;
  /************** Find best partition *********/
  //Iteration to calcule utopic upper bound of each vertex in a partition
  for (int i = 0; i < instance.DIM; i++)
    if (NotinPartition[i])
    {
      Val_OtherVert_byPartition = 0.0;

      //1st step: Calculate Maximum value of other with fixed partition
      for (int j = 0; j < instance.DIM; j++)
        if (NotinPartition[j] && i != j)
        {
          LargestVal = 0.0;
          for (int p = 0; p < Partitions.size(); p++)
          {
            sumVal = Val_newVertex_inPartition(instance, Partitions, p, j); // VALUE when vertex i is in partition j

            if (sumVal > LargestVal)
              LargestVal = sumVal;
          } //end FOR patition
          Val_OtherVert_byPartition += LargestVal;
        } //end FOR J

      //3rt part (Identify if there exist just one partition for vertex i or if it is infeasible)
      FirstPart = false;
      ScndPart = false; // if a second partition is also acceptable

      //	 cout << "ValPartition=" << ValPartition << "Val_OtherVert_byPartition=" << Val_OtherVert_byPartition << "Val_OtherVert_Outside=" << Val_OtherVert_Outside; cin.get();

      for (int p = 0; p < Partitions.size() && !ScndPart; p++)
      {
        sumVal = Val_newVertex_inPartition(instance, Partitions, p, i); // Simulate VALUE when vertex i is in partition j
        sumVal += ValPartition;                                         //sum of partition
        sumVal += Val_OtherVert_byPartition;                            // sum with other vertex maximum (over stimated) with partition
        sumVal += Val_OtherVert_Outside;                                // sum of other vertices maximum with other non in partition

        if (sumVal > bestLB)
        {
          part_j = p;
          //verify if just one of the partitions is allowed
          if (FirstPart)
            ScndPart = true;
          else
            FirstPart = true;
        }
      } //end FOR p

      if (!FirstPart)
      {
        //	   cout << "Return false ValPartition=" << ValPartition; cin.get();
        return false; //end program non of the partitions give upperbound great than LB
      }
      else if (!ScndPart)
      {
        VerticesToBeFixed.push_back(i);
        Partition_VectorToBeFixed.push_back(part_j);
        flag = true;
      } //end Else if Second Part
    }   //end for I

  //at least one change in newPartitions
  if (flag)
  {
    // 	int bf;
    //        bf = TotalNbVertices_Partition (Partitions ) ;

    for (int i = 0; i < VerticesToBeFixed.size(); i++)
      Partitions[Partition_VectorToBeFixed[i]].push_back(VerticesToBeFixed[i]);

    // 	cout << endl << "ImprovePartition_FixMoreVertices has included : (" << TotalNbVertices_Partition (Partitions ) - bf << ") Vertices... now it has" << TotalNbVertices_Partition (Partitions )  << endl;
    //PrintPartitionICH(Partitions );
    //cin.get();
  }

  return true;

  //cout << endl << "Fin de funcao"; cin.get();
}

double UpperBoundSmallestEigenvalue_Partition_AllVerticesToo(const T_Instance &instance, std::vector<std::vector<int>> &Partitions)
{
  double LargestVal, sumVal;
  std::vector<bool> InPartition(instance.DIM, false); //easy way to check vertices  NOT in partition

  //if vertex is INSIDE partitions (it will have a value true)
  int counter = 0;
  for (unsigned j = 0; j < Partitions.size(); j++)
    for (unsigned jj = 0; jj < Partitions[j].size(); jj++)
    {
      InPartition[Partitions[j][jj]] = true;
      counter++;
    }

  int I, J, counterVer = 0;
  long aux_edge;

  double eigVal = -1, MinEigVal, up, MaxEigVal, Val_Par;
  SelfAdjointEigenSolver<MatrixXd> es; // just for symmetric matrix (comp. it is 10 times faster than general method)
  double w_V = 0.0, minEig;
  std::vector<int> NewVal(instance.DIM, -1);

  //   printvector(NewVal);
  MatrixXd mat_W(instance.DIM, instance.DIM);

  w_V = 0.0;
  //Tranformation G in to adjacent matrix mat_W for valid Vertices
  for (int i = 0; i < instance.DIM; i++)
  {
    for (int j = i; j < instance.DIM; j++)
    {
      if (i != j)
      {

        I = i + 1; //i+1;
        J = j + 1; //j+1;
        //putting in xpr
        aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];

        if (aux_edge != -1 && (!InPartition[i] || !InPartition[j]))
        {                                              //it is a valid edge (at least i or j cannot be in partition to be count )
          mat_W(i, j) = instance.cij.barc_v[aux_edge]; // non weighted put  1.0 instead
          mat_W(j, i) = instance.cij.barc_v[aux_edge]; // non weighted put  1.0 instead
          w_V += instance.cij.barc_v[aux_edge];
        }
        else
        {
          //	  	cout << "ch" << j << ", " << i; cin.get();
          mat_W(i, j) = 0.0; // non weighted
          mat_W(j, i) = 0.0; // non weighted
        }
      }
      else if (i == j)
        mat_W(i, j) = 0.0; // non weighted put  1.0 instead
    }                      //end J
  }                        //end of I

  //    cout << mat_W << endl << endl  ; //Print Matrix
  //    cin.get();
  //Compute the eigenvalue and eigenvectr
  es.compute(mat_W);

  MinEigVal = es.eigenvalues()[0]; // Get min eigenvalue of G matrix
                                   //   MaxEigVal = es.eigenvalues()[instance.DIM -1];
  double nb = counter / Partitions.size();

  up = (((double)K - 1.0) / K) * (w_V - ((MinEigVal * (instance.DIM - counter + nb)) / 2.0)); // Nikiforov

  //    cout << endl << "MinEigVal = " << MinEigVal << ", w_V =" << w_V << "Up = " <<up << endl; cin.get();

  Val_Par = Get_valueOfPartition(instance, Partitions) + instance.sum_cost; // actual val of partitions (verified)

  return up + Val_Par;
}
//Upper bound base on mathematical formulation of Nikiforov and Adam&Sotirov (see article Max-k-cut and the smallest eigenvalue)
double UpperBoundSmallestEigenvalue_Partition(const T_Instance &instance, const vector<bool> VerInPartition)
{

  //cout << "Begin Eig find"; cin.get();
  //calcule eigenvalues and eigenvector is made by EIGEN (http://eigen.tuxfamily.org)
  int I, J, counterVer = 0;
  long aux_edge;

  double eigVal = -1, MinEigVal, up, MaxEigVal, up_SD;
  SelfAdjointEigenSolver<MatrixXd> es; // just for symmetric matrix (comp. it is 10 times faster than general method)
  double w_V = 0.0, minEig;
  std::vector<int> NewVal(instance.DIM, -1);

  counterVer = 0;
  for (int i = 0; i < instance.DIM; i++)
    if (VerInPartition[i])
    {
      NewVal[i] = counterVer;
      counterVer++;
    }

  //   printvector(NewVal);
  MatrixXd mat_W(counterVer, counterVer);

  w_V = 0.0;
  //Tranformation G in to adjacent matrix mat_W for valid Vertices
  for (int i = 0; i < instance.DIM; i++)
  {
    if (VerInPartition[i])
      for (int j = i; j < instance.DIM; j++)
      {
        if (i != j && VerInPartition[j])
        {

          I = i + 1; //i+1;
          J = j + 1; //j+1;
          //putting in xpr
          aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];

          if (aux_edge != -1)
          {                                                              //it is a valid edge
            mat_W(NewVal[i], NewVal[j]) = instance.cij.barc_v[aux_edge]; // non weighted put  1.0 instead
            mat_W(NewVal[j], NewVal[i]) = instance.cij.barc_v[aux_edge]; // non weighted put  1.0 instead
            w_V += instance.cij.barc_v[aux_edge];
          }
          else
          {
            mat_W(NewVal[i], NewVal[j]) = 0.0; // non weighted
            mat_W(NewVal[j], NewVal[i]) = 0.0; // non weighted
          }
        }
        else if (i == j)
          mat_W(NewVal[i], NewVal[j]) = 0.0; // non weighted put  1.0 instead
      }                                      //end J
  }                                          //end of I

  //    cout << mat_W2 << endl << endl  ; //Print Matrix
  //    cout << mat_W << endl;  cin.get();//Print Matrix
  //    cin.get();
  //Compute the eigenvalue and eigenvectr
  es.compute(mat_W);

  MinEigVal = es.eigenvalues()[0]; // Get min eigenvalue of G matrix
                                   //   MaxEigVal = es.eigenvalues()[instance.DIM -1];

  up = (((double)K - 1.0) / K) * (w_V - ((MinEigVal * counterVer) / 2.0)); // Nikiforov

  //up_SD = ((double)(instance.DIM *(K-1.0))/(2.0*K))*(MaxEigVal); // Satirov

  //  cout << endl << "MinEigVal = " << MinEigVal << ", w_V =" << w_V << "Up = " <<up << endl; cin.get();

  //   cout << endl << "MaxEigVal = " << MaxEigVal << ", ((double)(instance.DIM *(K-1.0))/(2.0*K))=" << ((double)(instance.DIM *(K-1.0))/(2.0*K)) << "Up = " <<up_SD; cin.get();

  //   if (up < up_SD)  return up;
  //  else

  return up; //Nikiforov upper bound

} // end function

///Does not work like I intended
void ImprovePartition_FixMoreVertices(const T_Instance &instance, std::vector<std::vector<int>> &Partitions, const double &bestLB)
{

  bool flag = false; //controls changes in partition
  double LargestVal, secondLrg, sumVal, UtopicUB, ValEdge, LargestDif_ALL;
  int part_j, I, Z;
  std::vector<std::vector<int>> newPartitions;
  newPartitions = Partitions;
  double gap; //= ub - bestLB;

  //Possible include new
  if (newPartitions.size() < K)
  {
    return void();
  }
  else
  {
    for (int j = 0; j < newPartitions.size(); j++)
      if (newPartitions[j].size() == 0)
        return void();
  }

  std::vector<bool> verfVertex(instance.DIM, false);    //easy way to check vertices in partition
  std::vector<bool> NotinPartition(instance.DIM, true); //easy way to check vertices  NOT in partition

  //if vertex is INSIDE partitions (it will have a value true)
  for (unsigned j = 0; j < Partitions.size(); j++)
    for (unsigned jj = 0; jj < Partitions[j].size(); jj++)
    {
      verfVertex[Partitions[j][jj]] = true;
      NotinPartition[Partitions[j][jj]] = false;
    }

  /*calculate the maximum (unrealistic) upper bound */
  /**************************************************/

  /***** First phase: Val of actual partition *****/
  LargestDif_ALL = 0.0;
  UtopicUB = Get_valueOfPartition(instance, Partitions);

  // 	cout << "The correct = " << Get_valueOfPartition (instance,Partitions) << "The calculate = " << UpperBoundSmallestEigenvalue_Partition(instance,verfVertex); cin.get();
  /***** Second phase: Utopic value of edges still not assigned *****/
  UtopicUB += UpperBoundSmallestEigenvalue_Partition(instance, NotinPartition); //Upper bound based on Nikiforov for vertex not in partition

  /***** Third: select best partition and its cost  *****/

  for (int i = 0; i < instance.DIM - 1; i++)
  {
    if (!verfVertex[i])
    {
      I = i + 1;
      //select best partition and its cost
      LargestVal = 0.0;
      for (int j = 0; j < newPartitions.size(); j++)
      {
        sumVal = Val_newVertex_inPartition(instance, Partitions, j, i); // VALUE when vertex i is in partition j

        if (sumVal >= LargestVal)
        {
          secondLrg = LargestVal;
          LargestVal = sumVal;
        }
        else if (sumVal > secondLrg)
          secondLrg = sumVal;
      } //end FOR j

      UtopicUB += LargestVal; // came from best partition to insert vertex i

      if (LargestVal - secondLrg > LargestDif_ALL)
        LargestDif_ALL = LargestVal - secondLrg;
    } //end if I
  }   //end for I

  /* Our Utopic gap */
  gap = UtopicUB - bestLB;

  if (LargestDif_ALL < gap + 0.5) //no need to proceed
    return void();                // end function

  //iterations for each vertice
  for (int i = 0; i < instance.DIM; i++)
  {
    if (!verfVertex[i])
    { //not yet fixed
      LargestVal = 0.0;
      secondLrg = 0.0;
      for (int j = 0; j < Partitions.size(); j++)
      {                                                                 // simulate that vertex i goes to partition j
        sumVal = Val_newVertex_inPartition(instance, Partitions, j, i); // VALUE if vertex i is in partition j

        if (sumVal >= LargestVal)
        { //must be >=
          secondLrg = LargestVal;
          LargestVal = sumVal;
          part_j = j;
        }
        else if (sumVal > secondLrg)
        {
          secondLrg = sumVal; //endd if sumVal
        }                     //end else if
      }                       //end FOR j (newpartitions)

      //if diff fst and second is bigger than gap.. then we must include vertex i in partition part_j
      if ((LargestVal - secondLrg) > gap)
      {
        cout << "gap=" << gap << ", dif=" << (LargestVal - secondLrg);
        cin.get();
        //insert in partition
        newPartitions[part_j].push_back(i);
        verfVertex[i] = true;
        flag = true; // we have at least one change in Partitions
      }
    } //end IF for I in any partition
  }   //end FOR i (vertices)

  //at least one change in newPartitions
  if (flag)
  {
    int bf;
    bf = TotalNbVertices_Partition(Partitions);
    Partitions = newPartitions;
    cout << endl
         << "ImprovePartition_FixMoreVertices has included : (" << TotalNbVertices_Partition(Partitions) - bf << ") Vertices... now it has" << TotalNbVertices_Partition(Partitions) << endl;
    //PrintPartitionICH(Partitions );
    cin.get();
  }

  //cout << endl << "Fin de funcao"; cin.get();
}

inline bool Select_validVertex_BB_zeroAllPartition(T_Instance &instance, const std::vector<std::vector<int>> &Partitions, double *bestLowerBound, int *selectedVertex)
{
  long aux_edge;
  int selectedEdge = -1,
      totalVerPartition = TotalNbVertices_Partition(Partitions);
  std::vector<bool> verfVertex(instance.DIM, false);
  double val = 0.0, lb;

  *selectedVertex = -1;

  // verify number of vertices in Partition and save time if all are in Partitions
  if (totalVerPartition == instance.DIM)
  {
    lb = Get_valueOfPartition(instance, Partitions) + instance.sum_cost;
    if (lb > *bestLowerBound)
    {
      *bestLowerBound = lb;
      //		cout << "foi daki"; cin.get();
    }
    return false;
  }

  //PrintPartitionICH(Partitions ); cin.get();
  //if vertex is INSIDE partitions (it will have a value true)
  for (unsigned j = 0; j < Partitions.size(); j++)
    for (unsigned jj = 0; jj < Partitions[j].size(); jj++)
      verfVertex[Partitions[j][jj]] = true;

  for (int i = 0; i < instance.DIM; i++)
    if (!verfVertex[i])
    {
      val = 0.0;
      for (int j = 0; j < instance.DIM; j++)
        if (verfVertex[j])
        {
          aux_edge = instance.indice_cij[(i + 1) + (j + 1) * (instance.DIM + 1)];
          if (aux_edge != -1)
            val += ValueXij(i + 1, j + 1, instance);

        } //end for J

      if (val == 0.0)
      {
        // 	cout << "chegou with i = " << i << endl ; cin.get();
        *selectedVertex = i;
        return true;
      }

    } //end for I

  //ALl vertices have at least one edge = 1

  return Select_validVertex_BB(instance, Partitions, bestLowerBound, selectedVertex); //do old method
}

inline bool Select_validVertex_BB(T_Instance &instance, const std::vector<std::vector<int>> &Partitions, double *bestLowerBound, int *selectedVertex)
{
  int selectedEdge = -1,
      totalVerPartition = TotalNbVertices_Partition(Partitions);
  std::vector<bool> verfVertex(instance.DIM, false);
  double lb = 0.0;

  *selectedVertex = -1;

  // verify number of vertices in Partition and save time if all are in Partitions
  if (totalVerPartition == instance.DIM)
  {
    lb = Get_valueOfPartition(instance, Partitions) + instance.sum_cost;
    ;
    if (lb > *bestLowerBound)
    {
      *bestLowerBound = lb;
      //		cout << instance.sum_cost << "Veio daki... Select_validVertex_BB ..."  ;
      //		PrintPartitionICH(Partitions ); cin.get();
    }
    return false;
  }

  //PrintPartitionICH(Partitions ); cin.get();
  //if vertex is INSIDE partitions (it will have a value true)
  for (unsigned j = 0; j < Partitions.size(); j++)
    for (unsigned jj = 0; jj < Partitions[j].size(); jj++)
      verfVertex[Partitions[j][jj]] = true;

  selectedEdge = Branch_Rule_ChooseVariable(instance, NULL, &Partitions, selectedVertex);

  if (*selectedVertex != -1)
    return true;

  //   selectedEdge = ChooseEdge_BB (instance); //select an non integer variable from instance solution (from generator branch)
  //    selectedEdge = ChooseEdge_ConsiderPartition_BB(instance, Partitions); // select edge that has at least one vertex fixed

  if (selectedEdge == -1)
  { // all the edges are integer
    //do a iteration of ICH if it is ok than stop
    //if (totalVerPartition >= instance.DIM*0.8)
    if (ISCOMPLETE_GRAPH)
      lb = ICH_Heuristique_Ghaddar_oneIteration(instance); //verify the integer solution (if not ansewez if -1.0) // not good for sparse // can chage it ot true or false (lb as parameter)
    else
      lb = VNS_Heuristic_FeasibleSoltion(instance); // for sparse only instances

    if (lb > *bestLowerBound)
      *bestLowerBound = lb;

    *selectedVertex = Find_firstVertexNotInPartition_BB(verfVertex, instance.DIM);

    if (*selectedVertex == -1) //error
      cout << "Erro in Select_validVertex_BB  in  Find_firstVertexNotinPartition_BB =  *selectedVertex " << *selectedVertex;
  }
  else
  { // find the vertex

    *selectedVertex = instance.cij.barc_i[selectedEdge] - 1; // start by 1 in cost but in Partitions the vertices start by 0
    if (verfVertex[*selectedVertex])
    {
      *selectedVertex = instance.cij.barc_j[selectedEdge] - 1;
      if (verfVertex[*selectedVertex])
      { // may enter here if we are using ImprovePartition_FixMoreVertices
        *selectedVertex = Find_firstVertexNotInPartition_BB(verfVertex, instance.DIM);
        //cout << "val = "<< instance.varS[selectedEdge];
        //cout << "It is possibly an erro because the edge is not integer but its vertices are already fixed "; cin.get();
      }
    } //end IF

  } //end ELSE selected edge

  if (*selectedVertex != -1)
    return true;
  else
  {
    cout << "selectedEdge =" << selectedEdge << endl;
    cout << "Erro in Select_validVertex_BB  for partition bb  Could not find a valid vertex ";
    cin.get();
    return false; //error could not find a vertex (???)
  }
}

bool GetExceptionTriangle(const T_Instance &instance, const int &i, const int &j, const int &h)
{
  long aux_edge;

  aux_edge = instance.indice_cij[(i + 1) + (j + 1) * (instance.DIM + 1)];
  if (aux_edge == -1)
    return true;
  aux_edge = instance.indice_cij[(i + 1) + (h + 1) * (instance.DIM + 1)];
  if (aux_edge == -1)
    return true;
  aux_edge = instance.indice_cij[(h + 1) + (j + 1) * (instance.DIM + 1)];
  if (aux_edge == -1)
    return true;

  return false;
}

double ICH_Heuristique_Ghaddar_oneIteration(const T_Instance &instance)
{
  //printSolution_screen(instance);
  bool CONTINUE;
  int counter, bestVertex, I, H, J;
  double sumVal, bestSumVal;
  std::vector<std::vector<int>> Partitions;
  std::vector<int> vecVertex;
  std::set<ICH_Var> setICHvar;
  std::set<ICH_Var>::iterator it;
  double epslonICH = 3; //Only Integer solution

  vecVertex.resize(instance.DIM, -1);

  setICHvar.clear();
  //   cout << "instance.DIM = " << instance.DIM; cin.get();
  for (unsigned i = 0; i < instance.DIM - 2; i++)
    for (unsigned j = i + 1; j < instance.DIM - 1; j++)
      for (unsigned h = j + 1; h < instance.DIM; h++)
        if (!GetExceptionTriangle(instance, i, j, h))
        {
          //	  cout << i << j << h; cin.get();
          sumVal = ValueXij(i + 1, j + 1, instance);
          sumVal += ValueXij(i + 1, h + 1, instance);
          sumVal += ValueXij(h + 1, j + 1, instance);

          if (sumVal < epslonICH)
            continue; // exit this iteration

          I = i;
          J = j;
          H = h;

          if (vecVertex[I] == -1 || vecVertex[J] == -1 || vecVertex[H] == -1)
          {
            if (vecVertex[I] == -1 && vecVertex[J] == -1 && vecVertex[H] == -1)
            {
              std::vector<int> newPartition;
              newPartition.push_back(I);
              newPartition.push_back(J);
              newPartition.push_back(H);
              Partitions.push_back(newPartition);
              vecVertex[I] = vecVertex[J] = vecVertex[H] = Partitions.size() - 1; //
            }
            else if (vecVertex[I] == vecVertex[J])
            {
              if (vecVertex[I] == -1)
              {
                Partitions[vecVertex[H]].push_back(I);
                Partitions[vecVertex[H]].push_back(J);
                vecVertex[I] = vecVertex[J] = vecVertex[H];
              }
              else
              {
                Partitions[vecVertex[I]].push_back(H);
                vecVertex[H] = vecVertex[I];
              }
            }
            else if (vecVertex[I] == vecVertex[H])
            {
              if (vecVertex[I] == -1)
              {
                Partitions[vecVertex[J]].push_back(I);
                Partitions[vecVertex[J]].push_back(H);
                vecVertex[I] = vecVertex[H] = vecVertex[J];
              }
              else
              {
                Partitions[vecVertex[I]].push_back(J);
                vecVertex[J] = vecVertex[I];
              }
            }
            else if (vecVertex[J] == vecVertex[H])
            {
              if (vecVertex[J] == -1)
              {
                Partitions[vecVertex[I]].push_back(J);
                Partitions[vecVertex[I]].push_back(H);
                vecVertex[J] = vecVertex[H] = vecVertex[I];
              }
              else
              {
                Partitions[vecVertex[J]].push_back(I);
                vecVertex[I] = vecVertex[J];
              }
            }
          }
          else
          { //end of main if
            //All the three vertex are in a partition
            if (vecVertex[I] != vecVertex[J] || vecVertex[J] != vecVertex[H] || vecVertex[H] != vecVertex[I]) // not a validy integer solution
              return -1.0;                                                                                    //end function
          }
        } //end FOR I, J, H

  for (unsigned i = 0; i < instance.DIM; i++)
  {
    if (vecVertex[i] == -1)
    {
      std::vector<int> newPartition;
      newPartition.push_back(i);
      Partitions.push_back(newPartition);
    }
  }

  if ((Partitions.size() > K) && (Partitions.size() <= instance.DIM))
  {

    return -1.0;
  }
  else
  { // if
    /* NO need to check. Because it will be checked if lower bound is equal to upper bound in function Select_validVertex_BB (above)
	//check all zeros are ok in partition
		for (unsigned i=0; i < instance.DIM-1; i++)
    			for (unsigned j= i+1; j <instance.DIM; j++){
				sumVal = ValueXij(i+1,j+1,instance);
				if (sumVal == 0.0 && vecVertex[i] == vecVertex[j]) //not validy integer solution
					return -1.0;
		}//end i and j
	//end of vefirication (IT is an integer solution)
	*/

    if (Partitions.size() < instance.DIM)
    { //it is a final feasible solution
      LocalSearch_Solution(instance, Partitions);
      return Get_valueOfPartition(instance, Partitions) + instance.sum_cost;
    }
  }
  return -1.0;
} //end function

inline int TotalNbVertices_Partition(const std::vector<std::vector<int>> &Partitions)
{
  int counter = 0;
  for (int i = 0; i < Partitions.size(); i++)
    for (int ii = 0; ii < Partitions[i].size(); ii++)
      counter++;

  return counter;
}

inline int Find_firstVertexNotInPartition_BB(const std::vector<bool> &verfVertex, const int &size)
{
  for (unsigned i = 0; i < size; i++)
    if (verfVertex[i] == false)
      return i;

  return -1;
}
void CreatNewCandidates_FixVariable_BB(std::set<T_Branch> &ListBB, T_Instance &instance, std::vector<T_fixVar> &fixvar, double &bestLB)
{
  long selectedEdge;
  double Lb;

  if (TYPE_SOLVER_BB != TYPE_SDP_BB)
    Fix_reduced_cost_dichotomic(instance, &fixvar, bestLB);

  selectedEdge = Branch_Rule_ChooseVariable(instance, &fixvar, NULL, NULL);

  if (selectedEdge == -1)
  {
    Lb = ICH_Heuristique_Ghaddar_oneIteration(instance);
    if (Lb >= bestLB)
    {
      bestLB = Lb;
    }
    if (Lb <= instance.ObSol - 0.99)
    {
      selectedEdge = Pick_FirstEdgeAvailable(instance, fixvar);
      //		    cout <<"All variables are INTEGER ... see triangle and clique violation for determine another way of fixing"; cin.get();
    }
  }

  if (selectedEdge != -1)
  {
    Creat_TwoCandidates_BB(ListBB, instance, fixvar, bestLB, selectedEdge);
  }
}

void Fix_reduced_cost_dichotomic(T_Instance &instance, std::vector<T_fixVar> *fixvar, const double &bestLowerBound)
{

  T_fixVar newFixVar;
  long aux_edge;
  double val_e,
      gap = instance.ObSol - (bestLowerBound + 1.0); //var with all the reduced cost of our problem
                                                     //   redcost =  (double*) calloc(instance.totalVars,sizeof(double)); //Reduced cost variable  *redcost,

  //Get Reduced cost
  //	MSK_getreducedcosts(task_BB_2, MSK_SOL_BAS,0, instance.totalVars,redcost_BB );

  int counter = 0;
  //	cout << "gap = " << gap;
  //fixing a vertex a partition by reduced cost
  for (int i = 0; i < instance.DIM; ++i)
    for (int j = i + 1; j < instance.DIM; ++j)
    {
      aux_edge = instance.indice_cij[(i + 1) + (j + 1) * (instance.DIM + 1)];
      if (IsValidEdge(aux_edge, fixvar))
      {
        val_e = instance.varS[aux_edge];
        //				cout << "val_e = " << val_e;
        //				cout << " redcost[aux_edge]  = " << redcost[aux_edge] ;  cin.get();
        if (val_e <= epslon)
        { //it means that value is zero
          if (-(redcost_BB[aux_edge]) > gap)
          {
            newFixVar.var = aux_edge;
            newFixVar.val = 0.0;
            (*fixvar).push_back(newFixVar);
            counter++;
            fixMoreVar_Dichtomic_triangle(instance, fixvar, (*fixvar)[(*fixvar).size() - 1]);
          }
        }
        else if (val_e >= 1.0 - epslon)
        { //val is 1.0
          if ((redcost_BB[aux_edge]) > gap)
          {
            newFixVar.var = aux_edge;
            newFixVar.val = 1.0;
            (*fixvar).push_back(newFixVar);
            counter++;
            fixMoreVar_Dichtomic_triangle(instance, fixvar, (*fixvar)[(*fixvar).size() - 1]);
          }
        }
      } //end if valid
    }   //end for i and j

  //		if (counter > 0){
  //			cout << "It fixed >>>" << counter ++;
  //			cin.get();
  //		}
  //FixVarible_to_ONE_RCisBig_BB
} //end of function

inline long Pick_FirstEdgeAvailable(const T_Instance &instance, const std::vector<T_fixVar> &fixvar)
{
  long aux_edge;
  for (int i = 0; i < instance.DIM; ++i)
    for (int j = i + 1; j < instance.DIM; ++j)
    {
      aux_edge = instance.indice_cij[(i + 1) + (j + 1) * (instance.DIM + 1)];
      if (IsValidEdge(aux_edge, &fixvar))
        return aux_edge;
    }
  return -1;
}

void Creat_TwoCandidates_BB(std::set<T_Branch> &ListBB, T_Instance &instance, const std::vector<T_fixVar> &fixvarOrig, double &bestLB, const double &selectedEdge)
{
  std::vector<T_fixVar> new_vecfixvar;
  T_fixVar fixVar;
  T_Branch newBranch;
  bool R;

  new_vecfixvar = fixvarOrig;
  new_vecfixvar.push_back(fixVar);
  new_vecfixvar[new_vecfixvar.size() - 1].var = selectedEdge;

  newBranch.lowerboud = bestLB;
  newBranch.upperbound = instance.ObSol;
  newBranch.Level_tree = seqqq_BB;

  R = Strategy_selecting_ActiveNodeTree_BB(instance, &newBranch.seq);

  newBranch.seq = -instance.ObSol;
  newBranch.ptxInst = &instance;
  newBranch.lastVarFix = selectedEdge;
  newBranch.ExtraCONST = ExtraCONST_BB;

  int InitialsizeFix = new_vecfixvar.size();

  for (int i = 0; i < 2; i++)
  {
    new_vecfixvar.resize(InitialsizeFix); //useful for fixMoreVar_Dichtomic_triangle

    if (i == 0)
      new_vecfixvar[new_vecfixvar.size() - 1].val = 0.0;
    else
      new_vecfixvar[new_vecfixvar.size() - 1].val = 1.0;
    newBranch.lastPartFix = i;
    //fix more variables (based on triange inequality )
    fixMoreVar_Dichtomic_triangle(instance, &new_vecfixvar, new_vecfixvar[new_vecfixvar.size() - 1]);

    if (STRATEGY_SOL_BB == EAGER_BB)
    {
      R = Solve_SubProblem_BB(instance, NULL, new_vecfixvar, false, TYPE_SOLVER_BB, false, &bestLB);
      if (R == true && instance.ObSol > bestLB + 0.99)
      {
        newBranch.fixvar = new_vecfixvar;
        newBranch.upperbound = instance.ObSol;
        newBranch.seq = -instance.ObSol;
        ListBB.insert(newBranch);
      }
    }
    else
    { // lazy_bb
      newBranch.fixvar = new_vecfixvar;
      ListBB.insert(newBranch);
      //       cout << "ListBB size = " << ListBB.size();cout <<"Entered"; cin.get();
    } //end of ifelse
  }
}

//Stategy how to rank
inline bool Strategy_selecting_ActiveNodeTree_BB(const T_Instance &instance, double *val_Change)
{
  switch (SELEC_STRATEGY_BB)
  {
  case (BeFS): //Best first
    *val_Change = -instance.ObSol;
    break;
  case (WFS): //Worse first
    *val_Change = instance.ObSol;
    break;
  case (BFS): //Breath first
    *val_Change = seqqq_BB;
    break;
  case (DFS): // depth rst search
    *val_Change = -seqqq_BB;
    break;
  default: //Defaut
    *val_Change = -instance.ObSol;
    break;
  } //end switch

  return true;
} //end function selecting strategy

inline long ChooseEdge_ConsiderPartition_BB(const T_Instance &instance, const std::vector<std::vector<int>> &Partitions)
{

  return ChooseEdge_by_LargestSmallest_Cijweight_BB(instance, -1, Partitions); // Select by edge weight (for smallest -1, for largest par= 1)
}

//choose non integer edge from instance.VarS (solution) if possible
inline long ChooseEdge_BB(const T_Instance &instance)
{

  //    return ChooseByFirstViolated_BB(instance);

  return ChooseEdge_by_LargestSmallest_Cijweight_BB(instance, -1); // Select by edge weight (for smallest -1, for largest par= 1)

  //return ChooseEdge_by_NearestToFixVal_BB (instance, 0.0); // (Accepted vals [0.0 ... 1.0])select by edge violation (e.g., for closest to 0.0  set last parameter to 0.0)
}

inline long Branch_Rule_ChooseVariable(T_Instance &instance, std::vector<T_fixVar> *fixvar, const std::vector<std::vector<int>> *Partitions, int *Vertex)
{

  //Branch rule
  //#define  R1 = 1, // Most decided first
  //#define  R2 = 2, //Best in MG
  //#define  R3 = 3; //Last decided first
  if (Vertex != NULL)
    *Vertex = -1;
  //int  = R1;
  if (Partitions != NULL)
  {
    switch (BRANCHING_RULE_BB)
    {
    case (R1):                                                                  // Most decided first
      return ChooseEdge_by_NearestToFixVal_BB(instance, 0.0, *Partitions);      // (Accepted vals [0.0 ... 1.0])select by edge violation (e.g., for closest to 0.0  set last parameter to 0.0)
    case (R2):                                                                  // least decided first
      *Vertex = ChooseVertex_by_ClosestFeasible_BB(instance, 0.5, *Partitions); //Closest to be feasible
      return -1;
    case (R3):                                                             // least decided first
      return ChooseEdge_by_NearestToFixVal_BB(instance, 0.5, *Partitions); // (Accepted vals [0.0 ... 1.0])select by edge violation (e.g., for closest to 0.0  set last parameter to 0.0)
    case (R5):                                                             //Edge weight
      *Vertex = ChooseVertex_by_Weight_Partition_BB(instance, *Partitions);
      return -1;
    case (R6):
      *Vertex = ChooseVertexStrongBranching_by_Partition_BB(instance, *Partitions);
      return -1;
    case (R7):
      *Vertex = ChooseVertexPseudoCost_by_Partition_BB(instance, *Partitions, &PseudoCcost);
      return -1;
    default:
      return ChooseEdge_by_NearestToFixVal_BB(instance, 0.5, *Partitions); //
    }
  }
  else
  {
    switch (BRANCHING_RULE_BB)
    {
    case (R1): // Most decided first
      return ChooseEdge_by_NearestToFixVal_BB(instance, 0.0);
    case (R2):
      return ChooseEdge_by_ClosestFeasible_BB(instance, 0.5, fixvar); //Closest to be feasible
    case (R3):                                                        // LEAST decided first
      return ChooseEdge_by_NearestToFixVal_BB(instance, 0.5);
    case (R5):
      return ChooseEdge_by_LargestSmallest_Cijweight_BB(instance, 1); // Select by edge weight (for smallest -1, for largest par= 1)
    case (R6):
      return ChooseEdge_StrongBranching_BB(instance, fixvar);
    case (R7):
      return ChooseEdge_PseudoCost_BB(instance, fixvar, &PseudoCcost);
    default:
      return ChooseEdge_by_ClosestFeasible_BB(instance, 0.5, fixvar); //Closest to be feasible
    }
  }
}

//return true if val is binary (0, 1) with epslon_val tolerance
inline bool IsBinary(const double &val)
{
  double epslon_val = 0.0001;

  if (val >= 1.0 - epslon_val || (val <= 0.0 + epslon_val && val >= 0.0 - epslon_val))
    return true;
  else
    return false;
}

inline long ChooseEdge_by_NearestToFixVal_BB(const T_Instance &instance, const double &fixVal)
{
  double val, bestVal = 2.0; //bid double
  int bestEdge = -1;

  int flag = 0;

  for (long i = 0; i < instance.edge_nb; i++)
  {
    if (!IsBinary(instance.varS[i]))
    {
      val = instance.varS[i] - fixVal; //Set is ordered by small to large
      if (val < 0.0)                   // set just positive values
        val *= -1.0;
      if (val < bestVal)
      { // not take the ones that cost 0
        bestVal = val;
        bestEdge = i;
      }
    } //end IF
  }

  return bestEdge;
}
//See reference of  R2 in article Solving k-Way of Miguel F. Anjos,
// i = argmin sum((1 −m)−|Xi_r − m|)^2
inline int ChooseVertex_by_ClosestFeasible_BB(const T_Instance &instance, const double &fixVal, const std::vector<std::vector<int>> &Partitions)
{
  long aux_edge;
  int bestVertex = -1, I, J;
  double val_i, auxV, best_val = 10000.0; // big double
  std::vector<bool> AllowVertices_easy(instance.DIM, true);

  for (int i = 0; i < Partitions.size(); i++)
    for (int ii = 0; ii < Partitions[i].size(); ii++)
      AllowVertices_easy[Partitions[i][ii]] = false;

  //	   PrintPartitionICH(Partitions);
  for (int i = 0; i < instance.DIM; ++i)
    if (AllowVertices_easy[i])
    {
      val_i = 0.0;
      I = i + 1;
      for (int j = 0; j < Partitions.size(); ++j)
        for (int jj = 0; jj < Partitions[j].size(); ++jj)
        {
          J = Partitions[j][jj] + 1;
          //		for (int j=0; j<instance.DIM; ++j) if (j != i)
          //			J = j+1;
          aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
          if (aux_edge != -1)
          {
            auxV = instance.varS[aux_edge] - fixVal;
            if (auxV < 0.0)
              auxV *= -1.0;
            val_i += pow((1.0 - fixVal) - auxV, 2);
            //				cout << "auxV=" << auxV << ", val_i= " << val_i; cin.get();
          } //end IF aux_edge ok
        }
      if (val_i < best_val && val_i > 0.0)
      {
        bestVertex = i;
        best_val = val_i;
      }
    } //end FOR i
      //  cout << "best_val=" << best_val << ", bestVertex= " << bestVertex; cin.get();
  return bestVertex;
}

//Similar to ChooseVertex_by_ClosestFeasible_BB but for edge
inline long ChooseEdge_by_ClosestFeasible_BB(const T_Instance &instance, const double &fixVal, const std::vector<T_fixVar> *fixvar)
{
  long aux_edge;
  int bestVertex_i = 0, I, J, bestVertex_j = 0;
  double val_i, val_j, auxV, best_val = 10000.0; // big double

  //	   PrintPartitionICH(Partitions);
  //find first
  for (int i = 0; i < instance.DIM; ++i)
  {
    val_i = 0.0;
    I = i + 1;
    for (int j = 0; j < instance.DIM; ++j)
    {
      J = j + 1;
      aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
      if (IsValidEdge(aux_edge, fixvar))
      {
        auxV = instance.varS[aux_edge] - fixVal;
        if (auxV < 0.0)
          auxV *= -1.0;
        val_i += pow((1.0 - fixVal) - auxV, 2);
        //				cout << "auxV=" << auxV << ", val_i= " << val_i; cin.get();
      } //end IF aux_edge ok
    }
    if (val_i < best_val && val_i > epslon)
    {
      bestVertex_i = i;
      best_val = val_i;
    }
  } //end FOR i

  best_val = 10000.0;
  //find second vertex
  for (int j = 0; j < instance.DIM; ++j)
    if (j != bestVertex_i)
    {
      val_j = 0.0;
      I = j + 1;
      aux_edge = instance.indice_cij[(bestVertex_i + 1) + I * (instance.DIM + 1)];
      if (IsValidEdge(aux_edge, fixvar))
      {
        for (int z = 0; z < instance.DIM; ++z)
        {
          J = z + 1;
          aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
          if (IsValidEdge(aux_edge, fixvar))
          {
            auxV = instance.varS[aux_edge] - fixVal;
            if (auxV < 0.0)
              auxV *= -1.0;
            val_j += pow((1.0 - fixVal) - auxV, 2);
            //				cout << "auxV=" << auxV << ", val_i= " << val_i; cin.get();
          } //end IF aux_edge ok
        }
        if (val_i < best_val && val_i > epslon)
        {
          bestVertex_j = j;
          best_val = val_j;
        }
      } //end IF valid edge between best_i and j
    }   //end FOR ij
        //  cout << "best_val=" << best_val << ", bestVertex= " << bestVertex; cin.get();

  //return best edge
  I = bestVertex_i + 1;
  J = bestVertex_j + 1;
  return instance.indice_cij[I + J * (instance.DIM + 1)];
}

inline bool IsValidEdge(const long &aux_edge, const std::vector<T_fixVar> *fixvar)
{
  if (aux_edge == -1)
    return false;

  if (fixvar != NULL)
  {
    for (unsigned i = 0; i < (*fixvar).size(); i++)
    {
      if (aux_edge == (*fixvar)[i].var)
        return false;
    }
  }

  return true;
}

inline long ChooseEdge_by_NearestToFixVal_BB(const T_Instance &instance, const double &fixVal, const std::vector<std::vector<int>> &Partitions)
{
  //  cout << "entroou" <<endl;
  double val, bestVal = 2.0; //bid double
  bool b_i, b_j;
  long bestEdge = -1;

  std::vector<bool> AllowVertices_easy(instance.DIM, false);

  //   PrintPartitionICH(Partitions);
  for (int i = 0; i < Partitions.size(); i++)
    for (int ii = 0; ii < Partitions[i].size(); ii++)
      AllowVertices_easy[Partitions[i][ii]] = true;

  int flag = 0;
  for (long i = 0; i < instance.edge_nb; i++)
  {
    //     if ( !IsBinary(instance.varS[i]))
    b_j = AllowVertices_easy[instance.cij.barc_j[i] - 1];
    b_i = AllowVertices_easy[instance.cij.barc_i[i] - 1];

    if ((b_j || b_i) && !(b_j && b_i))
    {                                  // one of the vertices should be already fixed
      val = instance.varS[i] - fixVal; //Set is ordered by small to large
      if (val < 0.0)                   // set just positive values
        val *= -1.0;
      if (val < bestVal)
      { // not take the ones that cost 0
        bestVal = val;
        bestEdge = i;
      }
    }
  } //end For
  //	cout << instance.cij.barc_j[bestEdge]-1 << "," << instance.cij.barc_i[bestEdge]-1 <<"val =" << bestVal; cin.get();

  return bestEdge;
}

//Look article (Branching rules revisited of Tobias Achterberg) it is thier Realibility algorithm
inline int ChooseVertexPseudoCost_by_Partition_BB(T_Instance &instance, const std::vector<std::vector<int>> &Partitions_origem,
                                                  std::vector<T_PseudCost_BB> *PseudoCcost)
{
  bool R = true, RESUME = true, flag, solve = false;
  int Nb_reliab = 4,
      lookAhead = 6,
      counter = 0,
      vertex, bestVert = -1;
  double val_i,
      Mu = 1.0 / 6.0,
      MaxScore = 0.0;
  vector<bool> validVertex(instance.DIM, true);
  if ((SDP_SEP == -1) || (SDP_SEP == -3))
    TYPE_SOLVER_BB = TYPE_SDP_BB;
  else
    TYPE_SOLVER_BB = TYPE_SIMPLEX_BB; //  TYPE_IPM_BB

  bool DO_NEW_TASK = false;
  if (TYPE_BRANCH == BnC)
    DO_NEW_TASK = true;

  std::vector<std::vector<int>> Partitions;
  Partitions = Partitions_origem;

  std::vector<T_fixVar> fixvar;

  double LocalUb = instance.ObSol;

  double fake_Lb = 0.0;

  for (int p = 0; p < Partitions.size(); ++p)
    for (int pp = 0; pp < Partitions[p].size(); ++pp)
      validVertex[Partitions[p][pp]] = false;

  for (int i = 0; i < instance.DIM && RESUME; ++i)
  {
    flag = true;
    vertex = FindBestVertex_score(validVertex, *PseudoCcost, Mu);

    if (vertex == -1)
      break; //out of for ... cannot find any vertex

    if ((*PseudoCcost)[vertex].MinNbAccess() < Nb_reliab)
    {
      for (int p = 0; p < Partitions.size(); ++p)
      {
        solve = true;
        Partitions[p].push_back(vertex);
        RESUME = Solve_SubProblem_BB(instance, &Partitions, fixvar, true, TYPE_SOLVER_BB, DO_NEW_TASK, &fake_Lb);
        Partitions[p].pop_back();
        if (RESUME)
        {
          Set_NewValPseudoCost(LocalUb, instance.ObSol, vertex, p, PseudoCcost);
        }
      }
      flag = false;
    }
    val_i = (*PseudoCcost)[vertex].Get_Score(Mu);
    validVertex[vertex] = false;

    if (val_i > MaxScore)
    {
      MaxScore = val_i;
      bestVert = vertex;
    }
    else
    {
      counter++;
      if (counter >= lookAhead)
        flag = true;
    }

    if (flag == true)
      RESUME = false;
  }

  if (solve == true)
    RESUME = Solve_SubProblem_BB(instance, &Partitions, fixvar, true, TYPE_SOLVER_BB, DO_NEW_TASK, &fake_Lb);

  return bestVert;
} //end function

long ChooseEdge_PseudoCost_BB(T_Instance &instance, std::vector<T_fixVar> *fixvar, std::vector<T_PseudCost_BB> *PseudoCcost)
{
  T_fixVar newVar;
  set<int> UsedEdges;
  for (int i = 0; i < (*fixvar).size(); ++i)
    UsedEdges.insert((*fixvar)[i].var);

  bool R = true, RESUME = true, flag;
  int Nb_reliab = 4,
      lookAhead = 8,
      counter = 0,
      edge, bestEdge = -1;
  double val_i,
      Mu = 1.0 / 6.0,
      MaxScore = 0.0;
  vector<bool> validVertex(instance.DIM, true);
  if ((SDP_SEP == -1) || (SDP_SEP == -3))
    TYPE_SOLVER_BB = TYPE_SDP_BB;
  else
    TYPE_SOLVER_BB = TYPE_SIMPLEX_BB; //  TYPE_IPM_BB

  double LocalUb = instance.ObSol;
  double fake_Lb = 0.0;

  bool DO_NEW_TASK = false;
  if (TYPE_BRANCH == BnC)
    DO_NEW_TASK = true;

  for (int i = 0; i < instance.edge_nb && RESUME; ++i)
  {
    flag = true;
    edge = FindBestEdge_score(UsedEdges, *PseudoCcost, Mu);
    val_i = (*PseudoCcost)[edge].Get_Score(Mu);
    //		cout <<edge <<  ", Before , val_i =" << val_i <<endl;
    if (edge == -1)
      break; //out of for ... cannot find any vertex
    if ((*PseudoCcost)[edge].MinNbAccess() < Nb_reliab)
    {
      for (int p = 0; p < 2; ++p)
      {
        newVar.var = edge;
        newVar.val = p; //value is {0, 1}
        (*fixvar).push_back(newVar);
        RESUME = Solve_SubProblem_BB(instance, NULL, *fixvar, false, TYPE_SOLVER_BB, DO_NEW_TASK, &fake_Lb);
        (*fixvar).pop_back();
        if (RESUME)
          Set_NewValPseudoCost(LocalUb, instance.ObSol, edge, p, PseudoCcost);
      }
      flag = false;
    }
    val_i = (*PseudoCcost)[edge].Get_Score(Mu);
    //		cout << "After , val_i =" << val_i <<endl; cin.get();
    UsedEdges.insert(edge);

    if (val_i > MaxScore)
    {
      MaxScore = val_i;
      bestEdge = edge;
    }
    else
    {
      counter++;
      if (counter >= lookAhead)
        flag = true;
    }
    if (flag == true)
      RESUME = false;
  }

  RESUME = Solve_SubProblem_BB(instance, NULL, *fixvar, false, TYPE_SOLVER_BB, DO_NEW_TASK, &fake_Lb);
  return bestEdge;

} //end function

long FindBestEdge_score(std::set<int> UsedEdges, const std::vector<T_PseudCost_BB> &PseudoCcost, const double &Mu)
{
  double valMax = 0.0;
  double val;
  long vetx = -1;
  for (long i = 0; i < PseudoCcost.size(); ++i)
    if (UsedEdges.count(i) == 0)
    {
      val = PseudoCcost[i].Get_Score(Mu);
      if (val >= valMax)
      {
        valMax = val;
        vetx = i;
      }
    }

  return vetx;
}
int FindBestVertex_score(const vector<bool> &validVertex, const std::vector<T_PseudCost_BB> &PseudoCcost, const double &Mu)
{
  double valMax = -epslon_s;
  double val;
  int vetx = -1;
  for (int i = 0; i < PseudoCcost.size(); ++i)
    if (validVertex[i])
    {
      val = PseudoCcost[i].Get_Score(Mu);
      if (val >= valMax)
      {
        valMax = val;
        vetx = i;
      }
    }

  return vetx;
}

int Find_Min_ArgMax_avg(const vector<bool> &validVertex, const std::vector<T_PseudCost_BB> &PseudoCcost)
{
  double valMin = 1000000.0;
  double val;
  int vetx = -1;
  for (int i = 0; i < PseudoCcost.size(); i++)
    if (validVertex[i])
    {
      val = PseudoCcost[i].Max_avg();
      if (val < valMin)
      {
        valMin = val;
        vetx = i;
      }
    }

  return vetx;
}
//strong brachin
inline int ChooseVertexStrongBranching_by_Partition_BB(const T_Instance &instance, const std::vector<std::vector<int>> &Partitions_orig)
{
  bool RESUME = true;
  std::vector<bool> AllowVertices_easy(instance.DIM, true);
  std::vector<std::vector<int>> Partitions;
  Partitions = Partitions_orig;

  int best_vertex = -1, lookAhead = 8, counter = 0;
  double min_All_val = 2 * instance.ObSol,
         val, max_val;
  //	PrintPartitionICH(Partitions); cin.get();
  for (int p = 0; p < Partitions.size(); ++p)
    for (int pp = 0; pp < Partitions[p].size(); ++pp)
      AllowVertices_easy[Partitions[p][pp]] = false;

  for (int i = 0; i < instance.DIM && RESUME; ++i)
    if (AllowVertices_easy[i])
    {
      max_val = 0.0;
      for (int part = 0; part < Partitions.size(); ++part)
      {
        Partitions[part].push_back(i);
        val = Calculate_BoundNikiforov(instance, Partitions);
        Partitions[part].pop_back();
        if (val > max_val)
        {
          max_val = val;
        }
      } // end FOR partitions

      if (max_val < min_All_val)
      {
        min_All_val = max_val;
        best_vertex = i;
      }
      else
      {
        counter++;
        if (counter >= lookAhead)
          RESUME = false;
      }

    } //end for I

  return best_vertex;
} //END FUNCTINO strong branching

//Strong branching by edge (similar to previous function but for edges )
long ChooseEdge_StrongBranching_BB(T_Instance &instance, std::vector<T_fixVar> *fixvar)
{
  T_fixVar newVar;
  bool RESUME = true;
  int best_vertex = -1, lookAhead = 4, counter = 0;
  double min_All_val = 2 * instance.ObSol,
         val, max_val;

  set<int> UsedEdges;
  for (int i = 0; i < (*fixvar).size(); ++i)
    UsedEdges.insert((*fixvar)[i].var);

  double fake_Lb = 0.0;

  if ((SDP_SEP == -1) || (SDP_SEP == -3))
    TYPE_SOLVER_BB = TYPE_SDP_BB;
  else
    TYPE_SOLVER_BB = TYPE_SIMPLEX_BB; //  TYPE_IPM_BB

  //	   PrintPartitionICH(Partitions);
  //find first
  for (int i = 0; i < instance.edge_nb && RESUME; ++i)
  {
    if (UsedEdges.count(i) == 0)
    {

      max_val = 0.0;
      for (int v = 0; v < 2; ++v)
      {
        newVar.var = i;
        newVar.val = v; //value is {0, 1}
        (*fixvar).push_back(newVar);
        RESUME = Solve_SubProblem_BB(instance, NULL, *fixvar, false, TYPE_SOLVER_BB, false, &fake_Lb);
        val = instance.ObSol;
        (*fixvar).pop_back();
        if (val > max_val)
        {
          max_val = val;
        }
      } //end FOR v

      if (max_val < min_All_val)
      {
        //				counter = 0;
        min_All_val = max_val;
        best_vertex = i;
      }
      else
      {
        counter++;
        if (counter >= lookAhead)
          RESUME = false;
      }
    }
  } //end FOR i

  //Solve again to change  instance Objective value to original (important in lazy_strategy )
  RESUME = Solve_SubProblem_BB(instance, NULL, *fixvar, false, TYPE_SOLVER_BB, false, &fake_Lb);

  return best_vertex;
}
//Choose the vertex that have the highest sum of weight with vertex already in partition
inline int ChooseVertex_by_Weight_Partition_BB(const T_Instance &instance, const std::vector<std::vector<int>> &Partitions)
{
  long aux_edge;
  int bestVertex = -1, I, J;
  double val_i, auxV, best_val = 0.0; // big double

  std::vector<bool> AllowVertices_easy(instance.DIM, true);

  for (int i = 0; i < Partitions.size(); i++)
    for (int ii = 0; ii < Partitions[i].size(); ii++)
      AllowVertices_easy[Partitions[i][ii]] = false;

  //   PrintPartitionICH(Partitions);
  for (int i = 0; i < instance.DIM; ++i)
    if (AllowVertices_easy[i])
    {
      val_i = 0.0;
      I = i + 1;
      for (int j = 0; j < Partitions.size(); ++j)
        for (int jj = 0; jj < Partitions[j].size(); ++jj)
        {
          J = Partitions[j][jj] + 1;
          aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
          if (aux_edge != -1)
          {
            auxV = instance.cij.barc_v[aux_edge];
            if (auxV < 0)
              auxV *= -1.0; //Just module
            val_i += auxV;
          }
        }

      if (val_i > best_val)
      {
        bestVertex = i;
        best_val = val_i;
      }

    } //end FOR i

  return bestVertex;

} //end function

inline long ChooseEdge_by_LargestSmallest_Cijweight_BB(const T_Instance &instance, const int &mult, const std::vector<std::vector<int>> &Partitions)
{
  long aux_edge;
  int bestedge = -1;
  double val_i, auxV, best_val = 0.0; // big double
  bool b_i, b_j;
  std::vector<bool> AllowVertices_easy(instance.DIM, false);

  //   PrintPartitionICH(Partitions);

  for (int i = 0; i < Partitions.size(); i++)
    for (int ii = 0; ii < Partitions[i].size(); ii++)
      AllowVertices_easy[Partitions[i][ii]] = true;

  for (long i = 0; i < instance.edge_nb; i++)
  {
    if (!IsBinary(instance.varS[i]))
    { //add allowed vertices
      b_j = AllowVertices_easy[instance.cij.barc_j[i] - 1];
      b_i = AllowVertices_easy[instance.cij.barc_i[i] - 1];
      if (b_j || b_i) // one of the vertices should be already fixed
        if (-mult * instance.cij.barc_v[i] < best_val)
        { // analyze if is the best value
          // 	    cout << "v-1=" << instance.cij.barc_j[i]-1 << ", v-2= " << instance.cij.barc_i[i]-1;
          best_val = -mult * instance.cij.barc_v[i];
          bestedge = i;
          // 	      cout << ", best_val = " << best_val; cin.get();
        } //end if if
    }     // end IF !Bynary
  }       // end for edges

  return bestedge;
}

inline long ChooseEdge_by_LargestSmallest_Cijweight_BB(const T_Instance &instance, const int &mult)
{

  int bestedge = -1;
  double best_val = 1000000.0; // big double

  for (long i = 0; i < instance.edge_nb; i++)
  {
    if (!IsBinary(instance.varS[i]))
      if (-mult * instance.cij.barc_v[i] < best_val)
      {
        best_val = -mult * instance.cij.barc_v[i];
        bestedge = i;
      }
  } // end for edge iteration

  return bestedge;

} //end function

//simpliest way to select a non-integer variable
inline long ChooseByFirstViolated_BB(const T_Instance &instance)
{
  //simple way
  for (int i = 0; i < instance.edge_nb; i++)
  {
    if (!IsBinary(instance.varS[i]))
    {
      return i;
    } //end IF
  }
  return -1; //all variables are integer
}

bool SolveMosek_LP_for_Branch(T_Instance &instance, MSKrescodee &r, MSKtask_t &task, MSKenv_t &env, const int TYPE, const std::vector<T_fixVar> &fixvar, const bool &NEW_TASK)
{

  double *xx;
  double *Obj;

  MSK_deleteenv(&env);

  // Create the mosek environment.
  r = MSK_makeenv(&env, NULL);

  if (r == MSK_RES_OK)
  {
    if (NEW_TASK)
    {

      //MSK_deletetask (&task);

      // Create the optimization task.
      r = MSK_maketask(env, (MSKint32t)instance.CONST.size(), (MSKint32t)instance.totalVars, &task);

      // Append 'numvar' variables.
      //The variables will initially be fixed at zero (x=0).
      r = MSK_appendvars(task, (MSKint32t)instance.totalVars);

      //Append obj function and variables bounds (0 <=x<=1)
      setObjFunction_andVarBounds__LPmosek(instance, r, task, env);

      //set bounds and input row of constraint
      setConstraint_LPmosek(instance, r, task);

      /* Maximize objective function. */
      if (r == MSK_RES_OK)
        r = MSK_putobjsense(task, MSK_OBJECTIVE_SENSE_MAXIMIZE);

      //Changing number of MSK_IPAR_NUM_THREADS (it should be done after the optimization)
      MSK_putintparam(task, MSK_IPAR_NUM_THREADS, num_of_threads); /*Nber of cpus*/
    }

    for (int i = 0; i < instance.edge_nb; i++)
      r = MSK_putvarbound(task, i, MSK_BK_RA, 0.0 /*lower bound*/, 1.0 /*upper bound*/);

    //set fixed variables
    for (int i = 0; i < fixvar.size(); i++)
      r = MSK_putvarbound(task, fixvar[i].var, MSK_BK_RA, fixvar[i].val /*lower bound*/, fixvar[i].val /*upper bound*/);

    if (r == MSK_RES_OK)
    {
      MSKrescodee trmcode;

      /* Directs the log task stream to the 'printstr' function. */
      //       r = MSK_linkfunctotaskstream(task,MSK_STREAM_LOG,NULL,printstr);

      //choose how to solve (simplex or IPM)
      /* Specify integer variables. */
      //	 for(int j=0; j<instance.edge_nb && r == MSK_RES_OK; ++j)
      //	 r = MSK_putvartype(task,j,MSK_VAR_TYPE_INT);

      if (TYPE == TYPE_IPM_BB)
      {
        r = MSK_putintparam(task, MSK_IPAR_OPTIMIZER, MSK_OPTIMIZER_INTPNT);
        r = MSK_putintparam(task, MSK_IPAR_INTPNT_BASIS, 1); //Controls whether the interior-point optimizer also computes an optimal basis.
      }
      else
      { //if (TYPE == TYPE_SIMPLEX_BB)
        r = MSK_putintparam(task, MSK_IPAR_OPTIMIZER, MSK_OPTIMIZER_FREE_SIMPLEX);
      } //end if else

      if (r != MSK_RES_OK)
      {
        cout << "Erro before Optimizer."; ///cin.get();
      }

      //      /* Run optimizer */
      if (r == MSK_RES_OK)
        r = MSK_optimizetrm(task, &trmcode);

      //	cout << "-->" << r << "<--";

      if (r == MSK_RES_OK)
      {

        MSKsolstae solsta;

        r = MSK_getsolsta(task, MSK_SOL_BAS, &solsta);

        switch (solsta)
        {
        case MSK_SOL_STA_OPTIMAL:
        {

          xx = (double *)calloc(instance.totalVars, sizeof(double));
          Obj = (double *)MSK_calloctask(task, 2, sizeof(MSKrealt));

          MSK_getxx(task, MSK_SOL_BAS, xx);
          MSK_getprimalobj(task, MSK_SOL_BAS, Obj);

          for (int i = 0; i < instance.totalVars; i++)
            instance.varS[i] = xx[i];

          instance.ObSol = Obj[0];

          break;
        }
        case MSK_SOL_STA_DUAL_INFEAS_CER:
        case MSK_SOL_STA_PRIM_INFEAS_CER:
          //printf("Primal or dual infeasibility certificate found.\n");
          return false;
          break;
        case MSK_SOL_STA_UNKNOWN:
        {
          char symname[MSK_MAX_STR_LEN];
          char desc[MSK_MAX_STR_LEN];
          /* If the solutions status is unknown, print the termination code
	    indicating why the optimizer terminated prematurely. */
          MSK_getcodedesc(trmcode,
                          symname,
                          desc);
          printf("The solutuion status is unknown.\n");
          printf("The optimizer terminitated with code: %s\n", symname);
          break;
        }
        default:
          printf("Other solution status.\n");
          break;
        }
      }
      else
      {
        cout << "MOSEK error optimization ... Maybe it is OUT OF MEMORY .... (r != MSK_RES_OK)  r= " << r << endl;
        exit(1);
      }
    }
  } //end first IF

  redcost_BB = (double *)calloc(instance.totalVars, sizeof(double)); //Reduced cost variable
  MSK_getreducedcosts(task, MSK_SOL_BAS, 0, instance.totalVars, redcost_BB);

  r = MSK_getdouinf(task, MSK_DINF_OPTIMIZER_TIME, &time_IPM_iteration);

  free(xx);
  MSK_freetask(task, Obj);

  Time_IPM += time_IPM_iteration; // sum time of IPM

  if (TYPE_BRANCH == BnC)
  {
    MSK_deletetask(&task);
  }

  return true;
} //end of function

//*****************************
//	Solve max k cut using ----SDP--- solver
///		for Branch and bound method
// *****************************/
bool SolveMosek_SDP_for_BB(T_Instance &instance, const std::vector<T_fixVar> &fixvar)
{

  //cout << "Entrou otimo  SDP ...";

  double auxtime = getCurrentTime_Double(start);
  //cout << "Chegou" ; cin.get();
  MSKenv_t env = NULL;
  MSKtask_t task = NULL;
  bool rept = true;
  MSKrescodee r;
  //double 	cstW = (-1.0)*((K-1.0)/K); 	/*Poid que matrix W va etres  = (-(k-1)/k)*/
  MSKint64t idx;
  double *xx, *Obj, *barx;

  double falpha = 1.0; //peso de cada matrix A sera 1.0

  //Number of constraints total to SDP (remember that we have xij<-1/(k-1) and xii = 1)
  MSKint32t NUMCONS;
  if (SDP_EdGE_TYPE)
    NUMCONS = instance.DIM + sdpEdgeConst.size + instance.CONST.size();
  else
    NUMCONS = instance.DIM + instance.edge_nb + instance.CONST.size();

  MSKint32t NUMBARVAR = 1;

  MSKint32t DIMBARVAR[] = {instance.DIM}; /* Dimension of semidefinite cone */

  //NUMCONS= instance.DIM+ instance.LENBARVAR;

  /* Create the mosek environment. */
  r = MSK_makeenv(&env, NULL);

  /* Create the optimization task. */
  if (r == MSK_RES_OK)
    r = MSK_maketask(env, NUMCONS, 0, &task); // change the NUMCONS

  //Print in screen Interior point iteration and the error raport (it it exist)
  //MSK_linkfunctotaskstream(task,MSK_STREAM_LOG,NULL,printstr);

  ////* Append 'NUMCON' empty constraints.
  // The constraints will initially have no bounds.
  if (r == MSK_RES_OK)
    r = MSK_appendcons(task, NUMCONS);

  // Append 'NUMBARVAR' semidefinite variables.
  if (r == MSK_RES_OK)
  {
    r = MSK_appendbarvars(task, NUMBARVAR, DIMBARVAR);
  }

  //// Lower and upper bounds of SDP variables
  //double LBsdp = 0.0;// before was -1.0/(K-1.0);
  double LBsdp = -1.0 / (K - 1.0);
  double UBsdp = 1.0;

  //
  ////
  ////////		Setting the Objective function
  /////////
  Set_ObFunction_Mosek_SDP(instance, r, task, LBsdp);
  if (r != MSK_RES_OK)
    cout << "Erro aqui 0";

  //
  ////
  ////////		Setting the constraints
  /////////

  if (r != MSK_RES_OK)
    cout << "Erro aqui 1 .... before Set_CombinatorialConstraints_MOSEK_SDP ";

  Set_CombinatorialConstraints_MOSEK_SDP(instance, r, task, LBsdp, UBsdp);

  if (r != MSK_RES_OK)
  {
    cout << "Nb of instances = " << instance.CONST.size();
    cout << "Erro aqui 1 .... after Set_CombinatorialConstraints_MOSEK_SDP ";
  }

  //
  ////
  /////	SDP constraint
  //////
  Set_OriginalConstraints_MOSEK_SDP(instance, r, task, LBsdp); // must be executed after Set_CombinatorialConstraints_MOSEK_SDP
  if (r != MSK_RES_OK)
    cout << "Erro aqui--";

  //
  ////	ADD constraint with fixed vals
  ////

  Set_FixVar_MOSEK_SDP_BB(instance, r, task, LBsdp, fixvar); // must be executed after Set_CombinatorialConstraints_MOSEK_SDP

  ////
  //////
  //////	SOLVING
  ////////

  // Maximize objective function.
  if (r == MSK_RES_OK)
    r = MSK_putobjsense(task, MSK_OBJECTIVE_SENSE_MAXIMIZE);

  MSK_putintparam(task, MSK_IPAR_NUM_THREADS, num_of_threads); //Nber of cpus
  double FOvalue;

  if (r == MSK_RES_OK)
  {

    MSKrescodee trmcode;

    // Run optimizer
    r = MSK_optimizetrm(task, &trmcode);

    //MSK_solutionsummary (task,MSK_STREAM_MSG);
    if (r == MSK_RES_OK)
    {
      MSKsolstae solsta;

      MSK_getsolsta(task, MSK_SOL_ITR, &solsta);

      switch (solsta)
      {
      case MSK_SOL_STA_OPTIMAL:
        //MSKint32t NUMVAR 	= 0;
        xx = (double *)MSK_calloctask(task, 0, sizeof(MSKrealt));
        barx = (double *)MSK_calloctask(task, instance.LENBARVAR, sizeof(MSKrealt));
        Obj = (double *)MSK_calloctask(task, 2, sizeof(MSKrealt));
        double *Obj2;
        Obj2 = (double *)MSK_calloctask(task, 2, sizeof(MSKrealt));

        MSK_getxx(task,
                  MSK_SOL_ITR,
                  xx);
        MSK_getbarxj(task,
                     MSK_SOL_ITR, /* Request the interior solution. */
                     0,
                     barx);

        MSK_getprimalobj(task, MSK_SOL_ITR, Obj);

        if (SDP_EdGE_TYPE_BB) // just for SDP solver
          Find_edgeViolated_in_SDP(instance, barx);

        //Setting soluition in instance but before we have to
        //Transform the soluition in 0,1 ... because all the separations are based on 0,1 variables.

        Transform_SDPsol_2_LPsol_for_BB(instance, barx, LBsdp, UBsdp);

        instance.ObSol = Obj[0];

        break; //end of switch
      case MSK_SOL_STA_DUAL_INFEAS_CER:
      case MSK_SOL_STA_PRIM_INFEAS_CER:
        //printf("Primal or dual infeasibility certificate found.\n");
        return false;
        break;

      case MSK_SOL_STA_UNKNOWN:
        printf("The status of the solution could not be determined.\n");
        break;
      default:
        printf("Other solution status.");
        break;
      } //end switch
    }
  }
  else
  {
    cout << "Erro before optimizing ( r !=  MSK_RES_OK) ... erro implementation while adding SDP constraints in SolveMosekSDP";
    exit(1);
  }

  r = MSK_getdouinf(task, MSK_DINF_OPTIMIZER_TIME, &time_IPM_iteration);

  //free memory
  if (r == MSK_RES_OK)
  {
    MSK_deletesolution(task, MSK_SOL_ITR); // clean solution (can change the sdp performance ??)
    MSK_freetask(task, xx);
    MSK_freetask(task, barx);
    MSK_freetask(task, Obj);
    MSK_deletetask(&task);
    MSK_deleteenv(&env);
  }

  Time_IPM += time_IPM_iteration; // sum time of IPM
  return true;
}

inline void Transform_SDPsol_2_LPsol_for_BB(T_Instance &instance, double *barx, const double &LBsdp, const double &UBsdp)
{
  //from  X = (Lb, Ub) to lp y = (0,1)
  //	From line equation We have:
  //		y = (X - Lb)/(Ub - Lb),
  //		  = X/(Ub - Lb) -  Lb/(Ub - Lb)
  //
  //

  int aux_edge, I, J;
  double valSDP;
  double divCst = (UBsdp - LBsdp);

  for (int j = 0; j < instance.DIM; j++)
    for (int i = j + 1; i < instance.DIM; i++)
    {
      I = i + 1;
      J = j + 1;
      aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
      if (aux_edge != -1)
      { //valid edge in graph
        valSDP = getValueXij_SDP(i, j, barx, instance.DIM);
        instance.varS[aux_edge] = (valSDP - LBsdp) / divCst; // y = (X - Lb)/(Ub - Lb),
      }
    }
}

inline void Set_FixVar_MOSEK_SDP_BB(const T_Instance &instance, MSKrescodee &r, MSKtask_t &task, const double &LBsdp, const std::vector<T_fixVar> &fixvar)
{

  int conidx, I, J;
  double ValFix, ubBoundSDP = 1.0;
  MSKint64t idx;
  double falpha = 1.0; //peso de cada matrix A sera 1.0

  /* Get index of new constraint*/
  if (r == MSK_RES_OK)
    r = MSK_getnumcon(task, &conidx);

  //iterate all fixed variables
  for (int j = 0; j < fixvar.size(); j++)
  {
    /* Append a new constraint */
    if (r == MSK_RES_OK)
      r = MSK_appendcons(task, 1);

    if (fixvar[j].val == 0.0)
      ValFix = LBsdp;
    else
      ValFix = ubBoundSDP;

    MSKboundkeye bkc = MSK_BK_FX; // It is equality key (=)

    //right size (ARE THE BOUNDS OF CONSTRAINTS)
    r = MSK_putconbound(task,
                        conidx,  // Index of constraint.
                        bkc,     // Bound key.
                        ValFix,  // Numerical value of lower bound.
                        ValFix); // Numerical value of upper bound.

    //left size

    I = instance.cij.barc_i[fixvar[j].var];
    J = instance.cij.barc_j[fixvar[j].var];

    //cout << "Var =" <<  fixvar[j].var << endl;
    //cout << " I = " << I <<  " and J =" << J ; cin.get();

    MSKint32t var_i = (MSKint32t)I - 1; //should start at 0
    MSKint32t var_j = (MSKint32t)J - 1; // should start at vertex 0 not 1
    double vall = 0.50;

    if (r == MSK_RES_OK)
      r = MSK_appendsparsesymmat(task,
                                 instance.DIM,
                                 1,
                                 &var_i,
                                 &var_j,
                                 &vall,
                                 &idx);

    if (r == MSK_RES_OK)
      r = MSK_putbaraij(task, conidx, 0, 1, &idx, &falpha);

    conidx++;

    if (r != MSK_RES_OK)
    {
      cout << "i=" << I << " j=" << J << endl;
      cout << endl
           << "r = " << r;
      cin.get();
    }
  } //end i and j
}
//This is a simplified version of the cutting plane algo to be applied in the branch and cut algorithm
bool CuttingPlane_Simple_for_BB(T_Instance &instance_orig, std::vector<std::vector<int>> *Partitions, std::vector<T_fixVar> &fixvar,
                                const bool &BY_PARTITION_BB, double *bestLowerBound, const int &cleanIneq, const std::vector<T_constraint> &CONST_branch)
{
  double MAXTIME_CP = instance_orig.DIM * 0.5;

  //cout << instance2.ObSol << endl;
  T_Instance instance = instance_orig;
  double gapMinImprBB = 0.001,
         gapMinCopyConstBB = 0.005, //min improvement of 0.1%
      FirstObj;
  int typeSolver;
  CLEAN = cleanIneq;
  NB_SDPineq = instance.DIM;
  bool DoBoundSep = true;
  COMPLET_EIG = false;

  if ((SDP_SEP == -1) || (SDP_SEP == -3))
    typeSolver = TYPE_SDP_BB;
  else
    typeSolver = TYPE_IPM_BB; //  TYPE_SIMPLEX_BB

  if (BY_PARTITION_BB)
    Set_FixVar_fromPartition_BB(instance, *Partitions, fixvar); //Set vector with all edges fixed from partition

  bool RESUME = true;
  MSKrescodee r; //mosek Parameters

  //clear Population of violated inequalities
  clear_AllPopSeparation();
  sdpEdgeConst.clear();
  ActiveAllIneq();

  if (JUST_TRIandCLI == true)
  {
    ActiveJustTriWhell();
    DYNAMIC_ACTIV_INEQ = false;
  }

  NBMAXINEQ = 100;

  /******* cpa algorithm ****/
  start = clock(); // start time

  //Pretreatment

  double PrvObj = 0.0;

  if (CLEAN != 0)
  {
    if ((SDP_SEP == -2) || (SDP_SEP == -3))
      CLEAN = 2;
    else
      CLEAN = 3;
  }

  //NBMAXINEQ = instance.DIM*0.50; // 50% of DIMBARVAR (for LP_EIG)

  //  cout <<"size = " << CONST_branch.size(); cin.get();
  //Inserting ExtraCONST_BB in Instance
  instance.CONST = CONST_branch;
  //  for (int c=0; c<ExtraCONST_BB.size(); ++c)
  //  	instance.CONST.push_back(ExtraCONST_BB[c]);

  //Optimization and Separation method
  for (int i = 0; i < nb_ITE && RESUME; i++)
  {
    try
    {

      RESUME = Solve_SubProblem_BB(instance, NULL, fixvar, false, typeSolver, true, bestLowerBound, DoBoundSep);

      if (instance.ObSol < *bestLowerBound + 1.0)
      {
        return false;
      }
      if (i == 0)
        FirstObj = instance.ObSol;

      // CLEAN Unimportant inequalities (just once !!!!!!)
      //      if (i==0 && CLEAN != 0 )
      //		Clean_uselessInequalities (instance);

      //stopping criterias Iteration improvement
      if (RESUME)
      {
        if (i == 0)
          gapImp = 1.0;
        else
          gapImp = (PrvObj - instance.ObSol) / PrvObj;

        if (gapImp < gapMinImprBB)
          RESUME = false;

        PrvObj = instance.ObSol;
      }
      else
      {
        //end of function (infeasible)
        return RESUME; //it means that we had problem (maybe infeasible problem) in Solve_SubProblem_BB
      }

      //print  iterations
      if (false)
      {
        Print_Iterations_screen(instance, FileLecture, i);
        cout << " * " << gapImp << ", | , ";
        cout << time_IPM_iteration << ", | , " << endl;
        cin.get();
      }

      //      cout << "Starting Search : " << endl;
      //STOP CRITERIA: No violation fund
      if (RESUME)
      {

        RESUME = FindViolation(instance);

        if ((!RESUME))
          if ((i == 0) || ((DoBoundSep) && (sdpEdgeConst.newadd > 0)))
            RESUME = true;
      }

      //3 STOP CRITERIA (max time)
      if ((getCurrentTime_Double(start) >= MAXTIME_CP || gapImp < epslon_s * 1000) /*|| (gapImp < MinIteImpro)*/)
        RESUME = false;

      //Selective separation or Printing final solution !!!
      if (RESUME)
        Selective_Separation(instance);

      //clear Population of violated inequalities
      clear_AllPopSeparation();
    }
    catch (std::exception &e)
    {
      cerr << "Std Error in cutting plane of branch and bound (cut) " << endl;
    }
    catch (...)
    {
      cerr << "Unknown Exception no cutting plane in Branch and cut" << endl;
    }
  } //end of iterations

  //  if ((FirstObj -  instance.ObSol)/instance.ObSol >= gapMinCopyConstBB && instance.ObSol > *bestLowerBound + 0.9 ){
  ////  cout << "Entrou aqui brother size to be included = " << instance.CONST.size() - LastConstraintInstance << ", with instance.ObSol= " << instance.ObSol; cin.get();
  Clean_uselessInequalities(instance);
  ExtraCONST_BB = instance.CONST;

  instance_orig.varS = instance.varS;
  instance_orig.ObSol = instance.ObSol;
  //	  cout << instance2.ObSol << endl; cin.get();
  return true;
}

/********************************** End of BB functions ********************/

//Main function to to the cutting plane algorithm
double CuttingPlane_Optimization(T_Instance &instance, const bool &PRINT_ITERATIONS, clock_t Start)
{
  if (K <= 6 && SDP_SEP == -1) // Only for small k that the  early termination is the best option for SDP
    SDP_SEP = -3;

  NB_SDPineq = instance.DIM;

  double MAXBEFORE, firstSolVal;
  bool RESUME = true;
  //   double 	MAXTIME_ITE_Begin = MAXTIME_ITE;
  MSKrescodee r; //mosek Parameters

  //clear Population of violated inequalities
  clear_AllPopSeparation();
  sdpEdgeConst.clear();

  /******* cpa algorithm ****/
  start = Start; //clock(); // start time

  MSKtask_t task = NULL;
  MSKenv_t env = NULL;

  MAXVALUE = MAXBEFORE = MSK_INFINITY;
  //MAXBEFORE = 0;

  //Pretreatmentf
  SetPretreatmen(instance);

  double PrvObj = 0.0;

  int intervalIPM = 5; // fixed after calcule

  if (SDP_SEP == -3)
    intervalIPM = 3;

  if (CLEAN != 0)
  {
    if ((SDP_SEP == -2) || (SDP_SEP == -3))
      CLEAN = 2;
    else
      CLEAN = 3;
  }

  //First NbInex ...
  //if (NBMAXINEQ == 0) NBMAXINEQ = 2*instance.DIM ; //
  //else DYNAMIC_INEQ = false;

  /*Fixing Number of INequalities (IMPORTANT) parameter for improving performance*/
  DYNAMIC_INEQ = false;

  if (SDP_SEP == 0)
  {
    DYNAMIC_ACTIV_INEQ = false;
    ActiveAllIneq();
    NBMAXINEQ = 1000;
  }
  else
  {
    DYNAMIC_ACTIV_INEQ = true;
    DesactivateAll();
    if (SDP_SEP == -1 && !SDP_EdGE_TYPE)
      ActiveJustTriWhell();

    if (SDP_SEP == -1 || SDP_SEP == -3)
      NBMAXINEQ = NBINEQSDP; //sdp
    else
      NBMAXINEQ = instance.DIM * 0.50; // 10% of DIMBARVAR (for LP_EIG)

    if (!ISCOMPLETE_GRAPH && SDP_SEP == -2)
    {
      DYNAMIC_ACTIV_INEQ = false;
      ActiveAllIneq();
      NBMAXINEQ = NBINEQSDP;
    }
  }

  if (JUST_TRIandCLI == true)
  {
    ActiveJustTriWhell();
    DYNAMIC_ACTIV_INEQ = false;
  }
  else if (instance.DIM <= 50)
  {
    DYNAMIC_ACTIV_INEQ = false;
    ActiveAllIneq();
    NBMAXINEQ = instance.DIM * 2;
  }
  else if (instance.DIM <= 100)
  {
    if (SDP_SEP != 0)
      NBMAXINEQ = instance.DIM;
  }

  bool OPTMAL_DONE, LP_TO_OPTM;
  OPTMAL_DONE = LP_TO_OPTM = false;

  //AddAll_TriangleInequality( instance, 10000, 0, PopTriangle);
  //cout << "TOTAL_TRI = " << TOTAL_TRI;
  int countNot = 0;
  for (int i = 0; i < nb_ITE && RESUME; i++)
  {
    //Optimization and Separation method
    try
    {
      //      cout << "Starting IPM : " << endl;
      if (((SDP_SEP == -2) || (SDP_SEP == -3)) && (i % intervalIPM == 0))
        LP_TO_OPTM = true;

      if ((i == 0) && ((SDP_SEP == 1) || (SDP_SEP == -2) || (SDP_SEP == 0) || (SDP_SEP == 3)))
      {                                // if LP method we will set the initial solution !!!
        clock_t start2 = std::clock(); //New start
        double eco = 0.0, eco2 = 0.0;  // are the savings brought by methods
        //eco = PreTrea_Div2Conq_forUpperbound(instance); //Creat a heuristic method to improve this pretreatment // Desacivated because it is worse than setInitial_LP_Solution
        eco2 = setInitial_LP_Solution(instance); //Trivial solution (good for all positive edges and small k , or Upper bounds from articles)

        time_IPM_iteration = getCurrentTime_Double(start2); //CPU time to calculate initial solution
        if (eco > eco2)
          instance.ObSol -= eco; //it means that Pre treatment was more effective than trivial solution
        else
          instance.ObSol -= eco2;

        if (DO_EIG_INEQ_FROM_SDP && SDP_SEP != 0)
        {
          setSolver(instance, r, task, env, -1); // solve  IPM  to optmality (SDP)
          DO_EIG_INEQ_FROM_SDP = false;
          clear_AllPopSeparation();
        }

        eco2 = setInitial_LP_Solution(instance); //Trivial solution (good for all positive edges and small k , or Upper bounds from articles)
        instance.ObSol -= eco2;

        OPTMAL_DONE = true; //flag that optmal IPM was performed

        Set_IneqFuncObj(instance, instance.ObSol); //This way we limit our next solutions to be inferior than first solution. Impotant when we use Upperbound from Nikiforov
      }
      else
      {
        DO_EIG_INEQ_FROM_SDP = false; // only first iteration of SDP_EIG
        //Set solver
        if (LP_TO_OPTM)
        {
          if (SDP_SEP == -2)
            setSolver(instance, r, task, env, 1); // solve  IPM  to optmality
          else if (SDP_SEP == -3)
            setSolver(instance, r, task, env, -1); // solve  IPM  to optmality (SDP)
          else
            setSolver(instance, r, task, env, SDP_SEP);

          OPTMAL_DONE = true;
        }
        else
        {

          setSolver(instance, r, task, env, SDP_SEP);

          if ((SDP_SEP == -2) || (SDP_SEP == -3))
            OPTMAL_DONE = false;
          else
            OPTMAL_DONE = true;
        }
      }

      //stopping criterias Iteration improvement
      if (OPTMAL_DONE)
      {
        if (i == 0)
          gapImp = 1.0;
        else
          gapImp = (PrvObj - instance.ObSol) / PrvObj;

        if (i == 0)
          firstSolVal = instance.ObSol;
        PrvObj = instance.ObSol;
      }

      //write in the iteration file
      if ((OPTMAL_DONE) && (PRINT_ITERATIONS))
      {
        WriteIteration_FILE(instance, file_ITE, FileLecture, i);
        bestSol = instance.ObSol;
      }
      if (PRINT_ITERATIONS)
      {
        Print_Iterations_screen(instance, FileLecture, i);
        if (OPTMAL_DONE)
          cout << " * ";
        cout << gapImp << ", | , ";
        cout << time_IPM_iteration << ", | , " << endl;
        //        cin.get();
      }

      LP_TO_OPTM = false; //reset

      if (i == 0)
        Obj1 = instance.ObSol;
      //      cout << "Starting Search : " << endl;

      //STOP CRITERIA: No violation fund
      RESUME = FindViolation(instance);

      if (!RESUME)
      {
        if (i == 0)
          RESUME = true;
        else if (!OPTMAL_DONE)
          RESUME = true;
        else if (SDP_EdGE_TYPE && sdpEdgeConst.newadd > 0)
          RESUME = true;
      } //end IF !RESUME

      //Dynamic inequality
      if ((DYNAMIC_INEQ) && (OPTMAL_DONE))
        DynamicIneqSize(instance, gapImp, &NBMAXINEQ);

      if ((DYNAMIC_ACTIV_INEQ) && (OPTMAL_DONE) && (SDP_SEP != 0))
        DynamicIneqActivation(instance, gapImp);

      //STOP CRITERIA (TIME and Gap improvement)
      if ((getCurrentTime_Double(start) >= MAXTIME) || time_IPM_iteration >= MAXTIME_ITE || (gapImp < epslon_s * 100 && (instance.ObSol < Obj1 - epslon || countNot >= 10)))
      {
        RESUME = false;
        // 	MAXTIME_ITE = 0.0; (there is no sense in putting this here)
      }

      if (instance.ObSol >= Obj1 - epslon)
        countNot++;

      //Selective separation or Printing final solution !!!
      if (RESUME)
        Selective_Separation(instance);
      //       else 		 WriteFinalResult_File (fileResult,instance,start,i);

      //if ((i == 0)&&(PreVP == 13))
      //SubGraphTreat_by_ViolatedIneq(instance);

      // CLEAN Unimportant inequalities
      if ((!RESUME && OPTMAL_DONE) || ((i != 0) && (CLEAN != 0) && (i % CLEAN == 0) && (OPTMAL_DONE) && (gapImp > epslon_s * 100)))
      {
        COMPLET_EIG = false;
        if (instance.ObSol >= Obj1 - epslon)
          instance.CONST.clear();
        else
          Clean_uselessInequalities(instance);
      }

      //clear Population of violated inequalities
      clear_AllPopSeparation();
    }
    catch (std::exception &e)
    {
      cerr << "Std Error " << endl;
    }
    catch (...)
    {
      cerr << "Unknown Exception no cutting plane" << endl;
    }

  } //end of iterations

  if (SDP_EdGE_TYPE && ((SDP_SEP == -1) || (SDP_SEP == -3)))
    Clear_edgeNotViolated_in_SDP(instance); //clean useless bound ineq in SDP formulation

  //making last iteration
  if (PRINT_ITERATIONS)
  {
    //executing last iteration (the one that is going to B&B)
    if (SDP_SEP == -2)
      setSolver(instance, r, task, env, 1); // solve IPM to optimality
    else if (SDP_SEP == -3)
      setSolver(instance, r, task, env, -1); // solve IPM to optimality
    else
      setSolver(instance, r, task, env, SDP_SEP);

    WriteIteration_FILE(instance, file_ITE, FileLecture, 1); //Writing last iteration
  }
  else
  {
    if (SDP_SEP == -2)
      setSolver(instance, r, task, env, 1); // solve IPM to optimality
    else if (SDP_SEP == -3)
      setSolver(instance, r, task, env, -1); // solve IPM to optimality
    else
      setSolver(instance, r, task, env, SDP_SEP);

    Clean_uselessInequalities(instance);
  }

  // Delete the task, environment and the associated data. (Verify if it is optimal)
  MSK_deletetask(&task);
  MSK_deleteenv(&env);

  //   MAXTIME_ITE = MAXTIME_ITE_Begin;

  return instance.ObSol;
}

//Upper bound base on mathematical formulation of Nikiforov and Adam&Sotirov (see article Max-k-cut and the smallest eigenvalue)
double UpperBoundSmallestEigenvalue(T_Instance &instance)
{
  //cout << "Begin Eig find"; cin.get();
  //calcule eigenvalues and eigenvector is made by EIGEN (http://eigen.tuxfamily.org)
  int I, J;
  long aux_edge;
  MinVP = 1;
  MatrixXd mat_X(instance.DIM, instance.DIM);

  double eigVal = -1, MinEigVal, up, MaxEigVal, up_SD;
  int counterNeg = 0;
  SelfAdjointEigenSolver<MatrixXd> es; // just for symmetric matrix (comp. it is 10 times faster than general method)

  double w_V = 0.0;
  //Tranformation G in to adjacent matrix mat_X
  for (int i = 0; i < instance.DIM; ++i)
    for (int j = i; j < instance.DIM; ++j)
    {
      if (i != j)
      {
        I = i + 1;
        J = j + 1;
        //putting in xpr
        aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
        //		cout << "aux_edge=" << aux_edge; cin.get();
        if (aux_edge != -1)
        {
          //		cout << "Chegou";
          mat_X(i, j) = instance.cij.barc_v[aux_edge]; // non weighted put  1.0 instead
          mat_X(j, i) = instance.cij.barc_v[aux_edge]; // non weighted put  1.0 instead
          w_V += instance.cij.barc_v[aux_edge];
        }
        else
        {                    //end if aux_edge = ok
          mat_X(i, j) = 0.0; // non weighted put  1.0 instead
          mat_X(j, i) = 0.0; // non weighted put  1.0 instead
        }
      }
      else
        mat_X(i, j) = 0.0; // non weighted put  1.0 instead
    }
  /*
   * 	*** The Upper bound from Nikiforov ***
   *
   *	mc_k(G,W=mat_X) = (k-1)/k(w_v - MinEigVal/2)
   */
  //    cout << mat_X; cin.get();//Print Matrix
  //Compute the eigenvalue and eigenvectr
  es.compute(mat_X);

  MinEigVal = es.eigenvalues()[0];                                           // Get min eigenvalue of G matrix
  up = (((double)K - 1.0) / K) * (w_V - ((MinEigVal * instance.DIM) / 2.0)); // Nikiforov

  //Calculating Dam and Sotirov Upper bound  (seems to be weaker than Nikiforov ... at leat it was the case for all tested instances)
  /*
   * Upper bound of Sotirov and Dam :
   *
      mc_k(G,W) <= n(k-1)/2k * (MaxEigVal(L))

   */

  /* (In all tested instance  it was worse than Nikirov)
  VectorXd VecId (instance.DIM), auxVec (instance.DIM);
  MatrixXd L(instance.DIM,instance.DIM);
  VecId.setOnes(); //1 vector (all ones)

  auxVec = mat_X*VecId;
  L = auxVec.asDiagonal(); // Diagonal of vector


  L = L -mat_X; //Calculating the laplacian of graph

  es.compute(L);
  MaxEigVal = es.eigenvalues()[instance.DIM -1]; // Maximum eigenvalue


  up_SD = (instance.DIM*(K-1.0)/(2.0*K))*(MaxEigVal);

  */

  //up_SD = ((double)(instance.DIM *(K-1.0))/(2.0*K))*(MaxEigVal); // Satirov

  //  cout << endl << "MinEigVal = " << MinEigVal << ", w_V =" << w_V << "Up = " <<up << endl; cin.get();

  //cout << endl << "MaxEigVal = " << MaxEigVal << "up_SD = " <<up_SD; cin.get();

  //   if (up < up_SD)  return up;
  //  else

  return up; //Nikiforov upper bound

} // end function

//Upper bound base on mathematical formulation of Nikiforov and Adam&Sotirov (see article Max-k-cut and the smallest eigenvalue)
double UpperBoundSmallestEigenvalue_UnweightedGraph(T_Instance &instance)
{
  //cout << "Begin Eig find"; cin.get();
  //calcule eigenvalues and eigenvector is made by EIGEN (http://eigen.tuxfamily.org)
  int I, J;
  long aux_edge;
  MinVP = 1;
  MatrixXd mat_X(instance.DIM, instance.DIM);

  double eigVal = -1, MinEigVal, up, MaxEigVal, up_SD;
  int counterNeg = 0;
  SelfAdjointEigenSolver<MatrixXd> es; // just for symmetric matrix (comp. it is 10 times faster than general method)

  double w_V = 0.0;
  //Tranformation G in to adjacent matrix mat_X
  for (int i = 0; i < instance.DIM; i++)
    for (int j = 0; j < instance.DIM; j++)
    {
      if (i != j)
      {
        I = i + 1;
        J = j + 1;
        //putting in xpr
        aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
        if (aux_edge != -1)
        { //valid edge in graph
          if (instance.cij.barc_v[aux_edge] != 0)
          {
            mat_X(i, j) = 1.0; // non weighted put  1.0 instead
            w_V += 0.5;
          }
        }
        else
        {
          mat_X(i, j) = 0.0; // non weighted put  1.0 instead
        }
      }
      else
      {
        mat_X(i, j) = 0.0; // non weighted put  1.0 instead
      }
    }

  // cout << mat_X; //Print Matrix
  //Compute the eigenvalue and eigenvectr
  es.compute(mat_X);

  MinEigVal = es.eigenvalues()[0]; // Get min eigenvalue of G matrix
                                   //MaxEigVal = es.eigenvalues()[instance.DIM -1];

  up = (((double)K - 1.0) / K) * (w_V - ((MinEigVal * instance.DIM) / 2.0)); // Nikiforov

  //up_SD = ((double)(instance.DIM *(K-1.0))/(2.0*K))*(MaxEigVal); // Satirov

  cout << endl
       << "MinEigVal = " << MinEigVal << ", w_V =" << w_V << ", Up = " << up << endl;
  cin.get();

  //   cout << endl << "MaxEigVal = " << MaxEigVal << ", ((double)(instance.DIM *(K-1.0))/(2.0*K))=" << ((double)(instance.DIM *(K-1.0))/(2.0*K)) << "Up = " <<up_SD; cin.get();

  //   if (up < up_SD)  return up;
  //  else

  if (up < w_V) // creat ineaquality
    Set_IneqNuberOfEdgesCut(instance, (double)((int)up), (int)w_V);

  return up; //Nikiforov upper bound

} // end function

void PreTrea_Div2Conq_Eigenvalue(T_Instance &instance)
{
  cout << "Enter PreTrea_Div2Conq_Eigenvalue" << endl;
  T_Instance instance2;      //Store information of instance
  vector<int> origen_indice; //useful to creat the valid inequalities in the instance
  int size_max = instance.DIM * 0.10;

  if (size_max > 10)
    size_max = 10;

  int realSize = size_max;
  int nbEdges = size_max * (size_max - 1) / 2;

  //cout << "nb of inequalities added = " <<   instance.CONST.size() << endl;

  origen_indice.resize(nbEdges + 1);

  for (int i = 0; i < instance.DIM; i = i + size_max)
  {

    if (i + size_max > instance.DIM)
      realSize = instance.DIM - i;

    ClearInstance(instance2);
    Copy_instance(instance, instance2, realSize, i, origen_indice);

    Optmize_subGraph_onlyEig(instance2);
    Set_validIneq_in_MainInstance_justEig(instance, instance2, origen_indice);
  } // end FOR

  //cin.get();
}

void PreTrea_Div2Conq_rank_exploreAll(T_Instance &instance)
{

  cout << "Enter PreTrea_Div2Conq_rank_exploreAll " << endl;
  T_Instance instance2; //Store information of instance
  set<MKC_InstanceRankVertices> ranked;
  set<MKC_InstanceRankVertices>::iterator it;
  vector<int> ranked_v;
  vector<int> begin_v;
  vector<int> end_v;
  int cont;
  double prop;

  if (ICH != 0)
    prop = (double)ICH / 200.0;
  else
    prop = 0.25;

  int size_max = instance.DIM * prop;
  if (size_max > 30)
    size_max = 30;

  int realSize = size_max;
  int size_max_two = size_max * 2;

  int nbEdges = size_max_two * (size_max_two - 1) / 2;
  vector<int> origen_indice; //useful to creat the valid inequalities in the instance
  origen_indice.resize(nbEdges + 1);

  // start rank vertex
  ranked_v.resize(instance.DIM);
  //if (PreVP == 10) //rank by vertex degree
  Rank_by_degree(ranked, instance);

  //if (PreVP == 11) //rank by vertex incident weight
  //Rank_by_incidentWeight(ranked,instance);

  cont = 0;
  if ((PreVP == 11) || (PreVP == 10))
  {
    for (it = ranked.begin(); it != ranked.end(); ++it)
    {
      ranked_v[cont] = (*it).vertex;
      cont++;
    }
  }
  else
  {
    for (int i = 0; i < instance.DIM; i++)
      ranked_v[i] = i + 1;
  }
  //end rank

  //creating all
  int qt = instance.DIM / size_max;
  if (instance.DIM % size_max != 0)
    qt++;

  for (int i = 0; i < qt; i++)
  {
    begin_v.push_back((size_max * i) + 1);

    if (begin_v[i] + size_max <= instance.DIM)
      end_v.push_back(begin_v[i] + size_max - 1);
    else
      end_v.push_back(instance.DIM);
  }

  for (int i = 0; i < (int)begin_v.size(); i++)
    for (int j = i + 1; j < (int)begin_v.size(); j++)
    {

      if (end_v[j] - begin_v[j] > 2)
      { //have to have at least 3 vertices in the group
        ClearInstance(instance2);
        Copy_by_two_SubGraphs(instance, instance2, begin_v, end_v, i, j, origen_indice);

        Optmize_subGraph(instance2);
        Set_validIneq_in_MainInstance(instance, instance2, origen_indice);
      }
    } //end i and j
}

void SubGraphTreat_by_ViolatedIneq(T_Instance &instance)
{

  cout << "Enter SubGraphTreat_by_ViolatedIneq " << endl;
  T_Instance instance2; //Store information of instance
  set<MKC_InstanceRankVertices> ranked;
  set<MKC_InstanceRankVertices>::iterator it;
  vector<int> ranked_v;
  int cont, realSize;
  int size_max = 50;
  int qt_ineq = 70;
  int max_rand = 200; // just some of the top max_rand inequalities will be randolly selected
  int Nb_Itertions = 20;

  int nbEdges = instance.DIM * (instance.DIM - 1) / 2;
  vector<int> origen_indice; //useful to creat the valid inequalities in the instance
  origen_indice.resize(nbEdges + 1);

  //Copying populations to not lost their information after the optimization
  MKC_ConstraintTrianglePopulate PopTriangle_origem = PopTriangle;
  MKC_ConstraintCliquePopulate PopClique_origem = PopClique;
  MKC_ConstraintCliquePopulate PopGenClique_origem = PopGenClique;
  MKC_ConstraintWheelPopulate PopWheell_origem = PopWheell;
  MKC_ConstraintWheelPopulate PopBiWheell_origem = PopBiWheell;

  // Population of inequalities
  MKC_ConstraintSelectionPopulate PopBestIneq_origem = PopBestIneq;

  //Select the most violated inequalities
  //MKC_ConstraintLPtoSDPPopulate		PopVecProp;

  //printvector(ranked_v);

  for (int i = 0; i < Nb_Itertions; i++)
  {

    select_vertices_fromMOstviolatedIneq(instance, ranked_v, max_rand, qt_ineq, size_max,
                                         PopBestIneq_origem, PopTriangle_origem, PopClique_origem, PopGenClique_origem,
                                         PopWheell_origem, PopBiWheell_origem);
    realSize = ranked_v.size();

    if (realSize > 10)
    {
      ClearInstance(instance2);
      //Copy_instance(instance, instance2, realSize, i, origen_indice);
      Copy_by_rank(instance, instance2, realSize, 0, origen_indice, ranked_v);
      Optmize_subGraph(instance2);
      Set_validIneq_in_MainInstance(instance, instance2, origen_indice);
    }
  }
}

inline double PreTrea_Div2Conq_forUpperbound(T_Instance &instance)
{

  cout << "Enter PreTrea_forUpperbound" << endl;
  T_Instance instance2; //Store information of instance
  set<MKC_InstanceRankVertices> ranked;
  set<MKC_InstanceRankVertices>::iterator it;
  vector<int> ranked_v;
  int cont;
  double begin, sum = 0.0;

  int size_max = instance.DIM / 2;
  if (size_max > 100)
    size_max = 100;

  //   if (size_max < K*10)
  //    size_max = K*10;

  int realSize = size_max;

  //cout << "size_max" << size_max;

  int nbEdges = size_max * (size_max - 1) / 2;
  vector<int> origen_indice; //useful to creat the valid inequalities in the instance
  origen_indice.resize(nbEdges + 1);

  // start rank vertex

  Rank_by_incidentWeight(ranked, instance);
  if (ranked.size() != instance.DIM)
  {
    cout << "Pegou differenca " << ranked.size();
    cin.get();
  }
  ranked_v.resize(ranked.size());
  cont = 0;

  for (it = ranked.begin(); it != ranked.end(); ++it)
  {
    ranked_v[cont] = (*it).vertex;
    cont++;
  }

  //end rank

  //printvector(ranked_v);

  for (int i = 0; i < ranked_v.size(); i = i + size_max)
  {

    if (i + size_max > ranked_v.size())
      realSize = ranked_v.size() - i;

    ClearInstance(instance2);
    Copy_by_rank(instance, instance2, realSize, i, origen_indice, ranked_v);

    //Initial solution
    setInitial_LP_Solution(instance2);
    begin = instance2.ObSol;

    Optmize_subGraph(instance2); //review

    //cout << "Antes = " << begin << ", depois =" << instance2.ObSol << " eco =" << eco; //cin.get();
    sum += begin - instance2.ObSol /*+ eco*/;

    //Set_validIneq_in_MainInstance(instance, instance2, origen_indice);
  }

  return sum;

} //end div2conquer new

void PreTrea_Div2Conq(T_Instance &instance)
{

  cout << "Enter PreTrea_Div2Conq " << endl;
  T_Instance instance2; //Store information of instance
  set<MKC_InstanceRankVertices> ranked;
  set<MKC_InstanceRankVertices>::iterator it;
  vector<int> ranked_v;
  int cont;
  double prop;

  if (ICH != 0)
    prop = (double)ICH / 100.0;
  else
    prop = 0.5;

  cout << "prop = " << prop << endl;
  int size_max = instance.DIM * prop;
  if (size_max > 60)
    size_max = 60;

  int realSize = size_max;

  cout << "size_max" << size_max;

  int nbEdges = size_max * (size_max - 1) / 2;
  vector<int> origen_indice; //useful to creat the valid inequalities in the instance
  origen_indice.resize(nbEdges + 1);

  // start rank vertex
  ranked_v.resize(instance.DIM);
  if ((PreVP == 10) || (PreVP == 13) || (PreVP == 14)) //rank by vertex degree
    Rank_by_degree(ranked, instance);

  if (PreVP == 11) //rank by vertex incident weight
    Rank_by_incidentWeight(ranked, instance);

  cont = 0;
  if ((PreVP == 11) || (PreVP == 10) || (PreVP == 13) || (PreVP == 14))
  {
    for (it = ranked.begin(); it != ranked.end(); ++it)
    {
      ranked_v[cont] = (*it).vertex;
      cont++;
    }
  }
  else
  { //PreVP == 9
    for (int i = 0; i < instance.DIM; i++)
      ranked_v[i] = i + 1;
  }
  //end rank

  //printvector(ranked_v);

  for (int i = 0; i < instance.DIM; i = i + size_max)
  {

    if (i + size_max > instance.DIM)
      realSize = instance.DIM - i;

    ClearInstance(instance2);
    //Copy_instance(instance, instance2, realSize, i, origen_indice);
    Copy_by_rank(instance, instance2, realSize, i, origen_indice, ranked_v);

    Optmize_subGraph(instance2);
    Set_validIneq_in_MainInstance(instance, instance2, origen_indice);
  }
}

inline void Rank_by_incidentWeight(std::set<MKC_InstanceRankVertices> &ranked, const T_Instance &instance)
{
  double cont_i;
  int aux_edge, I, J;
  vector<int> ranked_v;

  for (int i = 0; i < instance.DIM; i++)
  {
    cont_i = 0.0;
    for (int j = 0; j < instance.DIM; j++)
      if (i != j)
      {
        I = i + 1;
        J = j + 1;
        aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
        if (instance.cij.barc_v[aux_edge] != 0)
          cont_i += instance.cij.barc_v[aux_edge];
      } //end IF and FOR j
    if (cont_i > 0)
    { //jUST POSITIVES
      MKC_InstanceRankVertices rkVer(i + 1, cont_i);
      ranked.insert(rkVer);
    }
  } //end FOR i
}

void Rank_by_degree(std::set<MKC_InstanceRankVertices> &ranked, const T_Instance &instance)
{
  double cont_i;
  int aux_edge, I, J;

  for (int i = 0; i < instance.DIM; i++)
  {
    cont_i = 0.0;
    for (int j = 0; j < instance.DIM; j++)
      if (i != j)
      {
        I = i + 1;
        J = j + 1;
        aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
        if (instance.cij.barc_v[aux_edge] != 0)
          cont_i += 1.0;
      } //end IF and FOR j
    MKC_InstanceRankVertices rkVer(i + 1, cont_i);
    ranked.insert(rkVer);
  } //end FOR i

  //std::set<MKC_InstanceRankVertices>::iterator it;
  //for (it=ranked.begin(); it!=ranked.end(); ++it)
  //   std::cout << ' ' << (*it).vertex << " = " << (*it).weight << endl;
  //cin.get();
}

inline void DynamicIneqActivation(const T_Instance &instance, const double &gapImp)
{

  double minActGapTRI = 0.03, minActGapAll = 0.01;
  if (gapImp < minActGapAll)
    ActiveAllIneq();
  else if (gapImp < minActGapTRI)
    ActiveJustTriWhell();

  //if (gapImp < minActGapAll) cout<<"Entrou no ActiveAllIneq";
}
inline void ActiveAllIneq()
{
  //K = auxK;
  // 	TypeHeurClique = TypeHeurWheel = TypeHeurBiWheel = TypeHeurGeClique = TypeTriangle = 1;
  ALL_INEQ_ACTIV = true;
  TypeHeurBiWheel = TypeHeurGeClique = 0;
  TypeHeurClique = TypeHeurWheel = TypeTriangle = 1;
}

inline void DesactivateAll()
{
  ALL_INEQ_ACTIV = false;
  TypeHeurClique = TypeHeurBiWheel = TypeHeurGeClique = TypeHeurWheel = TypeTriangle = 0;
}

inline void ActiveJustTriWhell()
{
  //K = 3;
  ALL_INEQ_ACTIV = false;
  TypeHeurClique = TypeHeurBiWheel = TypeHeurGeClique = 0;
  TypeHeurWheel = TypeTriangle = 1;
}

//Dynamic Inequality size
inline void DynamicIneqSize(const T_Instance &instance, const double &gapImp, int *NBMAX)
{

  if (gapImp < MINGAP)
  {
    if (*NBMAX < instance.DIM * 3)
    {
      *NBMAX = instance.DIM * 2; //(1.0 + RATIOGAP); // INCREASE THE NBMAXINEQ
      DYNAMIC_INEQ = false;
    }
  }
}

void Clean_uselessInequalities_BB(const T_Instance &instance, std::vector<T_constraint> &CONST_BB)
{

  double leftside;
  bool viol = false;
  double toll = 0.0001;
  int counter = 0;

  //printvector(instance.varS);
  //printSolution_screen(instance); cin.get();

  for (int i = 0; i < (int)CONST_BB.size(); ++i)
  {

    if (CONST_BB[i].Origem > 30 && CONST_BB[i].Origem < 50)
      continue;

    viol = false;
    leftside = 0.0;

    for (int j = 0; j < (int)CONST_BB[i].aval.size(); j++)
      leftside += CONST_BB[i].aval[j] * instance.varS[CONST_BB[i].vars[j]];

    if (CONST_BB[i].bkc == MSK_BK_UP)
    { // Bound key (<=)
      if (leftside <= CONST_BB[i].buc - toll)
        viol = true;
    }
    else
    { // Bound key (>=)
      if (leftside >= CONST_BB[i].blc + toll)
        viol = true;
    }

    if (viol)
    {
      CONST_BB.erase(CONST_BB.begin() + i);
      counter++;
      i--;
    }
  } // end FOR i

  //cout << "Eliminated in clean function : "<< counter << endl;
}

void Clean_uselessInequalities(T_Instance &instance)
{
  double leftside;
  bool viol = false;
  double toll = 0.0001;
  int counter = 0;

  //printvector(instance.varS);
  //printSolution_screen(instance); cin.get();

  int random; //= (rand()%5) +5;
  random = 1;

  for (int i = 0; i < (int)instance.CONST.size(); i += random)
  {

    if (instance.CONST[i].Origem >= 30 && instance.CONST[i].Origem < 50)
      continue; /* Node and edge formulations... cannot be removed*/

    viol = false;
    leftside = 0.0;

    for (int j = 0; j < (int)instance.CONST[i].aval.size(); j++)
      leftside += instance.CONST[i].aval[j] * instance.varS[instance.CONST[i].vars[j]];

    //     cout << "leftside = " << leftside << "instance.CONST[i].blc = " << instance.CONST[i].blc << endl ; //cin.get();

    if (instance.CONST[i].bkc == MSK_BK_UP)
    { // Bound key (<=)
      if (leftside <= instance.CONST[i].buc - toll)
        viol = true;
    }
    else
    { // Bound key (>=)
      if (leftside >= instance.CONST[i].blc + toll)
        viol = true;
    }

    if (viol)
    {
      //print_Inequality_Tconstraint(instance.CONST[i]);

      instance.CONST.erase(instance.CONST.begin() + i);
      counter++;
      i--;
    }
  } // end FOR i

  //cout << "Eliminated in clean function : "<< counter << endl;
}

inline void Set_validIneq_in_MainInstance_justEig(T_Instance &destination, const T_Instance &source, std::vector<int> &origen_indice)
{
  double leftside;
  bool viol = false;
  double tol = 0.001;

  //Clean_uselessInequalities (instance);

  for (int i = 0; i < (int)source.CONST.size(); i++)
  {
    leftside = 0.0;
    viol = false;

    //print_Inequality_Tconstraint(source.CONST[i]);
    //cin.get();
    for (int j = 0; j < (int)source.CONST[i].aval.size(); j++)
      leftside += source.CONST[i].aval[j] * source.varS[source.CONST[i].vars[j]];
    if (source.CONST[i].Origem == 9)
    {
      if (source.CONST[i].bkc == MSK_BK_UP)
      { // Bound key (<=)
        if (leftside >= source.CONST[i].buc - tol)
          viol = true;
      }
      else
      { // Bound key (>=)
        if (leftside <= source.CONST[i].blc + tol)
          viol = true;
      }
    }
    viol = true;

    if (viol)
      copy_constraint_i_FromsubGraph(destination, source, i, origen_indice);

  } // END i
}

void Set_validIneq_in_MainInstance(T_Instance &destination, T_Instance &source, std::vector<int> &origen_indice)
{
  double leftside;
  bool viol = false;
  double tol = 0.001;

  Clean_uselessInequalities(source);

  for (int i = 0; i < (int)source.CONST.size(); i++)
  {
    copy_constraint_i_FromsubGraph(destination, source, i, origen_indice);
  } // END i
}

inline void copy_constraint_i_FromsubGraph(T_Instance &destination, const T_Instance &source, const int &idx, std::vector<int> &origen_indice)
{

  //cout << "Entrou aqui" <<endl;
  //cin.get();
  //creating a constraint
  T_constraint Const;

  Const.Origem = source.CONST[idx].Origem; //sdp
  Const.bkc = source.CONST[idx].bkc;       // Bound key (>=)
  Const.blc = source.CONST[idx].blc;       // Numerical value of Lowwer bound
  Const.buc = source.CONST[idx].buc;       // Numerical value of Upper bound

  //resize by number of edge in a complete graph = n*(n-1)/2
  Const.vars.resize(source.CONST[idx].vars.size()); // fix
  Const.aval.resize(source.CONST[idx].aval.size()); // fix

  for (int j = 0; j < (int)source.CONST[idx].aval.size(); j++)
  {
    Const.vars[j] = origen_indice[source.CONST[idx].vars[j]];
    Const.aval[j] = source.CONST[idx].aval[j];
  }

  Const.blc = source.CONST[idx].blc; // Numerical value of Lowwer bound

  //print_Inequality_Tconstraint(Const);
  destination.CONST.push_back(Const);
}

inline void ClearInstance(T_Instance &instance)
{
  instance.cij.barc_j.clear();
  instance.cij.barc_i.clear();
  instance.cij.barc_v.clear();

  instance.CONST.clear();

  instance.indice_cij.clear();

  instance.varS.clear();
}

inline void Optmize_subGraph_onlyEig(T_Instance &instance)
{
  int preSDP_SEP = SDP_SEP;
  int preTr = TypeTriangle;
  int preCl = TypeHeurClique;
  int preGC = TypeHeurGeClique;
  int preWh = TypeHeurWheel;
  int preBW = TypeHeurBiWheel;

  TypeTriangle = 0; //0 or 1
  TypeHeurClique = 1;
  TypeHeurGeClique = 1;
  TypeHeurWheel = 0;
  TypeHeurBiWheel = 0;
  SDP_SEP = 1;

  bool RESUME = true;
  double PrvObj = 0.0;
  ;
  //clear Population of violated inequalities
  clear_AllPopSeparation();

  // **** Mosek enviroment
  MSKrescodee r; //mosek Parameters
  MSKtask_t task = NULL;
  MSKenv_t env = NULL;

  for (int i = 0; i < nb_ITE && RESUME; i++)
  {
    //Optimization and Separation method
    try
    {
      //SolveMosekSDP (instance);
      SolveMosek(instance, r, task, env);

      if (i == 0)
        Obj1 = instance.ObSol;

      RESUME = FindViolation(instance);

      //Set_IneqFuncObj(instance, MAXVALUE);

      //stopping criterias: Time and Iteration improvement
      double gapImp = (PrvObj - instance.ObSol) / PrvObj;
      if (i == 0)
        gapImp = 1.0;
      PrvObj = instance.ObSol;
      if ((getCurrentTime_Double(start) >= MAXTIME_subG) || (gapImp < MinIteImpro_subGraph))
        RESUME = false;

      //write in the iteration file
      //WriteIteration_FILE(instance, file_ITE,FileLecture, i);
      //Print_Iterations_screen(instance, FileLecture,i);

      //Selective separation or Printing final solution !!!
      Selective_Separation(instance);

      //clear Population of violated inequalities
      clear_AllPopSeparation();

      // Delete the task, environment and the associated data.
      MSK_deletetask(&task);
      MSK_deleteenv(&env);
    }
    catch (...)
    {
      cerr << "Unknown Exception" << endl;
    }

  } //end of iterations

  //cout << "Obj1 = " << Obj1;
  //cout << "Final Obj = " << instance.ObSol;
  //cin.get();

  SDP_SEP = preSDP_SEP;
  TypeTriangle = preTr;
  TypeHeurClique = preCl;
  TypeHeurGeClique = preGC;
  TypeHeurWheel = preWh;
  TypeHeurBiWheel = preBW;
}

inline void Optmize_subGraph(T_Instance &instance)
{

  bool RESUME = true;
  double PrvObj = 0.0;
  ;
  //clear Population of violated inequalities
  clear_AllPopSeparation();

  // **** Mosek enviroment
  MSKrescodee r; //mosek Parameters
  MSKtask_t task = NULL;
  MSKenv_t env = NULL;
  clock_t startSub = clock(); // mesure the time

  for (int i = 0; i < nb_ITE && RESUME; i++)
  {
    //Optimization and Separation method
    try
    {

      //SolveMosekSDP (instance);
      SolveMosek(instance, r, task, env);

      if (i == 0)
        Obj1 = instance.ObSol;

      RESUME = FindViolation(instance);

      //Set_IneqFuncObj(instance, MAXVALUE);

      //stopping criterias: Time and Iteration improvement
      double gapImp = (PrvObj - instance.ObSol) / PrvObj;
      if (i == 0)
        gapImp = 1.0;
      PrvObj = instance.ObSol;
      if ((getCurrentTime_Double(startSub) >= MAXTIME_subG) /*|| (gapImp < MinIteImpro_subGraph)*/)
        RESUME = false;

      //write in the iteration file
      //WriteIteration_FILE(instance, file_ITE,FileLecture, i);
      //cout << "Iterations sub_graph "<< endl;
      //Print_Iterations_screen(instance, "Test.txt",i);
      //cin.get();

      //Selective separation or Printing final solution !!!
      Selective_Separation(instance);

      //clear Population of violated inequalities
      clear_AllPopSeparation();

      // Delete the task, environment and the associated data.
      MSK_deletetask(&task);
      MSK_deleteenv(&env);
    }
    catch (...)
    {
      cerr << "Unknown Exception" << endl;
    }

  } //end of iterations

  //cout << "Obj1 = " << Obj1;
  //cout << "Final Obj = " << instance.ObSol;
  //cin.get();
}

inline void Copy_by_rank(const T_Instance &instance, T_Instance &destination, const int &size_max, const int &start,
                         std::vector<int> &origen_indice, const std::vector<int> &ranked_v)
{

  int aux_edge, index, I, J;
  double weight;
  destination.edge_nb = 0; //size_max*(size_max-1)/2; // complet graph
  destination.totalVars = destination.edge_nb;
  destination.indice_cij.resize((size_max + 1) * (size_max + 1), -1);
  destination.ant_indice_cij.resize(((size_max + 1) * (size_max)) / 2); // variables i and j of indice_cij

  destination.DIM = size_max;
  destination.LENBARVAR = (size_max * (size_max + 1) / 2);
  destination.sum_cost = instance.sum_cost;
  destination.varS = instance.varS;
  destination.cst = instance.cst;

  //copying data of instance
  index = 0;
  for (int i = 1; i < size_max; i++)
  {
    I = ranked_v[(i - 1) + start];
    for (int j = i + 1; j <= size_max; j++)
    {

      J = ranked_v[(j - 1) + start];
      aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];

      if (aux_edge != -1)
      {
        weight = instance.cij.barc_v[aux_edge];

        destination.indice_cij[i + j * (destination.DIM + 1)] = index;
        destination.indice_cij[j + i * (destination.DIM + 1)] = index;

        destination.ant_indice_cij[index].idx_j = j; // start from 1
        destination.ant_indice_cij[index].idx_j = i; // in SDP idx j should be the smaller onedf

        origen_indice[index] = aux_edge;

        index++;

        destination.cij.barc_j.push_back(i);
        destination.cij.barc_i.push_back(j);
        destination.cij.barc_v.push_back(weight);

        destination.edge_nb++;
      }

    } //end i and j
  }   // end i
}
inline double GetEdgeWeight(const T_Instance &instance, const int &I, const int &J)
{
  double zero = 0.0;
  int aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];

  if (aux_edge != -1)
  {
    return instance.cij.barc_v[aux_edge];
  }
  else
  {
    return zero;
  }
}

inline void Copy_by_two_SubGraphs(const T_Instance &instance, T_Instance &destination,
                                  const std::vector<int> &begin_v, const std::vector<int> &end_v, const int &frst, const int &scond, std::vector<int> &origen_indice)
{
  if (frst != scond)
  {
    int aux_edge, index, I, J, size_max;
    double weight;

    vector<int> vertexx;

    //puting first and second group in vertex vector
    for (int i = begin_v[frst]; i <= end_v[frst]; i++)
      vertexx.push_back(i);
    for (int i = begin_v[scond]; i <= end_v[scond]; i++)
      vertexx.push_back(i);

    size_max = vertexx.size();

    //cout << "Entrou = "<<size_max << endl;
    //printvector(vertexx);
    //cin.get();

    destination.edge_nb = size_max * (size_max - 1) / 2; // complet graph
    destination.indice_cij.resize((size_max + 1) * (size_max + 1), -1);
    destination.DIM = size_max;

    //copying data of instance
    index = 0;
    for (int i = 1; i < size_max; i++)
    {
      I = vertexx[i - 1];
      for (int j = i + 1; j <= size_max; j++)
      {

        J = vertexx[j - 1];
        aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)]; //indice in the origem

        weight = instance.cij.barc_v[aux_edge];

        destination.indice_cij[i + j * (destination.DIM + 1)] = index;
        destination.indice_cij[j + i * (destination.DIM + 1)] = index;

        origen_indice[index] = aux_edge;

        index++;

        destination.cij.barc_j.push_back(i);
        destination.cij.barc_i.push_back(j);
        destination.cij.barc_v.push_back(weight);

      } //end i and j
    }
  } //END IF if (frst != scond)
}

inline void Copy_instance(const T_Instance &instance, T_Instance &destination, const int &size_max, const int &start, std::vector<int> &origen_indice)
{

  int aux_edge, index, I, J;
  double weight;
  destination.edge_nb = size_max * (size_max - 1) / 2; // complet graph
  destination.indice_cij.resize((size_max + 1) * (size_max + 1), -1);
  destination.DIM = size_max;

  //copying data of instance
  index = 0;
  for (int i = 1; i < size_max; i++)
    for (int j = i + 1; j <= size_max; j++)
    {

      I = i + start;
      J = j + start;
      aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
      weight = instance.cij.barc_v[aux_edge];

      destination.indice_cij[i + j * (destination.DIM + 1)] = index;
      destination.indice_cij[j + i * (destination.DIM + 1)] = index;

      origen_indice[index] = aux_edge;

      index++;

      destination.cij.barc_j.push_back(i);
      destination.cij.barc_i.push_back(j);
      destination.cij.barc_v.push_back(weight);

    } //end i and j
}

inline void clear_AllPopSeparation()
{

  PopBestIneq.clearAll();
  //Cleaning the heap
  PopTriangle.clearAll();
  PopClique.clearAll();
  PopGenClique.clearAll();
  PopWheell.clearAll();
  PopBiWheell.clearAll();
  PopVecProp.clearAll();
}

inline bool FindViolation(T_Instance &instance)
{
  /// Inequalities
  int type;
  int QtMaxIneq = NBMAXINEQ; // QtMaxIneq will recive the max number of inequality for
  int Found = QtMaxIneq;
  double auxtime = 0;

  //cout << "TypeTriangle = " << TypeTriangle << "TypeHeurGeClique = " << TypeHeurGeClique << "TypeHeurClique = " << TypeHeurClique;
  //cout << "TypeHeurWheel =" << TypeHeurWheel << "TypeHeurBiWheel = " << TypeHeurBiWheel;

  //Triangles inequality
  if ((TypeTriangle > 0) && (QtMaxIneq > 0))
  {
    type = 0;
    auxtime = getCurrentTime_Double(start); // mesure the time
    TriangleInequalitySelective(instance, QtMaxIneq, type, PopTriangle);
    Time_TRI += getCurrentTime_Double(start) - auxtime;
  }
  /**/
  //     cout << "Saiu tringulo"; cin.get();
  //General Clique inequality
  if ((TypeHeurGeClique > 0) && (QtMaxIneq > 0))
  {
    type = 2;
    auxtime = getCurrentTime_Double(start); // mesure the time
    GenClique_Inequality(instance, QtMaxIneq, type, PopGenClique);
    Time_GCL += getCurrentTime_Double(start) - auxtime;
  }
  //      cout << "Saiu GEn_Cl"; cin.get();

  //Clique inequality
  if ((TypeHeurClique > 0) && (QtMaxIneq > 0))
  {
    type = 1;
    auxtime = getCurrentTime_Double(start); // mesure the time
    Clique_InequalitySelective_Heuristic(instance, QtMaxIneq, K + 1, 1.0, type, PopClique);
    Time_CLI += getCurrentTime_Double(start) - auxtime;
  }
  //     cout << "Saiu Clique"; cin.get();

  //Wheel inequality
  if ((TypeHeurWheel > 0) && (QtMaxIneq > 0))
  {
    type = 3;
    auxtime = getCurrentTime_Double(start); // mesure the time
    Wheel_InequalitySelective_Heuristics(instance, QtMaxIneq, type, PopWheell);
    Time_WHE += getCurrentTime_Double(start) - auxtime;
  }

  //     cout << "Saiu Wheel"; cin.get();
  //Bicycle Wheel inequality
  if ((TypeHeurBiWheel > 0) && (QtMaxIneq > 0))
  {
    type = 4;
    auxtime = getCurrentTime_Double(start); // mesure the time
    Bi_Wheel_InequalitySelective(instance, QtMaxIneq, type, PopBiWheell);
    Time_BWH += getCurrentTime_Double(start) - auxtime;
  }
  //     cout << "Saiu BW"; cin.get();

  //SDP Violations
  if ((SDP_SEP > 0) || (SDP_SEP == -2))
  {
    type = 5;
    auxtime = getCurrentTime_Double(start); // mesure the time
    //if ((PopBiWheell.size() + PopWheell.size()+PopClique.size() + PopGenClique.size() + PopTriangle.size()) == 0)
    SDP_violations_in_LPSOlution(instance, QtMaxIneq, PopVecProp, type);

    Time_SDPCUT += getCurrentTime_Double(start) - auxtime;
  }

  Found -= QtMaxIneq;

  //     cout << "Found = " << Found << endl;cin.get();
  if (!SepMethod2)
  {
    if (Found > 0)
      return true;
    else
      return false;
  }
  else if (PopBestIneq.size() > 0)
    return true;
  else
    return false;
}

void SolveMosekSDP(T_Instance &instance)
{

  //cout << "Entrou otimo  SDP ...";

  double auxtime = getCurrentTime_Double(start);
  //cout << "Chegou" ; cin.get();
  MSKenv_t env = NULL;
  MSKtask_t task = NULL;
  bool rept = true;
  MSKrescodee r;
  //double 	cstW = (-1.0)*((K-1.0)/K); 	/*Poid que matrix W va etres  = (-(k-1)/k)*/
  MSKint64t idx;
  double *xx, *Obj, *barx;

  double falpha = 1.0; //peso de cada matrix A sera 1.0

  //Number of constraints total to SDP (remember that we have xij<-1/(k-1) and xii = 1)
  MSKint32t NUMCONS;
  if (SDP_EdGE_TYPE)
    NUMCONS = instance.DIM + sdpEdgeConst.size + instance.CONST.size();
  else
    NUMCONS = instance.DIM + instance.edge_nb + instance.CONST.size();

  MSKint32t NUMBARVAR = 1;

  MSKint32t DIMBARVAR[] = {instance.DIM}; /* Dimension of semidefinite cone */

  //NUMCONS= instance.DIM+ instance.LENBARVAR;

  /* Create the mosek environment. */
  r = MSK_makeenv(&env, NULL);

  /* Create the optimization task. */
  if (r == MSK_RES_OK)
    r = MSK_maketask(env, NUMCONS, 0, &task); // change the NUMCONS

  //Print in screen Interior point iteration and the error raport (it it exist)
  //MSK_linkfunctotaskstream(task,MSK_STREAM_LOG,NULL,printstr);

  ////* Append 'NUMCON' empty constraints.
  // The constraints will initially have no bounds.
  if (r == MSK_RES_OK)
    r = MSK_appendcons(task, NUMCONS);

  // Append 'NUMBARVAR' semidefinite variables.
  if (r == MSK_RES_OK)
  {
    r = MSK_appendbarvars(task, NUMBARVAR, DIMBARVAR);
  }

  //// Lower and upper bounds of SDP variables
  //double LBsdp = 0.0;// before was -1.0/(K-1.0);
  double LBsdp = -1.0 / (K - 1.0);
  double UBsdp = 1.0;

  //
  ////
  ////////		Setting the Objective function
  /////////
  Set_ObFunction_Mosek_SDP(instance, r, task, LBsdp);
  if (r != MSK_RES_OK)
    cout << "Erro aqui 0";

  //
  ////
  ////////		Setting the constraints
  /////////

  if (r != MSK_RES_OK)
    cout << "Erro aqui 1 .... before Set_CombinatorialConstraints_MOSEK_SDP ";

  Set_CombinatorialConstraints_MOSEK_SDP(instance, r, task, LBsdp, UBsdp);
  if (r != MSK_RES_OK)
  {
    cout << "Nb of instances = " << instance.CONST.size();
    cout << "Erro aqui 1 .... after Set_CombinatorialConstraints_MOSEK_SDP ";
  }

  //
  ////
  /////	SDP constraint
  //////
  Set_OriginalConstraints_MOSEK_SDP(instance, r, task, LBsdp); // must be executed after Set_CombinatorialConstraints_MOSEK_SDP
  if (r != MSK_RES_OK)
    cout << "Erro aqui";

  ////
  //////
  //////	SOLVING
  ////////

  // Maximize objective function.
  if (r == MSK_RES_OK)
    r = MSK_putobjsense(task, MSK_OBJECTIVE_SENSE_MAXIMIZE);

  /*
    r = MSK_putdouparam(task,MSK_DPAR_INTPNT_CO_TOL_MU_RED, 0.999); //Relative complementarity gap tolerance.
    r = MSK_putdouparam(task,MSK_DPAR_INTPNT_CO_TOL_PFEAS, 0.99); // primal feasibility
    r = MSK_putdouparam(task,MSK_DPAR_INTPNT_CO_TOL_DFEAS, 0.00001); // dual feasibility
    r = MSK_putdouparam(task,MSK_DPAR_INTPNT_CO_TOL_REL_GAP, 0.999); // Tol to optimality
*/

  MSK_putintparam(task, MSK_IPAR_NUM_THREADS, num_of_threads); //Nber of cpus
  double FOvalue;

  if (r == MSK_RES_OK)
  {

    MSKrescodee trmcode;

    // Run optimizer
    r = MSK_optimizetrm(task, &trmcode);

    //MSK_solutionsummary (task,MSK_STREAM_MSG);
    if (r == MSK_RES_OK)
    {
      MSKsolstae solsta;

      MSK_getsolsta(task, MSK_SOL_ITR, &solsta);

      switch (solsta)
      {
      case MSK_SOL_STA_OPTIMAL:
        //MSKint32t NUMVAR 	= 0;
        xx = (double *)MSK_calloctask(task, 0, sizeof(MSKrealt));
        barx = (double *)MSK_calloctask(task, instance.LENBARVAR, sizeof(MSKrealt));
        Obj = (double *)MSK_calloctask(task, 2, sizeof(MSKrealt));
        double *Obj2;
        Obj2 = (double *)MSK_calloctask(task, 2, sizeof(MSKrealt));

        MSK_getxx(task,
                  MSK_SOL_ITR,
                  xx);
        MSK_getbarxj(task,
                     MSK_SOL_ITR, /* Request the interior solution. */
                     0,
                     barx);

        /*
	    //primal or dual obj
	    if (TOL != 40)  MSK_getprimalobj(task, MSK_SOL_ITR, Obj);
	    else	    MSK_getdualobj(task, MSK_SOL_ITR, Obj); // get dual as solution
			  */

        MSK_getprimalobj(task, MSK_SOL_ITR, Obj);

        //Get dual
        //MSK_getdualobj(task, MSK_SOL_ITR, Obj2);

        //cout << "Primal =" << Obj[0] << ", dual " << Obj2[0];
        //cin.get();

        if ((SDP_EdGE_TYPE) && ((SDP_SEP == -1) || (SDP_SEP == -3))) // just for SDP solver
          Find_edgeViolated_in_SDP(instance, barx);

        if (DO_EIG_INEQ_FROM_SDP)
          FindNeg_EigFromSDP(barx, instance);

        //Setting soluition in instance but before we have to
        //Transform the soluition in 0,1 ... because all the separations are based on 0,1 variables.
        Transform_SDPsol_2_LPsol(instance, barx, LBsdp, UBsdp);

        //printSolution_screen(instance); cin.get();

        if ((!SDP_EdGE_TYPE))
        {
          FOvalue = ManualCalculeOfObjFunc(instance);

          if ((FOvalue < Obj[0] - epslon) || (FOvalue > Obj[0] + epslon))
            cout << "Erro in ManualCalculeOfObjFunc, its value is diff from Obj[0], FOvalue = " << FOvalue << " and Obj[0]=" << Obj[0];
        }

        instance.ObSol = Obj[0];

        break; //end of switch
      case MSK_SOL_STA_DUAL_INFEAS_CER:
      case MSK_SOL_STA_PRIM_INFEAS_CER:
        printf("Primal or dual infeasibility certificate found.\n");
        break;

      case MSK_SOL_STA_UNKNOWN:
        printf("The status of the solution could not be determined.\n");
        break;
      default:
        printf("Other solution status.");
        break;
      } //end switch
    }
  }
  else
  {
    cout << "Erro before optimizing ( r !=  MSK_RES_OK) ... erro implementation while adding SDP constraints in SolveMosekSDP";
    exit(1);
  }

  r = MSK_getdouinf(task, MSK_DINF_OPTIMIZER_TIME, &time_IPM_iteration);

  //free memory
  if (r == MSK_RES_OK)
  {
    MSK_deletesolution(task, MSK_SOL_ITR); // clean solution (can change the sdp performance ??)
    MSK_freetask(task, xx);
    MSK_freetask(task, barx);
    MSK_freetask(task, Obj);
    MSK_deletetask(&task);
    MSK_deleteenv(&env);
  }

  Time_IPM += time_IPM_iteration; // sum time of IPM
}

//complementarity gap
void SolveMosekSDP2(T_Instance &instance)
{

  double auxtime = getCurrentTime_Double(start);
  //cout << "Chegou" ; cin.get();
  MSKenv_t env = NULL;
  MSKtask_t task = NULL;
  bool rept = true;
  MSKrescodee r;
  //double 	cstW = (-1.0)*((K-1.0)/K); 	/*Poid que matrix W va etres  = (-(k-1)/k)*/
  MSKint64t idx;
  double *xx, *Obj, *barx;

  double falpha = 1.0; //peso de cada matrix A sera 1.0

  //Number of constraints total to SDP (remember that we have xij<-1/(k-1) and xii = 1)
  MSKint32t NUMCONS;
  if (SDP_EdGE_TYPE)
    NUMCONS = instance.DIM + sdpEdgeConst.size + instance.CONST.size();
  else
    NUMCONS = instance.DIM + instance.edge_nb + instance.CONST.size();

  MSKint32t NUMBARVAR = 1;

  MSKint32t DIMBARVAR[] = {instance.DIM}; /* Dimension of semidefinite cone */

  //NUMCONS= instance.DIM+ instance.LENBARVAR;

  /* Create the mosek environment. */
  r = MSK_makeenv(&env, NULL);

  /* Create the optimization task. */
  if (r == MSK_RES_OK)
    r = MSK_maketask(env, NUMCONS, 0, &task); // change the NUMCONS

  //Print in screen Interior point iteration and the error raport (it it exist)
  //MSK_linkfunctotaskstream(task,MSK_STREAM_LOG,NULL,printstr);

  ////* Append 'NUMCON' empty constraints.
  // The constraints will initially have no bounds.
  if (r == MSK_RES_OK)
    r = MSK_appendcons(task, NUMCONS);

  // Append 'NUMBARVAR' semidefinite variables.
  if (r == MSK_RES_OK)
  {
    r = MSK_appendbarvars(task, NUMBARVAR, DIMBARVAR);
  }

  //// Lower and upper bounds of SDP variables
  //double LBsdp = 0.0;// before was -1.0/(K-1.0);
  double LBsdp = -1.0 / (K - 1.0);
  double UBsdp = 1.0;

  //
  ////
  ////////		Setting the Objective function
  /////////
  Set_ObFunction_Mosek_SDP(instance, r, task, LBsdp);

  //
  ////
  ////////		Setting the constraints
  /////////

  Set_CombinatorialConstraints_MOSEK_SDP(instance, r, task, LBsdp, UBsdp);

  //
  ////
  /////	SDP constraint
  //////
  Set_OriginalConstraints_MOSEK_SDP(instance, r, task, LBsdp); // must be executed after Set_CombinatorialConstraints_MOSEK_SDP

  ////
  //////
  //////	SOLVING
  ////////

  // Maximize objective function.
  if (r == MSK_RES_OK)
    r = MSK_putobjsense(task, MSK_OBJECTIVE_SENSE_MAXIMIZE);

  //GAP of Interior point method
  MSKrealt gapTol = 0.2;

  //Dynamicfddf
  if ((gapImp < MINGAP) && (gapTolRel > 0.5))
  {
    gapTolRel *= 0.25;          //Divided by 4
    MINGAP = MINGAP * RATIOGAP; //Divided by 2
    gapImp = 1.0;               //reset gap of Improvement
  }

  r = MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_MU_RED, gapTol);           //Relative complementarity gap tolerance.
  r = MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_PFEAS, gapPrimal);         // primal feasibility
  r = MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_DFEAS, gapTol);            // dual feasibility
  r = MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_REL_GAP, gapTolRel / 100); // Tol to optimality

  MSK_putintparam(task, MSK_IPAR_NUM_THREADS, num_of_threads); //Nber of cpus
  double FOvalue;

  if (r == MSK_RES_OK)
  {

    MSKrescodee trmcode;

    // Run optimizer
    r = MSK_optimizetrm(task, &trmcode);

    //MSK_solutionsummary (task,MSK_STREAM_MSG);
    if (r == MSK_RES_OK)
    {
      MSKsolstae solsta;

      MSK_getsolsta(task, MSK_SOL_ITR, &solsta);

      switch (solsta)
      {
      case MSK_SOL_STA_OPTIMAL:
        //MSKint32t NUMVAR 	= 0;
        xx = (double *)MSK_calloctask(task, 0, sizeof(MSKrealt));
        barx = (double *)MSK_calloctask(task, instance.LENBARVAR, sizeof(MSKrealt));
        Obj = (double *)MSK_calloctask(task, 2, sizeof(MSKrealt));
        double *Obj2;
        Obj2 = (double *)MSK_calloctask(task, 2, sizeof(MSKrealt));

        MSK_getxx(task,
                  MSK_SOL_ITR,
                  xx);
        MSK_getbarxj(task,
                     MSK_SOL_ITR, /* Request the interior solution. */
                     0,
                     barx);

        /*
	    //primal or dual obj
	    if (TOL != 40)  MSK_getprimalobj(task, MSK_SOL_ITR, Obj);
	    else	    MSK_getdualobj(task, MSK_SOL_ITR, Obj); // get dual as solution
			  */

        MSK_getprimalobj(task, MSK_SOL_ITR, Obj);

        //Get dual
        //MSK_getdualobj(task, MSK_SOL_ITR, Obj);

        //cout << "Primal =" << Obj[0] << ", dual " << Obj2[0];
        //cin.get();

        if ((SDP_EdGE_TYPE) && ((SDP_SEP == -1) || (SDP_SEP == -3))) // just for SDP solver
          Find_edgeViolated_in_SDP(instance, barx);

        //Setting soluition in instance but before we have to
        //Transform the soluition in 0,1 ... because all the separations are based on 0,1 variables.
        Transform_SDPsol_2_LPsol(instance, barx, LBsdp, UBsdp);

        //printSolution_screen(instance); cin.get();

        if ((!SDP_EdGE_TYPE))
        {
          FOvalue = ManualCalculeOfObjFunc(instance);

          if ((FOvalue < Obj[0] - epslon) || (FOvalue > Obj[0] + epslon))
            cout << "Erro in ManualCalculeOfObjFunc, its value is diff from Obj[0]";
        }

        instance.ObSol = Obj[0];

        break; //end of switch
      case MSK_SOL_STA_DUAL_INFEAS_CER:
      case MSK_SOL_STA_PRIM_INFEAS_CER:
        printf("Primal or dual infeasibility certificate found.\n");
        break;

      case MSK_SOL_STA_UNKNOWN:
        printf("The status of the solution could not be determined.\n");
        break;
      default:
        printf("Other solution status.");
        break;
      } //end switch
    }
  }
  else
  {
    cout << "Erro before optimizing ( r !=  MSK_RES_OK) ... erro implementation while adding SDP constraints in SolveMosekSDP";
    exit(1);
  }
  r = MSK_getdouinf(task, MSK_DINF_OPTIMIZER_TIME, &time_IPM_iteration);

  //free memory
  if (r == MSK_RES_OK)
  {
    MSK_deletesolution(task, MSK_SOL_ITR); // clean solution (can change the sdp performance ??)
    MSK_freetask(task, xx);
    MSK_freetask(task, barx);
    MSK_freetask(task, Obj);
    MSK_deletetask(&task);
    MSK_deleteenv(&env);
  }

  Time_IPM += time_IPM_iteration; // sum time of IPM
}

inline void printSolution_screen(const T_Instance &instance)
{
  cout << "Printing solution:" << endl;
  int counter = 0;
  for (int i = 0; i < instance.DIM; i++)
    for (int j = i + 1; j < instance.DIM; j++)
    {
      cout << instance.indice_cij[i + 1 + (j + 1) * (instance.DIM + 1)] << ": x_(" << i + 1 << "," << j + 1 << ") = " << ValueXij(i + 1, j + 1, instance) << endl;
      counter++;
    }
}

inline void Set_CombinatorialConstraints_MOSEK_SDP(const T_Instance &instance, MSKrescodee &r, MSKtask_t &task, const double &LBsdp, const double &UBsdp)
{
  int nb;
  MSKint64t idx;
  double falpha = 1.0; //peso de cada matrix A sera 1.0

  MSKint32t *a_i; // index i of variable
  MSKint32t *a_j; // index j of vairable
  double *vl;     // value of aij
  double *vl2;
  double blc; // lower bound of constraint
  double buc; // upper bound of constraint

  for (int i = 0; i < (int)instance.CONST.size() && r == MSK_RES_OK; i++)
  {

    blc = instance.CONST[i].blc; // lower bound of constraint
    buc = instance.CONST[i].buc; // upper bound of constraint

    //Alloc memory
    nb = (int)instance.CONST[i].aval.size(); // number of non-zero variables in the constraint
    a_i = new MSKint32t[nb + 1];
    a_j = new MSKint32t[nb + 1];
    vl = new double[nb + 1];
    vl2 = new double[nb + 1];

    //Have to fix the problem here.
    for (int j = 0; j < nb; j++)
    { // setting value in allocated vectors
      a_i[j] = instance.ant_indice_cij[instance.CONST[i].vars[j]].idx_i - 1;
      a_j[j] = instance.ant_indice_cij[instance.CONST[i].vars[j]].idx_j - 1;
      vl[j] = instance.CONST[i].aval[j]; // /2.0; // in SDP we should divide for symmetric matri
    }

    // In the separation methods we use the LP variables it is 0,1 variables
    //tranforme (0,1) to (Lb, Ub) variables .... mostly LP (0,1) to (-1/(k-1), 1) SDP
    if (instance.CONST[i].bkc == MSK_BK_LO) // Bound key (>=)
      Transforme_LpIneq01_in_SDPineq(nb, LBsdp, UBsdp, vl, blc);
    else
      Transforme_LpIneq01_in_SDPineq(nb, LBsdp, UBsdp, vl, buc);

    /*double sum2 = 0.0;
      double sum1 = 0.0;
      for (int j=0; j<nb; j++ ){
	sum2 += vl[j]*((double)(K)/(K-1.0)*ValueXij(a_i[j]+1,a_j[j]+1,instance) - 1.0/(K-1.0));
	sum1 += instance.CONST[i].aval[j]*ValueXij(a_i[j]+1,a_j[j]+1,instance);
      }
        */
    for (int j = 0; j < nb; j++)
      vl[j] *= 0.5; // we must multiply for 0.5 each element of SDP because they are from symmetric matrix

    //right size (ARE THE BOUNDS OF CONSTRAINTS)
    r = MSK_putconbound(task,
                        i,                     // Index of constraint.
                        instance.CONST[i].bkc, // Bound key.
                        blc,                   // Numerical value of lower bound.
                        buc);                  // Numerical value of upper bound.

    //Left size (the variables and index)
    if (r == MSK_RES_OK)
      r = MSK_appendsparsesymmat(task,
                                 instance.DIM,
                                 nb,
                                 a_i,
                                 a_j,
                                 vl,
                                 &idx);
    if (r == MSK_RES_OK)
      r = MSK_putbaraij(task, i, 0, 1, &idx, &falpha);

    //free memory
    delete[] a_i;
    delete[] a_j;
    delete[] vl;
  } //end i
} //end Set_CombinatorialConstraints_MOSEK_SDP

inline void Set_OriginalConstraints_MOSEK_SDP(const T_Instance &instance, MSKrescodee &r, MSKtask_t &task, const double &LBsdp)
{
  MSKint64t idx;
  double falpha = 1.0; //peso de cada matrix A sera 1.0

  // Add the first row of barA  (X_ii = 1)
  int counter = (int)instance.CONST.size();

  for (int i = 0; i < instance.DIM && r == MSK_RES_OK; ++i)
  {

    MSKboundkeye bkc = MSK_BK_FX; // It is equality key (=)
    double blc = 1.0;

    //right size (ARE THE BOUNDS OF CONSTRAINTS)
    r = MSK_putconbound(task,
                        counter, // Index of constraint.
                        bkc,     // Bound key.
                        blc,     // Numerical value of lower bound.
                        blc);    // Numerical value of upper bound.

    //left size
    MSKint32t var = (double)i;
    double vall = 1.0;
    if (r == MSK_RES_OK)
      r = MSK_appendsparsesymmat(task,
                                 instance.DIM,
                                 1,
                                 &var,
                                 &var,
                                 &vall,
                                 &idx);

    if (r == MSK_RES_OK)
      r = MSK_putbaraij(task, counter, 0, 1, &idx, &falpha);

    counter++;
  }
  //End of diagonal constraint

  //Add constraint of barA (X_ij >= (-1)/(k-1))
  // This constraint is based on the lower bound of SDP variables LBsdp it is activated if LBsdp != -1
  if ((LBsdp != -1.0) && (!SDP_EdGE_TYPE))
  {
    for (int j = 0; j < instance.DIM; j++)
      for (int i = j + 1; i < instance.DIM && r == MSK_RES_OK; i++)
      {

        MSKboundkeye bkc = MSK_BK_LO; //>=

        //right size (ARE THE BOUNDS OF CONSTRAINTS)
        r = MSK_putconbound(task,
                            counter,        // Index of constraint.
                            bkc,            // Bound key.
                            LBsdp,          // Numerical value of lower bound.
                            +MSK_INFINITY); // Numerical value of upper bound.

        //left size
        MSKint32t var_i = (MSKint32t)i;
        MSKint32t var_j = (MSKint32t)j;
        double vall = 0.50;

        if (r == MSK_RES_OK)
          r = MSK_appendsparsesymmat(task,
                                     instance.DIM,
                                     1,
                                     &var_i,
                                     &var_j,
                                     &vall,
                                     &idx);

        if (r == MSK_RES_OK)
          r = MSK_putbaraij(task, counter, 0, 1, &idx, &falpha);

        counter++;

        if (r != MSK_RES_OK)
        {
          cout << "i=" << i << " j=" << j << endl;
          cout << endl
               << "r = " << r;
          cin.get();
        }
      } //end i and j
  }
  else
  { //new way of

    for (int j = 0; j < sdpEdgeConst.size; j++)
    {

      MSKboundkeye bkc = MSK_BK_LO; //>=

      //right size (ARE THE BOUNDS OF CONSTRAINTS)
      r = MSK_putconbound(task,
                          counter,        // Index of constraint.
                          bkc,            // Bound key.
                          LBsdp,          // Numerical value of lower bound.
                          +MSK_INFINITY); // Numerical value of upper bound.

      //left size
      MSKint32t var_i = (MSKint32t)sdpEdgeConst.varI[j];
      MSKint32t var_j = (MSKint32t)sdpEdgeConst.varJ[j];
      double vall = 0.50;

      if (r == MSK_RES_OK)
        r = MSK_appendsparsesymmat(task,
                                   instance.DIM,
                                   1,
                                   &var_i,
                                   &var_j,
                                   &vall,
                                   &idx);

      if (r == MSK_RES_OK)
        r = MSK_putbaraij(task, counter, 0, 1, &idx, &falpha);

      counter++;

      if (r != MSK_RES_OK)
      {
        cout << "Dim =" << instance.DIM;
        cout << " j=" << j << endl;
        cout << endl
             << "r = " << r;
        cin.get();
      }

    } //end for j
  }   //end of else
} //end Set_OriginalConstraints_MOSEK_SDP

inline void Find_edgeViolated_in_SDP(const T_Instance &instance, double *barx)
{
  sdpEdgeConst.newadd = 0;
  long aux_edge;

  double minval = -1.0 / ((double)K - 1.0),
         valSDP;
  int I, J;
  int qtIneq = NBINEQSDP;

  for (int j = 0; j < instance.DIM && sdpEdgeConst.newadd < qtIneq; j++)
    for (int i = j + 1; i < instance.DIM && sdpEdgeConst.newadd < qtIneq; i++)
    {

      I = i + 1;
      J = j + 1;

      aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
      if (aux_edge != -1)
      {
        valSDP = getValueXij_SDP(i, j, barx, instance.DIM);

        if (valSDP <= minval - epslon)
        { // violation
          sdpEdgeConst.newadd++;
          sdpEdgeConst.size++;
          sdpEdgeConst.varJ.push_back(j);
          sdpEdgeConst.varI.push_back(i);
        }
      } //end aux_edge
    }

  //ENDFUNCFIND:
  //int a =0;
  //cout << "sdpEdgeConst.newadd = " << sdpEdgeConst.newadd << endl;
} //end function find edge_Violation

inline void Clear_edgeNotViolated_in_SDP(const T_Instance &instance)
{
  //erase from globa variable sdpEdgeConst the edges that are no longer important for formulation
  double valSDP;
  long aux_edge;
  int I, J, counter = 0;

  for (int s = 0; s < sdpEdgeConst.size; ++s)
  {
    J = sdpEdgeConst.varJ[s] + 1;
    I = sdpEdgeConst.varI[s] + 1;
    aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];

    valSDP = instance.varS[aux_edge];

    if (valSDP > 0.1)
    { //I may errase if value is close to 1
      sdpEdgeConst.varJ.erase(sdpEdgeConst.varJ.begin() + s);
      sdpEdgeConst.varI.erase(sdpEdgeConst.varI.begin() + s);
      --s;
      if (sdpEdgeConst.size > 0)
        sdpEdgeConst.size--;
      else
        return void();
    }
  } //end for size

  //cout << "counter = " << counter<<endl;
  //ENDFUNCFIND:
  //int a =0;
  //cout << "sdpEdgeConst.newadd = " << sdpEdgeConst.newadd << endl;
}

inline void Set_ObFunction_Mosek_SDP(const T_Instance &instance, MSKrescodee &r, MSKtask_t &task, const double &LBsdp)
{

  

  MSKint64t idx;

  double cstW = -1.0; // before (-1.0)*((K-1.0)/K); 	/*Poid que matrix W va etres  = (-(k-1)/k)*/
  double sumTotalCost = 0.0;
  MSKint32t *c_i;
  MSKint32t *c_j;
  double *vl;

  if (LBsdp != 0.0)
    cstW = (-1.0) * ((K - 1.0) / K);
  else
    cstW = -1.0;

  //memory alloc
  int nb = (int)instance.cij.barc_i.size();
  c_i = new MSKint32t[nb + 1];
  c_j = new MSKint32t[nb + 1];
  vl = new double[nb + 1];

  //Remember thqt cij should start by 0 until DIm -1 (that is why I put -1 )
  for (int j = 0; j < nb; j++)
  { // setting value in allocated vectors
    c_i[j] = instance.cij.barc_i[j] - 1;
    c_j[j] = instance.cij.barc_j[j] - 1;
    sumTotalCost += instance.cij.barc_v[j];
    vl[j] = instance.cij.barc_v[j] / 2.0; // in SDP we should divide for symmetric matrix
  }

  if (r == MSK_RES_OK)
    r = MSK_appendsparsesymmat(task,
                               instance.DIM,
                               instance.edge_nb, /* qt de non zero triangulo inferior*/
                               c_i,
                               c_j,
                               vl,
                               &idx);
  //
  if (r == MSK_RES_OK)
    r = MSK_putbarcj(task, 0, 1, &idx, &cstW);

  // Optionally add a constant term to the objective.
  if (cstW != 1.0)
    if (r == MSK_RES_OK)
      r = MSK_putcfix(task, -cstW * sumTotalCost + instance.sum_cost); // in sdp we have to add the sum of all the cost in the objective function (bfore sumTotalCost = 0.0)
  
  //free memory
  delete[] c_i;
  delete[] c_j;
  delete[] vl;
} //end Set_ObFunction_Mosek_SDP

inline double ManualCalculeOfObjFunc(T_Instance &instance)
{
  double sum = 0.0;

  for (unsigned i = 0; i < instance.edge_nb; i++)
    sum += instance.cij.barc_v[i] * (1 - instance.varS[i]);

  return sum + instance.sum_cost;
}

inline double getValueXij_SDP(const int &i, const int &j, double *barx, const int &DIM)
{
  int smaller, bigger, AuxJ;

  if (i < j)
  {
    smaller = i;
    bigger = j;
  }
  else
  {
    smaller = j;
    bigger = i;
  }

  AuxJ = 0;
  for (int w = 0; w <= smaller; w++)
    AuxJ += w;

  return barx[bigger + smaller * (DIM)-AuxJ];
}

inline void Transform_SDPsol_2_LPsol(T_Instance &instance, double *barx, const double &LBsdp, const double &UBsdp)
{
  //from  X = (Lb, Ub) to lp y = (0,1)
  //	From line equation We have:
  //		y = (X - Lb)/(Ub - Lb),
  //		  = X/(Ub - Lb) -  Lb/(Ub - Lb)
  //
  //

  double minvalSDP = -1.0 / ((double)K - 1.0),
         maxvalSDP = 1.0;

  instance.varS.clear();
  instance.varS.resize(instance.edge_nb);

  int aux_edge, I, J;
  double valSDP;
  double divCst = (UBsdp - LBsdp);

  for (int j = 0; j < instance.DIM; j++)
    for (int i = j + 1; i < instance.DIM; i++)
    {

      I = i + 1;
      J = j + 1;
      valSDP = getValueXij_SDP(i, j, barx, instance.DIM);
      //double newval = (valSDP - LBsdp)/divCst;
      aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];

      if (aux_edge != -1)
      {
        //cout << "newval =" << newval ; cin.get();

        if (valSDP < minvalSDP)
          valSDP = minvalSDP;
        if (valSDP > maxvalSDP)
          valSDP = maxvalSDP;

        instance.varS[aux_edge] = (valSDP - LBsdp) / divCst; // y = (X - Lb)/(Ub - Lb),
      }                                                      //end aux_edge
    }
}

inline void Transforme_LpIneq01_in_SDPineq(const int &nb, const double &LBsdp, const double &UBsdp, double *vl, double &bound)
{
  //from lp y = (0,1) we want to pass to SDP where X = (Lb, Ub)
  //	From line equation We have:
  //		y = (X - Lb)/(Ub - Lb),
  //		  = X//(Ub - Lb) -  Lb/(Ub - Lb)
  //
  //		Thus if we have vl*y, we transfor vl[i] = vl[i]*(1/(Ub - Lb)) and  bound = bound + Sum_i (vl[i]*(Lb/(Ub - Lb))).... end of tranformation

  double divCst = (UBsdp - LBsdp);
  double sum = 0.0;

  for (int j = 0; j < nb; j++)
  {

    sum += vl[j] * (LBsdp / divCst);

    vl[j] *= 1.0 / divCst;
  }

  bound += sum; // I must verify if it is + or -
}

void SDP_violations_in_LPSOlution(T_Instance &instance, int &NbIneqMax, MKC_ConstraintLPtoSDPPopulate &PopSDP, const int &type)
{
  double rh_cst,
      vio; //violation or here it is eigenvalue
  vector<double> aux_vec;

  //Finding the vector and valeu propres
  //FindNegatifEigenvaleuSubGraph(instance, PopSDP);
  //FindNegatifEigenvaleuSubGraph_triangle(instance, PopSDP);
  //FindNegatifEigenvaleu(instance, PopSDP);

  //test if there are any negative eigenvalue
  //if (PopSDP.size() > 0)
  if (SDP_SEP == 6)
  {
    Gershgorin_Circle(instance, PopSDP);
  }
  else if (ISCOMPLETE_GRAPH)
  {
    FindNegatifEigenvaleu(instance, PopSDP);
  }
  else
  {
    FindNegatifEigenvaleu_Chordal(instance, PopSDP);
  }

  //   //creat more violate Eigenvalue
  //   if (SDP_SEP == 3)
  //     Heuristic_to_ValidEigenvalues(instance, PopSDP);

  //   if (SDP_SEP == 4)
  //    Method_to_ValidEigenvalues_Miguel(instance, PopSDP);
  //
  //   if (SDP_SEP == 5)
  //    Method_to_ValidEigenvalues_Vilmar(instance, PopSDP);

  //if (SDP_SEP == 1)
  //Gershgorin_Circle(instance, PopSDP);

  // Desing of inequality

  //PopSDP.printAll();
  //cin.get();

  //Design Inequlities
  int MAX = NbIneqMax;
  for (int i = 0; i < PopSDP.size() && i < MAX; i++)
  {
    //PopSDP.Get_Vector_and_Value(i, aux_vec, vio);
    //printvector(aux_vec);
    rh_cst = 0.0;

    if (SepMethod2)
    {
      vio = PopSDP.Get_eigVal(i);
      //set_IneqData_in_SepPop(origem_separation,origem_Position, violation, b (right side ineq) )
      PopBestIneq.set_IneqData_in_SepPop(type, i, -10 * vio, rh_cst);
    }
    else
    {
      aux_vec = PopSDP.GetVector(i);
      Design_SDPIneq2(instance, aux_vec);
      NbIneqMax--;
    }
  }
}

inline double getCurrentTime_Double(clock_t &time)
{
  return (double)(clock() - time) / CLOCKS_PER_SEC;
}

inline bool Find_ViolatedTri_in_eigenvalue(const T_Instance &instance, std::vector<int> &aux_vec, const int &counter, const int &h)
{

  if (counter >= 2)
  {
    for (int i = 0; i < counter; i++)
      for (int j = i + 1; j < counter; j++)
        if (Find_ViolatedTri_in_wheel(instance, aux_vec[i] + 1, aux_vec[j] + 1, h + 1))
          return true;

  } //end IF size

  return false;
}

void FindNeg_EigFromSDP(double *barx, T_Instance &instance)
{
  MinVP = 1;
  MatrixXd mat_X(instance.DIM, instance.DIM);
  VectorXd vecXd(instance.DIM);
  vector<double> egVec(instance.DIM, 0.0);
  vector<double> eigvect(instance.edge_nb);
  double eigVal = -1;
  int counterNeg = 0;
  SelfAdjointEigenSolver<MatrixXd> es; // just for symmetric matrix (comp. it is 10 times faster than general method)
  double cstTranf = 1.0 / (K - 1);

  //Tranformation x to X
  for (int i = 0; i < instance.DIM; i++)
    for (int j = i; j < instance.DIM; j++)
    {
      if (i != j)
      {
        mat_X(i, j) = getValueXij_SDP(i, j, barx, instance.DIM);
        mat_X(j, i) = getValueXij_SDP(i, j, barx, instance.DIM);
      }
      else // Diagonal values
        mat_X(i, j) = 1.0;
    }

  //	cout << mat_X;

  //Compute the eigenvalue and eigenvectr
  es.compute(mat_X);

  /*// Iteration of all eigenvalues /*/
  for (int i = 0; i < instance.DIM; ++i)
  {
    eigVal = es.eigenvalues()[instance.DIM - 1 - i];
    //	  cout << "eigVal = " << eigVal; cin.get();
    if (eigVal >= epslon)
    { // eigenvalue is negatif, so there is a violation of the Semidefiniteness
      counterNeg++;
      vecXd = es.eigenvectors().col(i);
      for (int z = 0; z < instance.DIM; ++z)
        egVec[z] = vecXd[z];

      Design_SDPIneq2(instance, egVec);

      //		PopVecProp.SetIneq(es.eigenvectors().col(i), instance.DIM,  eigVal, false); // add new eigenvector in populations
    }
    else
    {
      return; //end of function // The eigenvalues  are ranked by non-decrese value
    }
  } //end for

  //	vector<double> aux_vec;
  //	for (int i=0; i<PopVecProp.size()  ; i++){
  //      aux_vec = PopVecProp.GetVector(i);
  //      Design_SDPIneq2(instance, aux_vec);
  //  	}

  //	PopVecProp.clearAll();
}

void FindNegatifEigenvaleu_Chordal(T_Instance &instance, MKC_ConstraintLPtoSDPPopulate &PopVecProp)
{
  for (unsigned i = 0; i < maximalCliques.size(); ++i)
    Eigenvaleu_ForSomeVertices(instance, PopVecProp, maximalCliques[i], 0.05);
}

//Calcule of eignevalue for some vertices indicated in Vec_Vertices
//We are using the EigeSolver to solve eigenvalue decomposition of a symmetric matrix
void Eigenvaleu_ForSomeVertices(T_Instance &instance, MKC_ConstraintLPtoSDPPopulate &PopVecProp, const std::vector<int> &Vec_Vertices, const double &frac)
{
  //  cout << "Begin Eigenvaleu_ForSomeVertices"; cin.get();
  //calcule eigenvalues and eigenvector is made by EIGEN (http://eigen.tuxfamily.org)
  int counterMaxNeg;
  int realSize = Vec_Vertices.size();
  MinVP = 1;
  MatrixXd mat_X(realSize, realSize);
  VectorXd vecXd(realSize);
  VectorXd egVecBig(instance.DIM);
  vector<double> bigvec(instance.DIM, 0.0);

  double eigVal = -1;
  int counterNeg = 0;

  SelfAdjointEigenSolver<MatrixXd> es; // just for symmetric matrix (comp. it is 10 times faster than general method)
  double cstTranf = 1.0 / (K - 1);

  //Tranformation x to X
  for (int i = 0; i < realSize; i++)
    for (int j = 0; j < realSize; j++)
    {
      if (i != j)
        mat_X(i, j) = ValueXij(Vec_Vertices[i] + 1, Vec_Vertices[j] + 1, instance) * ((double)K / (K - 1)) - cstTranf; // before it was mat_X(i,j) = ValueXij(vertices_RnkByIdex[i]+1,vertices_RnkByIdex[j]+1,instance)
      else                                                                                                             // Diagonal values
        mat_X(i, j) = 1.0;
    }
  //Compute the eigenvalue and eigenvectr
  es.compute(mat_X);

  counterMaxNeg = realSize * frac;
  //    counterMaxNeg = 1;

  /*// Iteration of all eigenvalues /*/
  for (int i = 0; i < realSize && i <= counterMaxNeg; i++)
  {
    eigVal = es.eigenvalues()[i];

    if (eigVal + 0.001 < 0.0 && PopVecProp.size() <= NBMAXINEQ)
    { // eigenvalue is negatif, so there is a violation of the Semidefiniteness
      counterNeg++;

      if (eigVal < MinVP)
        MinVP = eigVal;

      //	cout << "eigVal = " <<eigVal<< endl;
      vecXd = es.eigenvectors().col(i);

      for (int j = 0; j < instance.DIM; j++)
        egVecBig[j] = 0.0;

      //find j in vertices and changing their values
      for (int z = 0; z < realSize; z++)
        egVecBig[Vec_Vertices[z]] = vecXd[z];

      PopVecProp.SetIneq(egVecBig, instance.DIM, eigVal, true); // add new eigenvector in populations
                                                                //		Design_SDPIneq2(instance, bigvec);
                                                                //		cin.get();
    }
    else
    {
      return; //end of function // The eigenvalues  are ranked by non-decrese value
    }

  } //end for

  if ((SDP_SEP == 2) && (counterNeg == 0))
    PopVecProp.clearAll();

} //end of function

void Gershgorin_Circle(T_Instance &instance, MKC_ConstraintLPtoSDPPopulate &PopVecProp)
{
  int SizeGersh = 0;

  switch (PreVP)
  {
  case 0:
    SizeGersh = instance.DIM * 0.4;
  case 15:
    SizeGersh = 5;
    break;
  case 16:
    SizeGersh = 10;
    break;
  default:
    SizeGersh = PreVP;
  }

  //cout << "Chegou Gershgorin_Circle in beging PopVecProp.size() = "<<PopVecProp.size() <<endl;

  int nbOfVertices = SizeGersh; //instance.DIM*0.4;
  if (nbOfVertices > instance.DIM)
    nbOfVertices = instance.DIM;

  //int nbOfVertices = instance.DIM;
  vector<int> vertices_RnkByIdex;
  set<int> aux_set;
  vector<int> aux_vec;
  set<MKC_InstanceRankVertices> ranked;
  std::set<MKC_InstanceRankVertices>::iterator it2;
  double sum;
  int realSize, counter, flag, random;
  int MAX_ITEGER_ITE = 2;
  double eigVal;
  double cstTranf = 1.0 / (K - 1);

  vector<double> egVecBig(instance.DIM, 0.0);

  ranked.clear();

  //rank of columns by their lp value (Calcule radius)
  for (int i = 0; i < instance.DIM; i++)
  {
    sum = 0.0;
    for (int j = 0; j < instance.DIM; j++)
      if (i != j)
        sum += ValueXij(i + 1, j + 1, instance);

    MKC_InstanceRankVertices rkVer(i, sum);
    ranked.insert(rkVer);
  }

  // Start iterations

  for (int ite = 0; ite < MAX_ITEGER_ITE; ite++)
  {
    //Alloc memory
    vertices_RnkByIdex.clear();
    aux_set.clear();
    aux_vec.clear();
    aux_vec.resize(nbOfVertices, -1);

    it2 = ranked.begin();
    for (int i = 0; i < nbOfVertices && it2 != ranked.end(); ++it2)
    {
      flag = 0;
      //Random
      random = (rand() % 10);
      if (random < 6)
        flag = 1; // we give prob of 70%

      if (ite == 0)
        flag = 1; //In first iteration we take all the best

      if (flag == 1)
      { //we do not take a element if it give us a triangle violation
        aux_set.insert((*it2).vertex);
        aux_vec[i] = (*it2).vertex;
        i++;
      }
    }

    vertices_RnkByIdex.resize(aux_set.size());
    std::set<int>::iterator it = aux_set.begin();
    for (int i = 0; i < nbOfVertices && it != aux_set.end(); ++it)
    {
      vertices_RnkByIdex[i] = *it;
      i++;
    }

    //send it for finding eigenvalues
    Eigenvaleu_ForSomeVertices(instance, PopVecProp, vertices_RnkByIdex, 0.2);

  } // end MAX_ITEGER_ITE neg

} //end   Gershgorin_Circle function

void FindNegatifEigenvaleu_antigoEing(T_Instance &instance, MKC_ConstraintLPtoSDPPopulate &PopVecProp)
{
  //calcule eigenvalues and eigenvector is made by EIGEN (http://eigen.tuxfamily.org)

  MinVP = 1;
  MatrixXd mat_X(instance.DIM, instance.DIM);
  VectorXcd vecXc(instance.DIM);
  vector<double> egVec(instance.DIM, 0.0);

  EigenSolver<MatrixXd> es; //remember eigenvalues() is complex number in Eigen

  double cstTranf = 1.0 / (K - 1);
  //cout << "cstTranf =" << endl << cstTranf << endl;

  //Tranformation x to X
  for (int i = 0; i < instance.DIM; i++)
    for (int j = 0; j < instance.DIM; j++)
    {
      if (i != j)
        mat_X(i, j) = ValueXij(i + 1, j + 1, instance) * ((double)K / (K - 1)) - cstTranf;
      else // Diagonal values
        mat_X(i, j) = 1.0;
    }

  //cout << "Matrix X: "<<endl << mat_X << std::endl;
  //Compute the eigenvalue and eigenvectr
  es.compute(mat_X);

  //cout << "The eigenvalues of A are:" << endl << es.eigenvalues() << endl;
  //cout << "The eigenvectors of A are:" << endl << es.eigenvectors()<< endl;
  double eigVal;
  int counterNeg = 0;

  /*// Iteration of all eigenvalues /*/
  for (int i = 0; i < instance.DIM; i++)
  {
    eigVal = real(es.eigenvalues()[i]);

    if ((eigVal + /*epslon*100*/ 0.001 < 0) || (SDP_SEP == 2))
    { // eigenvalue is negatif, so there is a violation of the Semidefiniteness
      //cout << "The eigenvalues of A are:" << endl << real (es.eigenvalues()[i])<< endl;
      //cout << "The eigenvectors of A are:" << endl << es.eigenvectors().col(i)<< endl;
      //cin.get();

      counterNeg++;

      if (eigVal < MinVP)
        MinVP = eigVal;

      vecXc = es.eigenvectors().col(i);

      for (int j = 0; j < instance.DIM; j++)
        egVec[j] = real(vecXc[j]); //  real(es.eigenvectors().col(i)[j]); //taking the real part of the eigenvector

      if (eigVal >= 0)
        PopVecProp.SetIneq(egVec, -eigVal, false); // add new eigenvector in population
      else
        PopVecProp.SetIneq(egVec, eigVal, true); // add new eigenvector in population
    }
  }

  if ((SDP_SEP == 2) && (counterNeg == 0))
    PopVecProp.clearAll();

  //** Junk+
  if (SDP_SEP == 2)
  {
    PrintEigPos.resize(5, -1.0);
    double max = 0.0, vall = 0.0;
    int conteur = 0;
    for (int i = PopVecProp.size() - 1; i >= 0; i--)
    {
      if (!PopVecProp.Getreal(i))
      {
        vall = -PopVecProp.Get_eigVal(i);
        if (conteur < PrintEigPos.size() - 1)
        {
          PrintEigPos[conteur] = vall;
          conteur++;
        }

        if (vall > max)
          max = vall;
      }
    } //end forbidem

    PrintEigPos[PrintEigPos.size() - 1] = max;
  }
}

//New way of calculate the eigenvalue for symmetric matrix
void FindNegatifEigenvaleu(T_Instance &instance, MKC_ConstraintLPtoSDPPopulate &PopVecProp)
{
  //cout << "Begin Eig find"; cin.get();
  //calcule eigenvalues and eigenvector is made by EIGEN (http://eigen.tuxfamily.org)

  MinVP = 1;
  MatrixXd mat_X(instance.DIM, instance.DIM);
  VectorXd vecXd(instance.DIM);
  vector<double> egVec(instance.DIM, 0.0);
  vector<double> eigvect(instance.edge_nb);
  double eigVal = -1;
  int counterNeg = 0;
  SelfAdjointEigenSolver<MatrixXd> es; // just for symmetric matrix (comp. it is 10 times faster than general method)
  double cstTranf = 1.0 / (K - 1);

  //Tranformation x to X
  for (int i = 0; i < instance.DIM; i++)
    for (int j = 0; j < instance.DIM; j++)
    {
      if (i != j)
        mat_X(i, j) = ValueXij(i + 1, j + 1, instance) * ((double)K / (K - 1)) - cstTranf;
      else // Diagonal values
        mat_X(i, j) = 1.0;
    }

  //Compute the eigenvalue and eigenvectr
  es.compute(mat_X);

  //Find max number of neg
  int counterMaxNeg = 0;

  if (TOTAL_SDPCUT >= instance.DIM)
    COMPLET_EIG = false;

  if (!COMPLET_EIG)
  {
    for (int i = 0; i < instance.DIM; i++)
    {
      eigVal = es.eigenvalues()[i];
      if (eigVal + 0.001 < 0.0)
        counterMaxNeg++;
      else
        break;
    } //end FOR i
    counterMaxNeg /= QT_DIV_EIG;
  }
  else
  {
    counterMaxNeg = instance.DIM;
  }

  /*// Iteration of all eigenvalues /*/
  for (int i = 0; i < instance.DIM && i <= counterMaxNeg; i++)
  {
    eigVal = es.eigenvalues()[i];

    if (eigVal + 0.001 < 0.0 && PopVecProp.size() <= NBMAXINEQ)
    { // eigenvalue is negatif, so there is a violation of the Semidefiniteness

      counterNeg++;

      if (eigVal < MinVP)
        MinVP = eigVal;

      PopVecProp.SetIneq(es.eigenvectors().col(i), instance.DIM, eigVal, true); // add new eigenvector in populations
    }
    else
    {
      return; //end of function // The eigenvalues  are ranked by non-decrese value
    }

  } //end for

  if ((SDP_SEP == 2) && (counterNeg == 0))
    PopVecProp.clearAll();
}

/*
//New way of calculate the eigenvalue for symmetric matrix
void FindNegatifEigenvaleu_RankedVertices(T_Instance &instance,MKC_ConstraintLPtoSDPPopulate &PopVecProp)
{
  int sizeVertex = instance.DIM/2;
  //cout << "Begin Eig find"; cin.get();
  //calcule eigenvalues and eigenvector is made by EIGEN (http://eigen.tuxfamily.org)

  set<MKC_InstanceRankVertices>::iterator it;
  vector<int> ranked_v;

  	//First phase : Rank the vertices


  ranked_v.resize(sizeVertex );  //rank the vertices
  //Rank_by_degree(ranked,instance ); // (WARNING) HERE vertices start in 1
  Rank_by_incidentWeight(ranked,instance); // (WARNING) HERE, in ranked, vertices start in 1

  counter = 0;
  for (it=ranked.begin(); it!=ranked.end() && counter < qtNb; ++it){
       ranked_v[counter] =  (*it).vertex - 1;
       counter ++;
  }

  MatrixXd mat_X(sizeVertex ,sizeVertex );
  VectorXd vecXd (sizeVertex );
  VectorXd vecXdTotal (instance.DIM); //with all the vectors
  vector<double> egVec(sizeVertex,0.0);
  double eigVal = -1 ;
  int counterNeg = 0;
  SelfAdjointEigenSolver<MatrixXd> es; // just for symmetric matrix (comp. it is 10 times faster than general method)
  double cstTranf = 1.0/(K -1);


  //Tranformation x to X
  for (int i=0; i<sizeVertex; i++)
    for (int j=0; j<sizeVertex; j++){
	I = ranked_v[i];
	J = ranked_v[j];
      if (i != j)
	mat_X(i,j) = ValueXij(I,J,instance)*((double)K/(K-1) ) -  cstTranf;
      else // Diagonal values
	mat_X(i,j) = 1.0;
    }

    //Compute the eigenvalue and eigenvectr
    es.compute(mat_X);


//     cout << "counterMaxNeg = " << counterMaxNeg; // cin.get();

    // Iteration of all eigenvalues
    for (int i=0; i<sizeVertex ; i++){
     eigVal = es.eigenvalues()[i];

      if (eigVal + 0.001< 0.0  && PopVecProp.size() <= NBMAXINEQ){ // eigenvalue is negatif, so there is a violation of the Semidefiniteness

	counterNeg++;

	//cout << "eigVal = " <<eigVal<< endl;
 	vecXd  = es.eigenvectors().col(i);

	//set all to zero
	for (int i=0; i<instance.DIM; i++)
		vecXdTotal[i] = 0.0;

	//fill (all) with eigenvector
	for (int i=0; i<sizeVertex; i++)
		vecXdTotal[ranked_v[i]-1] = vecXd[i];


	PopVecProp.SetIneq(vecXdTotal, instance.DIM,  eigVal, true); // add new eigenvector in population

      }else{
		return; //end of function // The eigenvalues  are ranked by non-decrese value
	}


    }//end for


  if ((SDP_SEP == 2) &&(counterNeg == 0 ))
       PopVecProp.clearAll();

}

*/

//New way of calculate the eigenvalue for symmetric matrix
void FindNegatifEigenvaleu_Constraint_WithPositiveEig(T_Instance &instance, MKC_ConstraintLPtoSDPPopulate &PopVecProp)
{
  //cout << "Begin Eig find"; cin.get();
  //calcule eigenvalues and eigenvector is made by EIGEN (http://eigen.tuxfamily.org)

  MinVP = 1;
  MatrixXd mat_X(instance.DIM, instance.DIM);
  VectorXd vecXd(instance.DIM);
  vector<double> egVec(instance.DIM, 0.0);
  double eigVal = -1;
  int counterNeg = 0;
  SelfAdjointEigenSolver<MatrixXd> es; // just for symmetric matrix (comp. it is 10 times faster than general method)
  double cstTranf = 1.0 / (K - 1);

  //Tranformation x to X
  for (int i = 0; i < instance.DIM; i++)
    for (int j = 0; j < instance.DIM; j++)
    {
      if (i != j)
        mat_X(i, j) = ValueXij(i + 1, j + 1, instance) * ((double)K / (K - 1)) - cstTranf;
      else // Diagonal values
        mat_X(i, j) = 1.0;
    }

  //Compute the eigenvalue and eigenvectr
  es.compute(mat_X);

  /*// Iteration of all eigenvalues /*/
  for (int i = 0; i < instance.DIM; i++)
  {
    eigVal = es.eigenvalues()[i];

    { // eigenvalue is negatif, so there is a violation of the Semidefiniteness (es.eigenvectors().col(i) = eingenvector)
      if (eigVal >= 0)
        PopVecProp.SetIneq(es.eigenvectors().col(i), instance.DIM, -eigVal, false); // add new positive eigenvector in population
      else
        PopVecProp.SetIneq(es.eigenvectors().col(i), instance.DIM, eigVal, true); // add new eigenvector in population
    }
  } //end for
}

void MakeSparseEig(T_Instance &instance, vector<double> &eigvect, const double &eigval, const int &sizeSetZero)
{

  double viol, diff, minvio, inicialVio, ActualVio;
  int aux_edge, Vert, I, J,
      counter = 0;
  double rightCst = 0,
         left = 0, valvp;
  double FirstCst = (1.0 / (K)), SecondCst = (((double)(K - 1.0)) / (2.0 * K));
  //inicialVio = ActualVio= minvio = eigval/SecondCst; //CalculateViolation_SDP (instance, eigvect); //calcule violation of SDP-based constraint
  //cout << "Initial violation = "<<ActualVio << endl;
  //max_elim = instance.DIM/4;

  set<MKC_InstanceRankVertices>::iterator it;
  set<MKC_InstanceRankVertices> ranked;

  for (int i = 0; i < instance.DIM; i++)
  {
    left = 0.0;
    rightCst = 0.0;
    for (int j = 0; j < instance.DIM; j++)
    {
      if (i != j)
      {
        I = i + 1;
        J = j + 1;
        //putting in xpr
        aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];

        valvp = eigvect[i] * eigvect[j];

        left += instance.varS[aux_edge] * valvp;
        rightCst += valvp * FirstCst; //it is minus
      }
      else
        rightCst -= eigvect[i] * eigvect[j] * SecondCst; //we fixed x_{i,i} = 1
    }                                                    //end of j
    diff = rightCst - left;
    MKC_InstanceRankVertices rkVer(i, -diff);
    ranked.insert(rkVer);
  } //end of i

  //for (it=ranked.begin(); it!=ranked.end(); ++it)   cout <<  counter <<" : " <<  (*it).weight << endl;

  //cin.get();
  //Eliminate all of them
  it = ranked.begin();
  for (int i = 0; i < sizeSetZero && it != ranked.end(); i++)
  {
    eigvect[(*it).vertex] = 0.0;
    ++it;
  }

FIM:
  //cout << "Number of vertex cleaned = " << counter;
  //cout << "Final  the violation is = " << CalculateViolation_SDP (instance, eigvect) << endl; cin.get();
  return;
}
double CalculateViolation_SDP(T_Instance &instance, std::vector<double> &vecProp)
{

  double rightCst = 0, left = 0, valvp;
  int I, J, counter = 0, aux_edge;
  double FirstCst = (1.0 / (K)), SecondCst = (((double)(K - 1.0)) / (2.0 * K));

  for (int i = 0; i < instance.DIM; i++)
    for (int j = i; j < instance.DIM && vecProp[i] != 0.0; j++)
    {
      if (i != j)
      {
        I = i + 1;
        J = j + 1;
        //putting in xpr
        aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];

        valvp = vecProp[i] * vecProp[j];

        if (valvp != 0.0)
        {
          left += instance.varS[aux_edge] * valvp;
          rightCst += valvp * FirstCst; //it is minus
        }
      }
      else
        rightCst -= vecProp[i] * vecProp[j] * SecondCst; //we fixed x_{i,i} = 1
    }                                                    //end j

  //return (rightCst - left)*(2.0*K/(K-1));
  return (rightCst - left);
}

void Design_SDPIneq_SubGraph(T_Instance &instance, Eigen::VectorXcd &vecProp, const int begin, const int endd)
{
  // Disign in CPlex the inequality:
  //	sum(u,v_i) - sum(v_{i-1},v_i) - u_{1,2} <= cstW
  // vec_vertices {u | v_1, v_2, ... , V_n} where u is the hub of the wheel
  int aux_edge, counter, I, J;
  double rightCst = 0.0, valvp;
  int size_T = endd - begin;
  //cout << "size_T = " << size_T << ", begin = " << begin << ", endd=" << endd<<endl;
  //cin.get();
  TOTAL_SDPCUT++;

  //creating a constraint
  T_constraint Const;

  Const.Origem = 6;      //sdp
  Const.bkc = MSK_BK_LO; // Bound key (>=)
  //Const.blc 	= -cst;// Numerical value of Lowwer bound
  Const.buc = +MSK_INFINITY; // Numerical value of Upper bound

  //resize by number of edge in a complete graph = n*(n-1)/2
  Const.vars.resize(((size_T) * (size_T - 1)) / 2); // fix
  Const.aval.resize(((size_T) * (size_T - 1)) / 2); // fix

  counter = 0;
  //i=0; j=1, h=2
  //Edge i and J

  // making sum(u,v_i)
  for (int i = 0; i < size_T; i++)
    for (int j = i; j < size_T; j++)
    {
      if (i != j)
      {
        I = i + 1;
        J = j + 1;
        aux_edge = instance.indice_cij[(I + begin) + (J + begin) * (instance.DIM + 1)];
        valvp = 2 * real(vecProp[i]) * real(vecProp[j]);
        if (valvp != 0.0)
        {
          Const.vars[counter] = (int)aux_edge;
          Const.aval[counter] = valvp * ((double)K / (K - 1)); //
          rightCst += valvp * (1.0 / (K - 1));                 //it is minus
          counter++;
        }
      }
      else
        rightCst -= real(vecProp[i]) * real(vecProp[j]) * (1.0); //we fixed x_{i,i} = 1 three diagonal elements in the diagonal so we have to multiply for 3
    }

  if (counter > 0)
  {
    //resize by number of edge in a complete graph = n*(n-1)/2
    Const.vars.resize(counter); // fix
    Const.aval.resize(counter); // fix

    Const.blc = rightCst; // Numerical value of Lowwer bound

    //print_Inequality_Tconstraint(Const);

    instance.CONST.push_back(Const);
  }
}

void Design_SDPIneq_Triangle(T_Instance &instance, Eigen::VectorXcd &vecProp, const std::vector<int> &vertex)
{
  // Disign in CPlex the inequality:
  //	sum(u,v_i) - sum(v_{i-1},v_i) - u_{1,2} <= cstW
  // vec_vertices {u | v_1, v_2, ... , V_n} where u is the hub of the wheel
  int aux_edge, counter, I, J;
  double rightCst = 0.0, valvp;
  int size_T = 3;

  //TOTAL_SDPCUT++;

  //creating a constraint
  T_constraint Const;

  Const.Origem = 6;      //whee and bicycle wheel
  Const.bkc = MSK_BK_LO; // Bound key (>=)
  //Const.blc 	= -cst;// Numerical value of Lowwer bound
  Const.buc = +MSK_INFINITY; // Numerical value of Upper bound

  //resize by number of edge in a complete graph = n*(n-1)/2
  Const.vars.resize(((size_T) * (size_T - 1)) / 2); // fix
  Const.aval.resize(((size_T) * (size_T - 1)) / 2); // fix

  counter = 0;
  //i=0; j=1, h=2
  //Edge i and J

  // making sum(u,v_i)
  for (int i = 0; i < size_T; i++)
    for (int j = i; j < size_T; j++)
    {
      if (i != j)
      {
        aux_edge = instance.indice_cij[vertex[i] + vertex[j] * (instance.DIM + 1)];
        valvp = 2 * real(vecProp[i]) * real(vecProp[j]);
        if (valvp != 0.0)
        {
          Const.vars[counter] = (int)aux_edge;
          Const.aval[counter] = valvp * ((double)K / (K - 1)); //
          rightCst += valvp * (1.0 / (K - 1));                 //it is minus
          counter++;
        }
      }
      else
        rightCst -= real(vecProp[i]) * real(vecProp[j]) * (1.0); //we fixed x_{i,i} = 1 three diagonal elements in the diagonal so we have to multiply for 3
    }

  if (counter > 0)
  {
    //resize by number of edge in a complete graph = n*(n-1)/2
    Const.vars.resize(counter); // fix
    Const.aval.resize(counter); // fix

    Const.blc = rightCst; // Numerical value of Lowwer bound

    //print_Inequality_Tconstraint(Const);

    instance.CONST.push_back(Const);
  }
}

void FindNegatifEigenvaleuSubGraph(T_Instance &instance, MKC_ConstraintLPtoSDPPopulate &PopVecProp)
{
  //calcule eigenvalues and eigenvector is made by EIGEN (http://eigen.tuxfamily.org)
  MinVP = 1;
  vector<double> egVec(instance.DIM, 0.0);
  vector<int> vertex(3, 0);

  EigenSolver<MatrixXd> es; //remember eigenvalues() is complex number in Eigen

  double cstTranf = 1.0 / (K - 1);
  //cout << "cstTranf =" << endl << cstTranf << endl;

  //Tranformation x to X
  int step = instance.DIM * 0.51;

  MatrixXd mat_X_T(step, step);
  VectorXcd vecXc_T(step);

  for (int j = 0; j + step - 1 < instance.DIM; j += step)
  { // it start by 1

    //cout << "Entrou j = " << j << endl;
    //cin.get();

    mat_X_T.setIdentity();
    //Tranformation x to X
    for (int i = 0; i < step; i++)
      for (int h = 0; h < step; h++)
      {
        if (i != h)
          mat_X_T(i, h) = ValueXij(i + 1 + (j), h + 1 + (j), instance) * ((double)K / (K - 1)) - cstTranf;
      }

    //Calculating the eigenvalues
    es.compute(mat_X_T);

    //analyzing the eigenvalues
    double eigVal;
    int counterNeg = 0;
    for (int zz = 0; zz < step; zz++)
    {
      eigVal = real(es.eigenvalues()[zz]);

      if ((eigVal + /*epslon*100*/ 0.001 < 0))
      { // eigenvalue is negatif, so there is a violation of the Semidefiniteness
        //cout << "Entrou aqui safadim" <<endl;
        //cin.get();
        vecXc_T = es.eigenvectors().col(zz);                     //eigenvector
        Design_SDPIneq_SubGraph(instance, vecXc_T, j, j + step); //desing
      }

    } //end zz

  } //end J
  //cout << "acabou FindNegatifEigenvaleuSubGraph_triangle safadao";
  //		cin.get();
}

void FindNegatifEigenvaleuSubGraph_triangle(T_Instance &instance, MKC_ConstraintLPtoSDPPopulate &PopVecProp)
{
  //calcule eigenvalues and eigenvector is made by EIGEN (http://eigen.tuxfamily.org)
  MinVP = 1;
  MatrixXd mat_X(instance.DIM, instance.DIM);
  MatrixXd mat_X_T(3, 3);
  VectorXcd vecXc_T(3);
  VectorXcd vecXc(instance.DIM);
  vector<double> egVec(instance.DIM, 0.0);
  vector<int> vertex(3, 0);

  EigenSolver<MatrixXd> es; //remember eigenvalues() is complex number in Eigen

  double cstTranf = 1.0 / (K - 1);
  //cout << "cstTranf =" << endl << cstTranf << endl;

  //Tranformation x to X

  for (int j = 0; j < instance.DIM - 2; j++)
  { // it start by 1
    for (int i = j + 1; i < instance.DIM - 1; i++)
    { //j+1 pois nao qro xii
      for (int h = i + 1; h < instance.DIM; h++)
      { //i+1 pois nao qro xhh
        //setting new valeus in max_x
        mat_X_T.setIdentity();

        //i=0, j=1, k =2
        //cout << "Identity = " <<   mat_X_T;

        mat_X_T(0, 1) = ValueXij(i + 1, j + 1, instance) * ((double)K / (K - 1)) - cstTranf;
        mat_X_T(1, 0) = ValueXij(i + 1, j + 1, instance) * ((double)K / (K - 1)) - cstTranf;
        mat_X_T(2, 1) = ValueXij(h + 1, j + 1, instance) * ((double)K / (K - 1)) - cstTranf;
        mat_X_T(1, 2) = ValueXij(h + 1, j + 1, instance) * ((double)K / (K - 1)) - cstTranf;
        mat_X_T(0, 2) = ValueXij(i + 1, h + 1, instance) * ((double)K / (K - 1)) - cstTranf;
        mat_X_T(2, 0) = ValueXij(i + 1, h + 1, instance) * ((double)K / (K - 1)) - cstTranf;
        //cout << endl << "After = " <<  endl <<  mat_X_T;
        //cin.get();

        es.compute(mat_X_T);

        //analyzing the eigenvalues
        double eigVal;
        int counterNeg = 0;
        for (int zz = 0; zz < 3; zz++)
        {
          eigVal = real(es.eigenvalues()[zz]);

          if ((eigVal + /*epslon*100*/ 0.001 < 0))
          { // eigenvalue is negatif, so there is a violation of the Semidefiniteness
            //cout << "The eigenvalues of A are:" << endl << real (es.eigenvalues()[i])<< endl;
            //cout << "The eigenvectors of A are:" << endl << es.eigenvectors().col(i)<< endl;
            //cin.get();

            vertex[0] = i + 1;
            vertex[1] = j + 1;
            vertex[2] = h + 1;

            vecXc_T = es.eigenvectors().col(zz);

            //for (int jj=0; jj<instance.DIM; jj++)
            //egVec[jj] = real (vecXc[jj]);//  real(es.eigenvectors().col(i)[j]); //taking the real part of the eigenvector

            //mat_vio = v*v.transpose();
            //cout << "violation: "<< v.transpose()*mat_X*v << std::endl;
            //cout << "mat_vio: "<<  endl << mat_vio<< std::endl;

            //printvector(egVec);
            //cin.get();

            //include as a constraint
            Design_SDPIneq_Triangle(instance, vecXc_T, vertex);

            //PopVecProp.SetIneq(egVec, eigVal, true); // add new eigenvector in population
          }
        }

      } // end h
    }   //end i
  }     //end J

  //cout << "acabou FindNegatifEigenvaleuSubGraph_triangle safadao";
  //		cin.get();
}

inline void SetPretreatmen(T_Instance &instance)
{

  if ((PreVP != 0) && (PreVP < 15))
  { //Pre-treatment
    //We proved that the next two pre-treatmets (6 and 7) are not good (we can delete them)
    if ((PreVP == 6) || (PreVP == 8))
      set_pre_inequalities_VP2(instance); // HouseHOlder

    if ((PreVP == 7) || (PreVP == 8))
      set_pre_inequalities_VP3(instance); //Function objective

    // ****** start the pre-treatments with sub-graphs
    if (((PreVP >= 9) && (PreVP <= 11)) || (PreVP == 13) || (PreVP == 14))
      PreTrea_Div2Conq(instance);
    if (PreVP == 12)
      PreTrea_Div2Conq_rank_exploreAll(instance);

    if (PreVP == 14)
      PreTrea_Div2Conq_Eigenvalue(instance); // verify just eigenvalue violations in subgraphs os size = 10 vertices is used with other
  }

  if (PreVP == 99)
    PreTrea_Div2Conq_forUpperbound(instance);
}

//*******
//******		Here we build the inequalities with eigenvectors that are orthonormals they have all cases =1 exception of the case i
//**** Householder
void set_pre_inequalities_VP2(T_Instance &instance)
{
  MinVP = 1;
  MatrixXd mat_X(instance.DIM, instance.DIM);
  VectorXd vecXc(instance.DIM);
  MatrixXd Id(instance.DIM, instance.DIM);
  vector<double> egVec(instance.DIM, 0.0);
  double SqNorm;

  MKC_ConstraintLPtoSDPPopulate PopVec;
  PopVec.clearAll();

  for (int zz = 0; zz < 1; zz++)
  {

    SqNorm = 0.0;

    for (int i = 0; i < instance.DIM; i++)
    {
      vecXc[i] = sampleNormal();
      if ((vecXc[i] > 1) || (vecXc[i] < -1))
        vecXc[i] = 0.0;
      SqNorm += pow(vecXc[i], 2.0);
    }

    //Euclidian norm
    SqNorm = sqrt(SqNorm);

    for (int i = 0; i < instance.DIM; i++)
      vecXc[i] /= SqNorm;

    //cout << vecXc; THE HOUSEHOLDER
    Id.setIdentity();
    mat_X = Id - 2 * vecXc * vecXc.transpose();

    //** Making the i-1 new vectors from a comination of eigenvectors of matrix
    for (int j = 0; j < instance.DIM; j++)
    { //elements of new vector
      vecXc = mat_X.col(j);
      for (int i = 0; i < instance.DIM; i++)
        egVec[i] = vecXc[i]; //

      Design_SDPIneq2(instance, egVec);
    } //end FOR j
  }   //end zz;
  PopVec.clearAll();
}

//** Normal distribution from Uniform distribution (rand())
inline double sampleNormal()
{
  double u = ((double)rand() / (RAND_MAX)) * 2 - 1;
  double v = ((double)rand() / (RAND_MAX)) * 2 - 1;
  double r = u * u + v * v;
  if (r == 0 || r > 1)
    return sampleNormal();
  double c = sqrt(-2 * log(r) / r);
  return u * c;
}

//*** Vector build based in the edge weight in the cost matrix
void set_pre_inequalities_VP3(T_Instance &instance)
{
  int aux_edge, I, J;
  double val;
  double SqNorm = sqrt(double(instance.DIM - 1)); // it has n-1 elements different of zero

  vector<double> egVec(instance.DIM, 0.0);
  //cout << PopVecProp.size() << endl;
  //cout << "(int)MatrixVec.size() = " << (int)MatrixVec.size()<< endl;
  //** Making the i-1 new vectors from a comination of eigenvectors of matrix
  for (int i = 0; i < instance.DIM; i++)
  { // we must start from the second element in the Matrix
    for (int j = i + 1; j < instance.DIM; j++)
    { //elements of new vector

      if ((egVec[i] == 0) || (egVec[j] == 0))
      {
        I = i + 1;
        J = j + 1;
        //putting in xpr
        aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
        val = instance.cij.barc_v[aux_edge];

        if (val > 0)
        {

          if ((egVec[i] == 0) && (egVec[j] == 0))
            egVec[i] = 1.0 / SqNorm;
          else if (egVec[i] == 0)
            egVec[i] = egVec[j];

          egVec[j] = egVec[i]; //Je veux qu'il soit possitive
        }
        else if (val < 0)
        {

          if ((egVec[i] == 0) && (egVec[j] == 0))
            egVec[i] = 1.0 / SqNorm;
          else if (egVec[i] == 0)
            egVec[i] = -egVec[j];

          egVec[j] = -egVec[i]; //Je veux qu'il soit negatif
        }
      }
    } //end FOR j
    Design_SDPIneq2(instance, egVec);

    for (int j = 0; j < instance.DIM; j++)
      egVec[j] = 0.0;
  } //end FOR i
}

void set_pre_inequalities_VP(T_Instance &instance)
{
  //MKC_ConstraintLPtoSDPPopulate		PopVec;
  double abs = sqrt(double(instance.DIM - 1)); // it has n-1 elements different of zero
  vector<double> egVec(instance.DIM, abs);
  //PopVec.clearAll();

  //** Making the i-1 new vectors from a comination of eigenvectors of matrix
  for (int j = 0; j < instance.DIM; j++)
  { //elements of new vector
    egVec[j] = 0.0;

    //PopVec.SetIneq(egVec, -1, true); // add new eigenvector in population
    Design_SDPIneq2(instance, egVec);
    egVec[j] = abs;
  }

  //cout << " PopVec.size() before = " << PopVec.size()<<endl;
  //Method_to_ValidEigenvalues_Miguel(instance,PopVec);
  //cout << " PopVec.size() after = " << PopVec.size();
  //cin.get();

  //inserting inequalties in the LP
  //for (int j=0; j< PopVec.size(); j++){
  // egVec = PopVec.GetVector(j);
  // Design_SDPIneq(instance, egVec);
  //}

  //PopVec.clearAll();
}

//** Combination of the eigenvectors that are negatives
void Method_to_ValidEigenvalues_Vilmar(T_Instance &instance, MKC_ConstraintLPtoSDPPopulate &PopVecProp)
{
  double val;
  double SqNorm = sqrt(double(instance.DIM - 1)); // it has n-1 elements different of zero

  vector<double> egVec(instance.DIM, 0.0);
  //cout << PopVecProp.size() << endl;
  //cout << "(int)MatrixVec.size() = " << (int)MatrixVec.size()<< endl;
  //** Making the i-1 new vectors from a comination of eigenvectors of matrix
  for (int i = 0; i < instance.DIM; i++)
  { // we must start from the second element in the Matrix
    for (int j = i + 1; j < instance.DIM; j++)
    { //elements of new vector

      if ((egVec[i] == 0) || (egVec[j] == 0))
      {
        val = ValueXij(i + 1, j + 1, instance);
        if (val < 0.1)
        {

          if ((egVec[i] == 0) && (egVec[j] == 0))
            egVec[i] = 1.0 / SqNorm;
          else if (egVec[i] == 0)
            egVec[i] = egVec[j];

          egVec[j] = egVec[i]; //Je veux qu'il soit possitive
        }
        else if (val >= 0.5)
        {

          if ((egVec[i] == 0) && (egVec[j] == 0))
            egVec[i] = 1.0 / SqNorm;
          else if (egVec[i] == 0)
            egVec[i] = -egVec[j];

          egVec[j] = -egVec[i]; //Je veux qu'il soit negatif
        }
      }
    }                                    //end FOR j
    PopVecProp.SetIneq(egVec, -1, true); // add new eigenvector negatif in population
    for (int j = 0; j < instance.DIM; j++)
      egVec[j] = 0;
  } //end FOR i
  //cout << PopVecProp.size() << endl;
  //printvector(egVec);
  //cin.get();
}

//** Combination of the eigenvectors that are negatives
void Method_to_ValidEigenvalues_Miguel(T_Instance &instance, MKC_ConstraintLPtoSDPPopulate &PopVecProp)
{
  double tol = 0.1; //Tolerance to be considered
  double abs, SqNorm, viol;

  vector<double> egVec(instance.DIM, 0.0);
  vector<double> vall(instance.DIM, 0.0);

  //cout << "PopVecProp.size() = " << PopVecProp.size();

  vector<T_eign_Matrix> MatrixVec;                   // structure created to store the eigenvectors and values
  PopVecProp.GetMatrixAllEigenvalue(MatrixVec, tol); // copying all the vectors that have eigenvalue < -tol in PopVecProp to MatrixVec

  //PopVecProp.printAll();

  //cout << "(int)MatrixVec.size() = " << (int)MatrixVec.size()<< endl;

  //** Making the i-1 new vectors from a comination of eigenvectors of matrix
  for (int i = 0; i < (int)MatrixVec.size(); i++)
  { // we must start from the second element in the Matrix
    abs = 0.0;
    viol += MatrixVec[i].eigVal;
    for (int j = 0; j < instance.DIM; j++)
    { //elements of new vector
      vall[j] += MatrixVec[i].eigVec[j] * MatrixVec[i].eigVal;
      abs += pow(vall[j], 2.0);
    }

    if (i > 0)
    {
      SqNorm = sqrt(abs);

      //creating the new vector
      if (SqNorm != 0)
      {
        for (int j = 0; j < instance.DIM; j++)
          egVec[j] = vall[j] / SqNorm;
        /*
	cout << "vall = ";
	printvector(vall);
	cout << "SqNorm = " << SqNorm <<endl;
	printvector(egVec);
	cin.get();
	*/
        PopVecProp.SetIneq(egVec, viol / i, true); // add new eigenvector negatif in population
      }                                            //end IF SQNORM
    }
  } //end FOR i

  //cout << "PopVecProp.size() depois = " << PopVecProp.size() << endl; cin.get();
}

void Heuristic_to_ValidEigenvalues(T_Instance &instance, MKC_ConstraintLPtoSDPPopulate &PopVecProp)
{
  //calcule eigenvalues and eigenvector is made by EIGEN (http://eigen.tuxfamily.org)
  int random1, random2;
  double D_random, viol;
  int NB_ITE = 20;

  MatrixXd mat_X(instance.DIM, instance.DIM);
  vector<double> egVec(instance.DIM, 0.0);
  vector<double> aux_vec(instance.DIM, 0.0);
  vector<double> aux_vec2(instance.DIM, 0.0);

  if (PopVecProp.size() > 3)
    for (int i = 0; i < NB_ITE; i++)
    {

      random1 = (rand() % PopVecProp.size());
      random2 = (rand() % PopVecProp.size());
      D_random = (0.15 + ((double)(rand() % 85) / 100.0));

      //cout << "random1 = " << random1 <<", random2 = "<< random2 <<", D_random = "<< D_random;
      //cin.get();

      if (random1 != random2)
      {
        aux_vec = PopVecProp.GetVector(random1);
        aux_vec2 = PopVecProp.GetVector(random2);
        for (int j = 0; j < instance.DIM; j++)
          egVec[j] = D_random * aux_vec[j] + (1 - D_random) * aux_vec2[j]; //taking the real part of the eigenvector
      }

      viol = D_random * PopVecProp.Get_eigVal(random1) + (1 - D_random) * PopVecProp.Get_eigVal(random2);

      PopVecProp.SetIneq(egVec, viol, true); // add new eigenvector in population
    }

  //cin.get();
}

//suppose that graph is complete
void Set_IneqFuncObj(T_Instance &instance, const double &rghtValue)
{
  // Disign in CPlex the inequality:
  //	sum(u,v_i) - sum(v_{i-1},v_i) - u_{1,2} <= cstW
  // vec_vertices {u | v_1, v_2, ... , V_n} where u is the hub of the wheel
  int aux_edge, counter, I, J;

  //TOTAL_SDPCUT++;

  double sumTotal = 0;
  for (int j = 0; j < instance.edge_nb; ++j)
    sumTotal += instance.cij.barc_v[j];

  //creating a constraint
  T_constraint Const;

  Const.Origem = 77;                //triangle
  Const.bkc = MSK_BK_UP;            // Bound key (<=)
  Const.blc = -MSK_INFINITY;        // Numerical value of Lowwer bound
  Const.buc = rghtValue - sumTotal; // Numerical value of Upper bound

  //resize by number of edge in a complete graph = n*(n-1)/2
  Const.vars.resize(instance.edge_nb); // fix
  Const.aval.resize(instance.edge_nb); // fix

  counter = 0;
  // making sum(u,v_i)
  for (int i = 0; i < instance.DIM; i++)
    for (int j = i + 1; j < instance.DIM; j++)
    {
      I = i + 1;
      J = j + 1;
      //putting in xpr
      aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
      if (aux_edge != -1)
      {
        Const.vars[counter] = (int)aux_edge;
        Const.aval[counter] = -1.0 * instance.cij.barc_v[aux_edge]; //
        counter++;
      }
    } //end j

  //print_Inequality_Tconstraint(Const);
  //cout << "Chegou aqui" << endl;
  //cin.get();
  instance.CONST.push_back(Const);
}

//suppose that graph is complete (Based in Nikiforov upperBOun)
void Set_IneqNuberOfEdgesCut(T_Instance &instance, const double &rghtValue, const int &qtEdges)
{
  // Disign in CPlex the inequality:
  //	sum(u,v_i) - sum(v_{i-1},v_i) - u_{1,2} <= cstW
  // vec_vertices {u | v_1, v_2, ... , V_n} where u is the hub of the wheel
  int aux_edge, counter, I, J;

  //creating a constraint
  T_constraint Const;

  Const.Origem = 77;         //triangle
  Const.bkc = MSK_BK_UP;     // Bound key (<=)
  Const.blc = -MSK_INFINITY; // Numerical value of Lowwer bound
  Const.buc = rghtValue;     // Numerical value of Upper bound

  //resize by number of edge in a complete graph = n*(n-1)/2
  Const.vars.resize(qtEdges); // fix
  Const.aval.resize(qtEdges); // fix

  counter = 0;
  // making sum(u,v_i)
  for (int i = 0; i < instance.DIM; i++)
    for (int j = i + 1; j < instance.DIM; j++)
    {
      I = i + 1;
      J = j + 1;
      //putting in xpr
      aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
      if (counter > qtEdges)
      {
        cout << "Error in Set_IneqNuberOfEdgesCut  number of counter is bigger than qtEdges ... it will give segmentation";
        exit(1);
      }
      if ((aux_edge != -1) && (instance.cij.barc_v[aux_edge] != 0.0))
      {
        Const.vars[counter] = (int)aux_edge;
        Const.aval[counter] = 1.0; //
        counter++;
      }
    } //end j

  //print_Inequality_Tconstraint(Const);
  //   cout << "Chegou aqui" << endl;
  //cin.get();
  instance.CONST.push_back(Const);
}

void Design_SDPIneq2(T_Instance &instance, const std::vector<double> &vecProp)
{
  // Disign in CPlex the inequality:
  //	sum(u,v_i) - sum(v_{i-1},v_i) - u_{1,2} <= cstW
  // vec_vertices {u | v_1, v_2, ... , V_n} where u is the hub of the wheel
  int aux_edge, counter, I, J;
  double rightCst = 0.0, valvp;
  double violation = 0.0, left_side = 0;
  double epslonEig = 0.01;

  TOTAL_SDPCUT++;

  //creating a constraint
  T_constraint Const;

  Const.Origem = 9;      //eigenvalue ineq
  Const.bkc = MSK_BK_LO; // Bound key (>=)
  //Const.blc 	= -cst;// Numerical value of Lowwer bound
  Const.buc = +MSK_INFINITY; // Numerical value of Upper bound

  //resize by number of edge in a complete graph = n*(n-1)/2
  Const.vars.resize(((instance.DIM) * (instance.DIM - 1)) / 2); // fix
  Const.aval.resize(((instance.DIM) * (instance.DIM - 1)) / 2); // fix

  counter = 0;
  // making sum(u,v_i)
  for (int i = 0; i < instance.DIM; i++)
    if ((vecProp[i] <= -epslonEig) || (vecProp[i] >= epslonEig))
      for (int j = i; j < instance.DIM && vecProp[i] != 0.0; j++)
      {
        if ((i != j) && ((vecProp[j] <= -epslonEig) || (vecProp[j] >= epslonEig)))
        {
          I = i + 1;
          J = j + 1;
          //putting in xpr
          aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];

          if (aux_edge != -1)
          { // (-1) means that edge does not exist in graph
            valvp = 2 * vecProp[i] * vecProp[j];

            if (valvp != 0.0)
            {
              Const.vars[counter] = (int)aux_edge;
              Const.aval[counter] = valvp * ((double)K / (K - 1)); //
              left_side += valvp * ((double)K / (K - 1)) * instance.varS[aux_edge];
              rightCst += valvp * (1.0 / (K - 1)); //it is minus
              counter++;
            }
          } //end IF aux_edge = ok
        }
        else if ((i == j) && ((vecProp[j] <= -epslonEig) || (vecProp[j] >= epslonEig)))
          rightCst -= vecProp[i] * vecProp[j] * (1.0); //we fixed x_{i,i} = 1

      } //end j

  //  violation = rightCst -left_side;
  //  cout << "The violation in design is "<< violation << endl;

  if (counter > 0)
  {
    //resize by number of edge in a complete graph = n*(n-1)/2
    Const.vars.resize(counter); // fix
    Const.aval.resize(counter); // fix

    Const.blc = rightCst; // Numerical value of Lowwer bound

    //cout << "Printing Eigen valeu :" << endl;
    //print_Inequality_Tconstraint(Const);
    //print_Inequality_TconstraintIndex(Const, instance);
    //cin.get();

    instance.CONST.push_back(Const);
  }
}

void Design_SDPIneq3(T_Instance &instance, const std::vector<double> &vecProp)
{
  // Disign in CPlex the inequality:
  //	sum(u,v_i) - sum(v_{i-1},v_i) - u_{1,2} <= cstW
  // vec_vertices {u | v_1, v_2, ... , V_n} where u is the hub of the wheel
  int aux_edge, counter, I, J;
  double rightCst = 0.0, valvp;

  TOTAL_SDPCUT++;

  //creating a constraint
  T_constraint Const;

  Const.Origem = 9;      //eigenvalue ineq
  Const.bkc = MSK_BK_LO; // Bound key (>=)
  //Const.blc 	= -cst;// Numerical value of Lowwer bound
  Const.buc = +MSK_INFINITY; // Numerical value of Upper bound

  //resize by number of edge in a complete graph = n*(n-1)/2
  Const.vars.resize(((instance.DIM) * (instance.DIM - 1)) / 2); // fix
  Const.aval.resize(((instance.DIM) * (instance.DIM - 1)) / 2); // fix

  counter = 0;
  // making sum(u,v_i)
  for (int i = 0; i < instance.DIM; i++)
    for (int j = i; j < instance.DIM; j++)
    {
      if (i != j)
      {
        I = i + 1;
        J = j + 1;
        //putting in xpr
        aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
        valvp = 2 * vecProp[i] * vecProp[j];

        if (valvp != 0.0)
        {
          Const.vars[counter] = (int)aux_edge;
          Const.aval[counter] = valvp; //
          //rightCst += valvp*(1.0/(K-1)); //it is minus
          counter++;
        }
      }
      else
        rightCst -= vecProp[i] * vecProp[j] * (1.0); //we fixed x_{i,i} = 1

    } //end j

  if (counter > 0)
  {
    //resize by number of edge in a complete graph = n*(n-1)/2
    Const.vars.resize(counter); // fix
    Const.aval.resize(counter); // fix

    Const.blc = rightCst; // Numerical value of Lowwer bound

    //print_Inequality_Tconstraint(Const);
    //cin.get();

    instance.CONST.push_back(Const);
  }
}

//Design and add in the CPLEX model
void Design_SDPIneq(T_Instance &instance, const std::vector<double> &vecProp)
{
  // Disign in CPlex the inequality:
  //	sum(u,v_i) - sum(v_{i-1},v_i) - u_{1,2} <= cstW
  // vec_vertices {u | v_1, v_2, ... , V_n} where u is the hub of the wheel
  int aux_edge, counter, I, J;
  double rightCst = 0.0;

  TOTAL_SDPCUT++;

  //creating a constraint
  T_constraint Const;

  Const.Origem = 6;      //whee and bicycle wheel
  Const.bkc = MSK_BK_LO; // Bound key (>=)
  //Const.blc 	= -cst;// Numerical value of Lowwer bound
  Const.buc = +MSK_INFINITY; // Numerical value of Upper bound

  //resize by number of edge in a complete graph = n*(n-1)/2
  Const.vars.resize(((instance.DIM) * (instance.DIM - 1)) / 2); // fix
  Const.aval.resize(((instance.DIM) * (instance.DIM - 1)) / 2); // fix

  counter = 0;
  // making sum(u,v_i)
  for (int i = 0; i < instance.DIM; i++)
    for (int j = i; j < instance.DIM; j++)
    {
      if (i != j)
      {
        I = i + 1;
        J = j + 1;
        //putting in xpr
        aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
        Const.vars[counter] = (int)aux_edge;
        Const.aval[counter] = 2 * vecProp[i] * vecProp[j] * ((double)K / (K - 1)); //
        rightCst -= 2 * vecProp[i] * vecProp[j] * (1.0 / (K - 1));                 //it is minus
        counter++;
      }
      else
        rightCst += vecProp[i] * vecProp[j] * (1.0); //we fixed x_{i,i} = 1

    } //end j

  Const.blc = -rightCst; // Numerical value of Lowwer bound

  //print_Inequality_Tconstraint(Const);

  instance.CONST.push_back(Const);
}

inline void print_Inequality_Tconstraint(const T_constraint &Const)
{
  cout << "print_Inequality_Tconstraint :" << endl;

  for (int i = 0; i < (int)Const.vars.size(); i++)
  {
    cout << "+ " << Const.aval[i] << "*" << Const.vars[i];
  }

  if (Const.bkc == MSK_BK_LO)
    cout << " >= " << Const.blc;
  else if (Const.bkc == MSK_BK_FX)
    cout << " = " << Const.blc;
  else
    cout << " <= " << Const.buc;
}

inline void print_Inequality_TconstraintIndex(const T_constraint &Const, const T_Instance &instance)
{
  cout << "print_Inequality_Tconstraint :" << endl;

  for (int i = 0; i < (int)Const.vars.size(); i++)
  {
    cout << "+ " << Const.aval[i] << "*x" << instance.ant_indice_cij[Const.vars[i]].idx_j << "," << instance.ant_indice_cij[Const.vars[i]].idx_i;
  }

  if (Const.bkc == MSK_BK_LO)
    cout << " >= " << Const.blc;
  else if (Const.bkc == MSK_BK_FX)
    cout << " = " << Const.blc;
  else
    cout << " <= " << Const.buc;
}

inline void select_vertices_fromMOstviolatedIneq(const T_Instance &instance, std::vector<int> &ranked_v, const int &max_rand, const int &qt_Ineq, const int &max_size,
                                                 MKC_ConstraintSelectionPopulate &PopBestIneq_origem, MKC_ConstraintTrianglePopulate &PopTriangle_origem, MKC_ConstraintCliquePopulate &PopClique_origem, MKC_ConstraintCliquePopulate &PopGenClique_origem,
                                                 MKC_ConstraintWheelPopulate &PopWheell_origem, MKC_ConstraintWheelPopulate &PopBiWheell_origem)
{
  int max = max_rand;
  int cont, Seq;
  std::vector<int> vec_aux;
  std::set<int> set_vec;
  std::set<int>::iterator it;

  if (max > PopBestIneq_origem.get_CurrentSize())
    max = PopBestIneq_origem.get_CurrentSize();

  set_vec.clear();

  if (max > 2)
  {
    for (int i = 0; (i < qt_Ineq) && ((int)set_vec.size() < max_size); i++)
    {
      int random2 = (int)(rand() % max);
      Seq = PopBestIneq_origem.get_Data(random2).orPos;
      switch (PopBestIneq_origem.get_Data(random2).orSep)
      {
      case 0: //Triangle
        vec_aux = PopTriangle_origem.get_Vertices(Seq);
        break;
      case 1: //Clique inequality
        vec_aux = PopClique_origem.get_vecVertx_Inequality(Seq);
        break;
      case 2: //General Clique
        vec_aux = PopGenClique_origem.get_vecVertx_Inequality(Seq);
        break;
      case 3: //Wheel Inequality
        vec_aux = PopWheell_origem.GetVectorWheel(Seq);
        break;
      case 4: //Bicycle-Wheel Inequality
        vec_aux = PopBiWheell_origem.GetVectorWheel(Seq);
        break;
      case 5: //sdp
        //vec_aux = complet;
        break;
      default:
        cout << "IN switch OF select_vertices_fromMOstviolatedIneq: Wrong number of separation origem = " << PopBestIneq_origem.get_Data(i).orSep << endl;
      } //end switch
      for (int j = 0; j < (int)vec_aux.size(); j++)
        set_vec.insert(vec_aux[j]);
    } // end FOR i
  }   //end IF max
  ranked_v.resize(set_vec.size());
  cont = 0;
  for (it = set_vec.begin(); it != set_vec.end(); ++it)
  {
    ranked_v[cont] = *it;
    cont++;
  }
}

inline void Selective_Separation(T_Instance &instance)
{
  double cst;
  int Seq;
  std::vector<int> vec_aux;
  std::vector<double> vec_auxD;

  //PopBestIneq.print_all();
  for (int i = 0; i < PopBestIneq.get_CurrentSize() && i < NBMAXINEQ; i++)
  {
    cst = PopBestIneq.get_Data(i).b;
    Seq = PopBestIneq.get_Data(i).orPos;

    switch (PopBestIneq.get_Data(i).orSep)
    {
    case 0: //Triangle
            // 	    for(int j=0; j < 20000; j++)
      DesignTriangleIneq_Mosek(instance, PopTriangle.get_Inequality(Seq), cst);
      break;
    case 1: //Clique inequality
      vec_aux = PopClique.get_vecVertx_Inequality(Seq);
      DesignCliqueIneq_MOSEK(instance, vec_aux, cst);
      break;
    case 2: //General Clique
      vec_aux = PopGenClique.get_vecVertx_Inequality(Seq);
      DesignCliqueIneq_MOSEK(instance, vec_aux, cst);
      break;
    case 3: //Wheel Inequality
      vec_aux = PopWheell.GetVectorWheel(Seq);
      // 	    for(int j=0; j < 20000; j++)
      DesignWheelIneq_MOSEK(instance, vec_aux, cst, 1); // type =1
      break;
    case 4: //Bicycle-Wheel Inequality
      vec_aux = PopBiWheell.GetVectorWheel(Seq);
      DesignWheelIneq_MOSEK(instance, vec_aux, cst, 2); // type =2
      break;
    case 5:

      vec_auxD = PopVecProp.GetVector(Seq);
      //if (SDP_SEP == 6) Design_SDPIneq3(instance, vec_auxD); // the SDP matrix is all of 0,1
      //else
      //for(int j=0; j < 10; j++)
      Design_SDPIneq2(instance, vec_auxD);
      break;
    default:
      cout << "IN switch OF Selective_Separation: Wrong number of separation origem = " << PopBestIneq.get_Data(i).orSep << endl;
    } //end switch
  }   //end for
}

inline void WriteIteration_FILE_BB(std::ofstream &file_ITE, const int &Ite, const int &Size_list, const double &Lb, const double &Ub, const double &gap, const double &time)
{

  if (Ite == -1)
  {                                       // lecture
    file_ITE << fileResult_ITE_BB << " "; //name of file lecture
    file_ITE << SDP_SEP << " ";
    file_ITE << BY_PARTITION_BB << " ";
    file_ITE << SELEC_STRATEGY_BB << " ";
    file_ITE << BRANCHING_RULE_BB << " ";
    file_ITE << STRATEGY_SOL_BB << " ";
    file_ITE << TYPE_BRANCH << " ";
    file_ITE << MAXTIME_ITE << " ";
    file_ITE << endl;

    //Head of table
    file_ITE << "Node,";
    file_ITE << " ActiveNodes,";
    file_ITE << "LowerBound, ";
    file_ITE << " UpperBo, ";
    file_ITE << " gap,";
    file_ITE << " time,";
    file_ITE << endl;
  }
  else
  {
    file_ITE << Ite << " , ";
    file_ITE << Size_list << "  , ";
    file_ITE << setprecision(12) << Lb << " , ";
    file_ITE << setprecision(12) << Ub << "  ,  ";
    file_ITE << setprecision(3) << gap << " , ";
    file_ITE << setprecision(4) << time << " , ";
    file_ITE << endl;
  }

} //end function

inline void PrintScreen_BB(const int &Ite, const int &Size_list, const double &Lb, const double &Ub, const double &gap, const double &time)
{
  if (Ite == 0)
  {                                   // lecture
    cout << fileResult_ITE_BB << " "; //name of file lecture
    cout << SDP_SEP << " ";
    cout << BY_PARTITION_BB << " ";
    cout << SELEC_STRATEGY_BB << " ";
    cout << BRANCHING_RULE_BB << " ";
    cout << STRATEGY_SOL_BB << " ";
    cout << TYPE_BRANCH << " ";
    cout << MAXTIME_ITE << " ";
    cout << endl;

    //Head of table
    cout << "Node,";
    cout << " ActiveNodes,";
    cout << "LowerBound, ";
    cout << " UpperBo, ";
    cout << " gap,";
    cout << " time,";
    cout << endl;
  }

  {
    cout << Ite << ",";
    cout << Size_list << " ,";
    cout << Lb << ", ";
    cout << Ub << " , ";
    cout << gap << " ,";
    cout << time << " ,";
    cout << endl;
  }

} //end function

inline void WriteIteration_FILE(const T_Instance &instance, std::ofstream &file_ITE, const std::string FileLecture, const int &i)
{
  if (i == -1)
  {                          // lecture
    file_ITE << FileLecture; //name of file lecture

    //Consolidate of types
    file_ITE << " K=  " << K;
    //Consolidate of types
    file_ITE << " Types_Cosolidate=  ";
    file_ITE << TypeTriangle << " ";
    file_ITE << TypeHeurClique << " ";
    file_ITE << TypeHeurWheel << " ";
    file_ITE << TypeHeurBiWheel << " ";
    file_ITE << TypeHeurGeClique << " ";
    file_ITE << SDP_SEP << " ";
    file_ITE << PreVP << " ";
    file_ITE << endl;

    //Head of table
    file_ITE << "iteration,";
    file_ITE << " Final_Obj_value,";
    file_ITE << "NbIneq_total, ";
    file_ITE << " TOTAL_SDPCUT, ";
    file_ITE << " MinVP,";
    file_ITE << " time,";
    file_ITE << endl;
  }
  else
  {
    file_ITE << i << " , ";
    file_ITE << instance.ObSol << " , ";
    file_ITE << instance.CONST.size() << " , ";
    file_ITE << TOTAL_SDPCUT << " , ";
    file_ITE << MinVP << " , ";
    file_ITE << getCurrentTime_Double(start);
    file_ITE << " , ite_improv= , " << gapImp << " , ";
    file_ITE << " , ite_time= , " << time_IPM_iteration << " , ";

    //for (int i=0; i< PrintEigPos.size(); i++)
    //file_ITE << PrintEigPos[i] << ", ";

    file_ITE << endl;
  }
}

inline void Print_Iterations_screen(const T_Instance &instance, const std::string &FileLecture, const int &i)
{
  if (i == 0)
  { // lecture
    if (SDP_SEP == -1)
      cout << "SDP solver," << endl;
    if (SDP_SEP == -2)
      cout << "LP_IPM_modification solver," << endl;
    if (SDP_SEP == 0)
      cout << "LP_original solver," << endl;
    if (SDP_SEP >= 1)
      cout << "LP_Eigenvalue solver," << endl;

    cout << FileLecture << endl; //name of file lecture
    cout << "iteration,";
    cout << " Final Obj value,";
    cout << "NbIneq_total, ";
    cout << " TOTAL_SDPCUT, ";
    cout << " MinVP,";
    cout << " time,";
    cout << endl;
  }

  cout << i << ", ";
  cout << instance.ObSol << ", ";
  cout << instance.CONST.size() << ", ";
  cout << TOTAL_SDPCUT << ", ";
  cout << MinVP << ", ";
  cout << getCurrentTime_Double(start) << ", | , ";
  for (int i = 0; i < PrintEigPos.size(); i++)
    cout << PrintEigPos[i] << ", ";

  //cout <<  endl << "NbTri="<<TOTAL_TRI << ", NbCli=" << TOTAL_CLI << ", NbWheel=" <<TOTAL_WHE;
  //       cout <<endl;
}

void WriteFinalResult_File(const std::string &fileResult, const T_Instance &instance, clock_t &time, const int ite)
{
  /*Print in the fileResult*/
  ofstream myfile;
  myfile.open(fileResult.c_str());

  myfile << "*****RESULT OF THE PROGRAM ****** \n"
         << endl;
  myfile << "DIMENTION of X	      = " << instance.edge_nb << "\n";
  myfile << "NbIneq  Max          = " << NBMAXINEQ << "\n";
  myfile << " K                    = " << K << "\n";
  myfile << "\n";
  myfile << "Number of CPA iterations    = " << ite << "\n";
  myfile << "Nb Triangle Inequal.  = " << TOTAL_TRI << "\n";
  myfile << "Nb Clique Inequal.    = " << TOTAL_CLI << "\n";
  myfile << "\n";
  myfile << "Objetive Before T_C inequalities = " /*<< setprecision(15)*/ << Obj1 << "\n";
  myfile << "Final Optimal Objetive (relax)   = " << instance.ObSol << "\n";
  myfile << "ICH Heuristique (Best Value)     = " << 0.0 << "\n";
  myfile << "\n";

  //Cycle Inequality THERE IS NO CYCLE
  myfile << "Nb Eigenvalue Inequality   = " << TOTAL_SDPCUT << endl;
  myfile << " Type = " << SDP_SEP << "_" << PreVP;
  myfile << " (Not_The_Case) " << endl;

  myfile << endl;

  //Wheel Inequalities
  myfile << "Nb Wheel Inequality   = " << TOTAL_WHE << endl;
  myfile << " Type = " << TypeHeurWheel;
  if ((TypeHeurWheel == 0) || (TypeHeurWheel > 4))
    myfile << " (Not_The_Case) " << endl;
  if (TypeHeurWheel == 1)
    myfile << " (GRASP) " << endl;
  if (TypeHeurWheel == 2)
    myfile << " (GENETIC) " << endl;
  if (TypeHeurWheel == 4)
    myfile << " (Greedy) " << endl;

  myfile << endl;

  //BI_Wheel Inequalities
  myfile << "Nb Bicycle_Wheel Inequality   = " << TOTAL_BWH << endl;
  myfile << " Type = " << TypeHeurBiWheel;
  if ((TypeHeurBiWheel == 0) || (TypeHeurBiWheel > 4))
    myfile << " (Not_The_Case) " << endl;
  if (TypeHeurBiWheel == 1)
    myfile << " (GRASP) " << endl;
  if (TypeHeurBiWheel == 2)
    myfile << " (GENETIC) " << endl;
  if (TypeHeurBiWheel == 4)
    myfile << " (Greedy) " << endl;

  myfile << endl;
  //General_CLIQUE Inequalities
  myfile << "Nb General_CLIQUE Inequality   = " << TOTAL_GCL << endl;
  myfile << " Type = " << TypeHeurGeClique;
  myfile << " (GClique) " << endl;

  myfile << endl;
  myfile << "Types_Cosolidate =  ";
  myfile << TypeTriangle << " ";
  myfile << TypeHeurClique << " ";
  myfile << TypeHeurWheel << " ";
  myfile << TypeHeurBiWheel << " ";
  myfile << TypeHeurGeClique << " ";
  myfile << SDP_SEP << " ";
  myfile << PreVP << " ";

  myfile << endl;

  /*Show time*/

  myfile << endl
         << endl
         << "   Total_time_of_execution = " << (double)(clock() - time) / CLOCKS_PER_SEC << " sec";
  myfile << endl
         << "Time_Sep_wheel " << Time_WHE << " sec";
  myfile << endl
         << "Time_Sep_Biwheel " << Time_BWH << " sec";
  myfile << endl
         << "Time_Sep_Clique " << Time_CLI << " sec";
  myfile << endl
         << "Time_Sep_GenClique " << Time_GCL << " sec";
  myfile << endl
         << "Time_Sep_Triangle " << Time_TRI << " sec";
  myfile << endl
         << "Time_SEP_EIG " << Time_SDPCUT << " sec";
  myfile << endl
         << "Time_IPM " << Time_IPM << " sec";
  myfile << endl
         << "MAXTIME " << MAXTIME << " sec";

  myfile << endl;

  if (ite == nb_ITE - 1)
    myfile << " Max number of SDP iterations was achieved " << nb_ITE << "\n";

  myfile.close();

  /* */
  // Print in the ecran
  /*  cout <<endl <<endl;
  cout << "*****RESULT OF THE PROGRAMME ******" <<endl;
  cout << "DIMENTION of X	      = " << DIMBARVAR[0] <<endl;
  cout << "NbIneq  Max          = " << NbIneq << endl;
  cout << " K                    = " << K<< endl;
		  cout << endl ;
  cout << "Number of Res. SDP    = " << ite <<endl;
  cout << "Nb Triangle Inequal.  = " << TriangIne << endl;
  cout << "Nb Clique Inequal.    = " << CliqueIne << endl;
		  cout <<endl;
  cout << "Objetive Before T_C inequalities = "<< setprecision(10) << Obj1 <<endl;
  cout << "Final Optimal Objetive (relax)   = " << Obj[0] <<endl;
  cout << "ICH Heuristique (Best Value)     = "<< BEST << endl;
  cout <<endl;

  if (ite == nb_ITE)
    cout << " numer de resolution atingiu o maximo de " << nb_ITE <<endl <<endl;

    cout << "Total time of execution = " << (double) (clock()-time)/CLOCKS_PER_SEC << "sec" <<endl;
  cout <<endl;

  cout << "Nb Cycle Inequality   = " << CycleIne << "(" << TypeHeurCycle << ")"<<endl;
  cout << "Nb Wheel Inequality   = " << WheelIne << "(" << TypeHeurWheel<< ")"<<endl;
  cout << "Nb Bicycle_Wheel      = " << BiWheelIne << "(" << TypeHeurBiWheel << ")"<<endl;
  cout << "Nb General_CliqueIne  = " << General_CliqueIne << "(" << TypeHeurGeClique << ")"<<endl;
 /* */
}

void Inserting_Parameters_BB(char *argv[])
{
  int aux;
  //fix parameters
  MAXTIME = 10000;
  MAXTIME_ITE = 10;
  ICH = 0;
  TypeTriangle = 1;
  TypeHeurClique = 1;
  TypeHeurWheel = 1;
  TypeHeurWheel = 1;
  TypeHeurGeClique = 0;
  PreVP = 0;

  //from command line
  if (argv[2]) //K
  {
    istringstream ss(argv[2]);
    ss >> K;
  }

  if (argv[3]) //Type of solver (LP = 0, SDP = -1 or LP_EIG=-2)
  {
    istringstream ss(argv[3]);
    ss >> SDP_SEP;
  }

  if (argv[4]) //By_Partition
  {
    istringstream ss(argv[4]);
    ss >> aux;
    if (aux == 0)
      BY_PARTITION_BB = false;
    else
      BY_PARTITION_BB = true;
  }

  //Type of Select candidate
  if (argv[5]) //Se tem Cycle ou nao->(0)
  {
    istringstream ss(argv[5]);
    ss >> SELEC_STRATEGY_BB;

    if (SELEC_STRATEGY_BB > DFS || SELEC_STRATEGY_BB < BeFS)
      SELEC_STRATEGY_BB = DFS; //standard Depth first
  }

  //Type Branch rules
  if (argv[6])
  {
    istringstream ss(argv[6]);
    ss >> BRANCHING_RULE_BB;

    if (BRANCHING_RULE_BB > R7 && BRANCHING_RULE_BB < R1)
      BRANCHING_RULE_BB = R7; //
  }

  //Type of Strategy of solution (Lazy solve after )
  if (argv[7])
  {
    istringstream ss(argv[7]);
    ss >> STRATEGY_SOL_BB;

    if (STRATEGY_SOL_BB > LAZY_BB && STRATEGY_SOL_BB < EAGER_BB)
      STRATEGY_SOL_BB = LAZY_BB; //standard
  }

  //Type  of method (Bnb = branch and boud or BnC branch and cut )
  if (argv[8])
  {
    istringstream ss(argv[8]);
    ss >> TYPE_BRANCH;

    if (TYPE_BRANCH != BnC && TYPE_BRANCH != BnB)
      TYPE_BRANCH = BnB; //standard
  }

  if (argv[9]) //max iteration time
  {
    istringstream ss(argv[9]);
    ss >> MAXTIME_ITE;

    if (MAXTIME_ITE < 0.0)
      MAXTIME_ITE = 10;
  }

  Last_parameter_BB = 0;
  if (argv[10]) //max iteration time
  {
    istringstream ss(argv[10]);
    ss >> Last_parameter_BB;
  }

} //end function

void Inserting_Parameters(char *argv[])
{
  /*Prendre k*/
  if (argv[2])
  {
    istringstream ss(argv[2]);
    ss >> K;
  }

  if (argv[3]) //Numero de inequalities per resolution sdp
  {
    istringstream ss(argv[3]);
    ss >> NBMAXINEQ;
  }

  if (argv[4]) //Se tem ICH ou nao->(0)
  {
    istringstream ss(argv[4]);
    ss >> ICH;
  }

  //Type of Heuristic if it does not use the cycle value is zero (0)
  if (argv[5]) //Se tem Cycle ou nao->(0)
  {
    istringstream ss(argv[5]);
    ss >> TypeTriangle;
  }

  //Type of Heuristic if it does not use the clique value is zero (0)
  if (argv[6]) //Se tem Clique ou nao->(0)
  {
    istringstream ss(argv[6]);
    ss >> TypeHeurClique;
  }

  //Type of Heuristic if it does not use the Wheel value is zero (0)
  if (argv[7]) //Se tem Wheel ou nao->(0)
  {
    istringstream ss(argv[7]);
    ss >> TypeHeurWheel;
  }

  //Type of Heuristic if it does not use the BiWheelIne value is zero (0)
  if (argv[8]) //Se tem Bicycle Wheel ou nao->(0)
  {
    istringstream ss(argv[8]);
    ss >> TypeHeurBiWheel;
  }

  //Type General CLIQUE if  (0)
  if (argv[9]) //Se tem Bicycle Wheel ou nao->(0)
  {
    istringstream ss(argv[9]);
    ss >> TypeHeurGeClique;
  }

  //Type General CLIQUE if  (0)
  if (argv[10]) //Se tem Bicycle Wheel ou nao->(0)
  {
    istringstream ss(argv[10]);
    ss >> SDP_SEP;
  }

  if (argv[11]) //Se tem pre tratamento de eigenvalues ou nao->(0)
  {
    istringstream ss(argv[11]);
    ss >> PreVP;
  }

  if (argv[12]) //max time
  {
    istringstream ss(argv[12]);
    ss >> MAXTIME;
  }
}

void set_FileNamesITE_BB(char *argv[], string &FileResults)
{

  FileResults += "ResultsBB/ITERATIONS_test/";
  for (int i = 0; argv[1][i]; i++)
    FileResults += argv[1][i];

  stringstream sss, ss2, ss3, ss4;
  sss << K;
  FileResults += "K";
  FileResults += sss.str();

  FileResults += "_";

  /*Name by type of parmeters*/
  FileResults += "_";
  ss4 << SDP_SEP;
  ss4 << BY_PARTITION_BB;
  ss4 << SELEC_STRATEGY_BB;
  ss4 << BRANCHING_RULE_BB;
  ss4 << STRATEGY_SOL_BB;
  ss4 << TYPE_BRANCH;
  ss4 << MAXTIME_ITE;

  FileResults += ss4.str();
  FileResults += ".csv"; // kind of excel
}

void set_FileNamesITE(char *argv[], string &FileResults)
{
  FileResults += "ResultsEig/ITERATIONS_tests/";

  for (int i = 0; argv[1][i]; i++)
    FileResults += argv[1][i];

  stringstream ss, ss2, ss3, ss4;
  ss << K;
  FileResults += "K";
  FileResults += ss.str();

  ss2 << NBMAXINEQ;
  FileResults += "_";
  FileResults += ss2.str();

  /*Name by type of heuristics*/
  FileResults += "_";
  if (ICH != 0)
    ss4 << ICH;
  ss4 << TypeTriangle;
  ss4 << TypeHeurClique;
  ss4 << TypeHeurWheel;
  ss4 << TypeHeurBiWheel;
  ss4 << TypeHeurGeClique;
  ss4 << SDP_SEP;
  ss4 << PreVP;

  FileResults += ss4.str();
  FileResults += ".csv"; // kind of excel
}

void set_FileNames(char *argv[], string &FileResults)
{
  FileResults += "ResultsEig/";

  for (int i = 0; argv[1][i]; i++)
    FileResults += argv[1][i];

  stringstream ss, ss2, ss3, ss4;
  ss << K;
  FileResults += "K";
  FileResults += ss.str();

  ss2 << NBMAXINEQ;
  FileResults += "_";
  FileResults += ss2.str();

  /*Name by type of heuristics*/
  FileResults += "_";
  if (ICH != 0)
    ss4 << ICH;
  ss4 << TypeTriangle;
  ss4 << TypeHeurClique;
  ss4 << TypeHeurWheel;
  ss4 << TypeHeurBiWheel;
  ss4 << TypeHeurGeClique;
  ss4 << SDP_SEP;
  ss4 << PreVP;

  FileResults += ss4.str();
}

void Bi_Wheel_InequalitySelective(T_Instance &instance, int &NbIneqMax, const int &type, MKC_ConstraintWheelPopulate &PopWh)
{

  std::vector<int> vec_vertices, aux_vec;
  int typeWheel = 2; //1=Wheel, 2 = bicyle wheel
  int SizeCycleInWheel = 3;
  double rh_cst = (double)typeWheel * ((int)SizeCycleInWheel / 2);

  //cout << "rh_cst = " << rh_cst;

  PopWh.clearAll();

  for (int i = 1; i <= instance.DIM && PopWh.size() < NBMAXINEQ; i++)
  {
    vec_vertices.clear();
    vec_vertices.resize(SizeCycleInWheel + typeWheel, -1); // Changing size to
    vec_vertices[0] = i;                                   // will be fixed in the heuristc methods (cannot change never !!!)

    if (TypeHeurBiWheel == 1)
      GRASP_Wheel(instance, vec_vertices, typeWheel, SizeCycleInWheel, PopWh, rh_cst, 5);

    if (TypeHeurBiWheel == 2)
      GENETIC_Wheel(instance, vec_vertices, typeWheel, SizeCycleInWheel, PopWh);
    // It is not coded
    //if (TypeHeurBiWheel == 3)
    //kSGL_Wheel(intVector,barx,DIMBARVAR,PopWheell,1);

    if (TypeHeurBiWheel == 4)
      Wheel_Heuristic_MultipleSize(instance, vec_vertices, typeWheel, PopWh);

  } //end for i

  //PopWh.printAll();

  //Design Inequlities
  int MAX = NbIneqMax;
  for (int i = 0; i < PopWh.size() && i < MAX; i++)
  {
    aux_vec = PopWh.GetVectorWheel(i);
    //printvector(aux_vec);
    rh_cst = (double)typeWheel * ((int)(aux_vec.size() - typeWheel) / 2); // It is the righ hand of ineq ( wheel <= rh_cst = (typewheel* Lowe(q))); q is the size of wheel

    if (SepMethod2)
    {
      //set_IneqData_in_SepPop(origem_separation,origem_Position, violation, b (right side ineq) )
      PopBestIneq.set_IneqData_in_SepPop(type, i, PopWh.GetInequalityClass(i).violation / (aux_vec.size()), rh_cst); // I divided it for 3 to reduce it importance in the rank of Population
    }
    else
    {
      DesignWheelIneq_MOSEK(instance, aux_vec, rh_cst, typeWheel);
      NbIneqMax--;
    }
  }
}

void Wheel_InequalitySelective_Heuristics(T_Instance &instance, int &NbIneqMax, const int &type, MKC_ConstraintWheelPopulate &PopWh)
{

  std::vector<int> vec_vertices, aux_vec;
  int typeWheel = 1;        //1=Wheel, 2 = bicyle wheel
  int SizeCycleInWheel = 3; //it is the minimum size
  double rh_cst = (double)((int)SizeCycleInWheel / 2);

  PopWh.clearAll();

  for (int i = 1; i <= instance.DIM && PopWh.size() < NBMAXINEQ; i++)
  {
    vec_vertices.clear();
    vec_vertices.resize(SizeCycleInWheel + typeWheel, -1); // Changing size to
    vec_vertices[0] = i;                                   // will be fixed in the heuristc methods (cannot change never !!!)

    if (TypeHeurWheel == 1)
      GRASP_Wheel(instance, vec_vertices, typeWheel, SizeCycleInWheel, PopWh, rh_cst, 2);

    if (TypeHeurWheel == 2)
      GENETIC_Wheel(instance, vec_vertices, typeWheel, SizeCycleInWheel, PopWh);
    // It is not coded
    //if (TypeHeurWheel == 3)
    //kSGL_Wheel(intVector,barx,DIMBARVAR,PopWheell,1);

    if (TypeHeurWheel == 4)
      Wheel_Heuristic_MultipleSize(instance, vec_vertices, typeWheel, PopWh);
  } //end for i

  //PopWh.printAll();
  //int ss;
  //cin >> ss;

  //Design Inequlities
  int MAX = NbIneqMax;
  for (int i = 0; i < PopWh.size() && i < MAX; i++)
  {

    aux_vec = PopWh.GetVectorWheel(i);
    rh_cst = (double)typeWheel * ((int)(aux_vec.size() - typeWheel) / 2); // It is the righ hand of ineq ( wheel <= rh_cst = (typewheel* Lowe(q))); q is the size of wheel
    if (SepMethod2)
    {
      //set_IneqData_in_SepPop(origem_separation,origem_Position, violation, b (right side ineq) )
      PopBestIneq.set_IneqData_in_SepPop(type, i, PopWh.GetInequalityClass(i).violation / (aux_vec.size()), rh_cst); // I divided it for 3 to reduce it importance in the rank of Population
    }
    else
    {
      //printvector(aux_vec);
      DesignWheelIneq_MOSEK(instance, aux_vec, rh_cst, typeWheel);
      NbIneqMax--;
    }
  }
}

//Design and add in the CPLEX model
void DesignWheelIneq_MOSEK(T_Instance &instance, const std::vector<int> &vec_vertices, const double &cstW, const int &typeWheel)
{
  // Disign in CPlex the inequality:
  //	sum(u,v_i) - sum(v_{i-1},v_i) - u_{1,2} <= cstW
  // vec_vertices {u | v_1, v_2, ... , V_n} where u is the hub of the wheel
  int I, J, counter, aux_edge;

  if (typeWheel == 1)
    TOTAL_WHE++;
  else
    TOTAL_BWH++;

  //creating a constraint
  T_constraint Const;

  Const.Origem = 3 + typeWheel; //whee and bicycle wheel
  Const.bkc = MSK_BK_UP;        // Bound key (<=)
  Const.blc = -MSK_INFINITY;    // Numerical value of Lowwer bound
  Const.buc = cstW;             // Numerical value of Upper bound

  //resize by number of edge = nbVertice_cycle + nbVertice_cycle* typeWheel + clique that is formed in the hub of the wheel
  int nbVerCycle = vec_vertices.size() - typeWheel;
  int nbEdgehub = ((typeWheel) * (typeWheel - 1)) / 2;
  Const.vars.resize(nbVerCycle + nbVerCycle * typeWheel + nbEdgehub);
  Const.aval.resize(nbVerCycle + nbVerCycle * typeWheel + nbEdgehub);

  counter = 0;
  // making sum(u,v_i)
  for (int i = 0; i < typeWheel; i++)
  {
    I = vec_vertices[i];
    for (int j = typeWheel; j < (int)vec_vertices.size(); j++)
    {
      J = vec_vertices[j];

      //putting in xpr
      aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
      //cout << aux_edge << " = " << instance.varS[aux_edge]<<", ";
      Const.vars[counter] = (int)aux_edge;
      Const.aval[counter] = 1.0;
      counter++;
    } //end j
  }   // end i

  // making sum(v_{i-1},v_i)
  for (int i = typeWheel; i < (int)vec_vertices.size(); i++)
  {
    I = vec_vertices[i];

    //J Depends on where is i
    if (i == typeWheel)
    {                                            //first vertex of cycle
      J = vec_vertices[vec_vertices.size() - 1]; // last vertex (closing the cycle)
    }
    else
      J = vec_vertices[i - 1];

    //putting in xpr
    aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
    //cout << aux_edge << " = " << instance.varS[aux_edge]<<", ";
    Const.vars[counter] = (int)aux_edge;
    Const.aval[counter] = -1.0; //negatif
    counter++;
  }

  // making u_{1,2} (it works only for bicycle wheel or superior)
  for (int i = 0; i < typeWheel; i++)
  {
    I = vec_vertices[i];
    for (int j = i + 1; j < typeWheel; j++)
    {
      J = vec_vertices[j];

      //putting in
      aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
      Const.vars[counter] = (int)aux_edge;
      Const.aval[counter] = -1.0; //negatif
      counter++;
    } //end j
  }   //end i

  //printvector(instance.varS);
  //cout <<endl;
  //print_Inequality_Tconstraint(Const);
  //cin.get();

  instance.CONST.push_back(Const);
}

bool Wheel_Heuristic_MultipleSize(const T_Instance &instance, std::vector<int> &vec_vertices, const int &typeWheel, MKC_ConstraintWheelPopulate &PopWh)
{
  bool end = false;
  double sum, sum2, best, rh_cst;
  int AuxJ, pos,
      Max_SizeWheel = 2 * K,
      Min_SizeWheel = typeWheel + 3;

  std::set<int> frbdn_vertex;
  std::vector<int> aux_vec;

  vec_vertices.resize(Min_SizeWheel);

  for (int i = 0; i < 1; i++) // (changed to 1 for speed sake )
    frbdn_vertex.insert(vec_vertices[i]);

  //Construction of the cycle in the wheel
  pos = 1; //(changed to 1 for speed sake ) before it was typeWheel

  while (!end)
  {
    best = -MSK_INFINITY;
    AuxJ = -1;
    for (int h = 1; h <= instance.DIM; h++)
    {
      if (!get_exceptionsWheel(instance, vec_vertices, frbdn_vertex, typeWheel, h, pos))
      {
        sum = 0.0;

        // value of (v,v_i)
        if (pos >= typeWheel)
        {
          for (int j = 0; j < typeWheel; j++)
            sum += ValueXij(vec_vertices[j], h, instance);
        }
        else
        {
          for (int j = 0; j < pos; j++)
            sum -= ValueXij(vec_vertices[j], h, instance); //edge in hub of bicycle wheel
        }                                                  //end if else

        // value (v_i-1, v-i)
        if (pos > typeWheel) // all execpt the first vertex of the cycle
          sum -= ValueXij(vec_vertices[pos - 1], h, instance);

        //if last vertex in the wheel
        if (std::size_t(pos) == vec_vertices.size() - 1)
        {
          //creating a new vector to do the local search
          aux_vec = vec_vertices;
          aux_vec[pos] = h;
          //Make local search in new wheel
          sum2 = LocalSearch_Wheel(instance, aux_vec, typeWheel);
          rh_cst = (double)typeWheel * ((int)(aux_vec.size() - typeWheel) / 2); // It is the righ hand of ineq ( wheel <= rh_cst = (typewheel* Lowe(q))); q is the size of wheel
          if (sum2 > rh_cst + epslon)
          {                                              // Wheel inequality violated
            PopWh.SetIneq(aux_vec, sum2, sum2 - rh_cst); // Insertion in Population
            return true;                                 // end of function
          }
          else
          {
            //addding to sum the last edge of external cycle
            sum -= ValueXij(vec_vertices[typeWheel], h, instance); // edge closing the cycle
            if (sum > best)
            {
              AuxJ = h;
              best = sum;
            } //end IF sum
          }   //end else IF violation
        }
        else
        {
          if (sum > best)
          {
            AuxJ = h;
            best = sum;
          } //end IF sum
        }   //end IF ELSE pos == size
      }     //end IF exception
    }       //end for h

    if (AuxJ == -1) // it did not find any vertex to be added in the set
      return false;

    if (std::size_t(pos) == vec_vertices.size() - 1) // it is the last vetex we have the increase the size of vec_vertices
      vec_vertices.push_back(-1);

    if (vec_vertices.size() < (std::size_t)Max_SizeWheel)
    {
      //add  best vetex
      vec_vertices[pos] = AuxJ;

      //increment position
      pos++;
    }
    else
    {
      end = true;
    }
  } //end while

  return false; //not found
}

void GENETIC_Wheel(const T_Instance &instance, std::vector<int> &vec_vertices, const int &typeWheel, const int &SizeCycleInWheel, MKC_ConstraintWheelPopulate &PopWh)
{
  int SizePop = 20;
  double sum, rh_cst;
  MKC_ConstraintWheelPopulate PopGen;
  vector<int> aux_vec;

  //cout << "c 1"; int ss; //cin >> ss;
  Construction_FirstPopulation_Genetic_Wheel(instance, vec_vertices, typeWheel, SizeCycleInWheel, PopGen, -1.0, SizePop); // Thus it will accept all the wheel (not the ones that are actually violated)

  //cout << "Size Pop initial = "<< PopGen.size() << endl;
  // cout << " size = "<< PopGen.size() << "c 2";cin >> ss;

  CrossFitting_Wheel(instance, typeWheel, SizeCycleInWheel, PopGen, -1.0, SizePop);

  //cout << "c 3";

  //cout << "Size Pop After Crosfitting = "<< PopGen.size() << endl;

  Genetic_LocalSearch_Wheel(instance, typeWheel, SizeCycleInWheel, PopGen, -1.0, SizePop);
  //cout << "c 4";cin >> ss;
  //cout << "Size Pop after LocalSearch_Wheel = "<< PopGen.size() << endl;

  for (int i = 0; i < PopGen.size(); i++)
  {
    sum = PopGen.GetInequalityClass(i).sumTotal; // it is the value of the wheel or bicycle wheel
    aux_vec = PopGen.GetVectorWheel(i);
    rh_cst = (double)typeWheel * ((int)(aux_vec.size() - typeWheel) / 2); // wheel <= rh_cst = (typewheel* Lowe(q)); q is the size of wheel
    if (sum > rh_cst + epslon)                                            // Wheel inequality violated
      PopWh.SetIneq(aux_vec, sum, sum - rh_cst);
  } //end for (i)
}

void Genetic_LocalSearch_Wheel(const T_Instance &instance, const int &typeWheel, const int &SizeCycleInWheel,
                               MKC_ConstraintWheelPopulate &PopWh, const double &rh_cst, const int &SizePop)

{
  int maxSize = SizePop;
  double alpha = 0.3,
         sumTotal, sumTotal2;
  std::vector<int> auxVec;

  if (PopWh.size() < SizePop)
    maxSize = PopWh.size();

  for (int i = 0; i < (int)maxSize * alpha; i++)
  {

    auxVec = PopWh.GetVectorWheel(i);
    sumTotal = Evaluation_Wheel(instance, auxVec, typeWheel);

    //Local search
    sumTotal2 = LocalSearch_Wheel(instance, auxVec, typeWheel);
    if ((sumTotal2 > rh_cst + epslon) && (sumTotal2 > sumTotal)) // Wheel inequality violated
      PopWh.SetIneq_NoVerif(auxVec, sumTotal2, sumTotal2 - rh_cst);
  } //end for (i)
}

void CrossFitting_Wheel(const T_Instance &instance, const int &typeWheel, const int &SizeCycleInWheel,
                        MKC_ConstraintWheelPopulate &PopWh, const double &rh_cst, const int &SizePop)
{
  double cross1;
  int sizeParents = 0.5 * PopWh.size();
  vector<int> VecMama,
      VecPapa,
      Offspring;
  double minPapa, minMama,
      maxPapa, maxMama,
      random2, sumTotal;

  if (SizePop < PopWh.size())
    sizeParents = 0.5 * SizePop;

  for (int i = 0; i < sizeParents && sizeParents > 2; i++)
  {
    //WE will have 3 types of crossing
    //First type both parents are strong(have the bests violations)
    VecPapa.clear();
    VecMama.clear();

    if (i < 2 * (sizeParents / 4))
    {
      minPapa = minMama = 0.0;
      maxPapa = maxMama = 0.5;
      SelectMama_and_Papa(VecPapa, VecMama, PopWh,
                          minPapa, maxPapa,
                          minMama, maxMama, SizePop);
    } //end if(sizeParents/4)
    //Second type Father strong and Mother weak (have the bests violations)
    if ((i >= 2 * (sizeParents / 4)) && (i < 3 * (sizeParents / 4)))
    {
      minPapa = 0.0;
      minMama = 0.6;
      maxPapa = 0.5;
      maxMama = 1.0;

      SelectMama_and_Papa(VecPapa, VecMama, PopWh,
                          minPapa, maxPapa,
                          minMama, maxMama, SizePop);

    } //end if(sizeParents/4)
    //Thrid type both parents are weak(have the bests violations)
    if (i >= 3 * (sizeParents / 4))
    {
      minPapa = minMama = 0.60;
      maxPapa = maxMama = 1.0;
      SelectMama_and_Papa(VecPapa, VecMama, PopWh,
                          minPapa, maxPapa,
                          minMama, maxMama, SizePop);

    } //end if(sizeParents/4)

    if ((VecPapa.size() > 0) && (VecMama.size() > 0))
    {
      //Cross of parents (2 types of cross)
      //in the cycle of 4(min cycle) we will take at least 1 from father and mother
      random2 = (0.25 + ((double)(rand() % 75) / 100.0));

      //First son - Take 1st part of father end 2nd of mother
      cross1 = random2 * VecPapa.size();
      Offspring.clear();

      //Taking first part of father
      for (int j = 0; j < cross1; j++)
        Offspring.push_back(VecPapa[j]);

      //taking second part of mother
      //if cross1 is fraction (e.g, 2.5)  the external cycle of the offspring will be increased in one vertex;
      for (int j = cross1; j < (int)VecMama.size(); j++)
        Offspring.push_back(VecMama[j]);

      //Evaluation offspring
      if (validationWheel(instance, Offspring, typeWheel))
      {
        sumTotal = Evaluation_Wheel(instance, Offspring, typeWheel);
        PopWh.SetIneq_NoVerif(Offspring, sumTotal, sumTotal - rh_cst);
      }
      //Include some mutations in offspring
      GeneticALgo_MutationInWheel(instance, Offspring, typeWheel, SizeCycleInWheel);

      //Evaluation offspring
      if (validationWheel(instance, Offspring, typeWheel))
      {
        sumTotal = Evaluation_Wheel(instance, Offspring, typeWheel);
        PopWh.SetIneq_NoVerif(Offspring, sumTotal, sumTotal - rh_cst); //including all inequalities
      }                                                                //end if

    } //end if (size mama and Papa)
  }   //end for (i)
} //end function

//Verificar aqui
inline bool validationWheel(const T_Instance &instance, std::vector<int> &vec_vertices, const int &typeWheel)
{
  if (!ISCOMPLETE_GRAPH)
  {
    { // Verifing if there exist edge with the central vertex
      for (int i = 0; i < typeWheel; i++)
        for (int j = i + 1; j < (int)vec_vertices.size(); j++)
          if (instance.indice_cij[vec_vertices[j] + vec_vertices[i] * (instance.DIM + 1)] == -1)
            return false;
    }

    // verifying the last vertex and possible the first
    {
      for (int i = typeWheel; i < (int)vec_vertices.size(); i++)
      {
        int j = i + 1;
        if (i == (int)vec_vertices.size() - 1)
          j = typeWheel;

        if (instance.indice_cij[vec_vertices[j] + vec_vertices[i] * (instance.DIM + 1)] == -1)
          return false;
      } //end for
    }
  } //end ISCOMPLETE_GRAPH

  for (int i = 0; i < (int)vec_vertices.size(); i++)
    for (int j = i + 1; j < (int)vec_vertices.size(); j++)
      if (vec_vertices[i] == vec_vertices[j])
        return false;

  return true; // the edge h is ok !!!
}

inline void GeneticALgo_MutationInWheel(const T_Instance &instance, std::vector<int> &vec_vertices, const int &typeWheel, const int &SizeCycleInWheel)
{
  int randVert, randPos,
      counter = 0, MaxCounter = 10, auxInt;
  bool end = false;
  std::vector<int> aux_vec;

  aux_vec = vec_vertices;

  while ((!end) && (counter < MaxCounter))
  {

    randPos = typeWheel + (rand() % SizeCycleInWheel); //random position to be changed, cannot change the vetices in the center of the wheel (because !!!)
    randVert = (1 + (rand() % instance.DIM));          //chose randonly a vertex (1 ... instance.DIM) to replace the vec_vertices[Random2]
    auxInt = vec_vertices[randPos];

    aux_vec[randPos] = randVert;

    if (validationWheel(instance, aux_vec, typeWheel))
    {
      vec_vertices[randPos] = randVert;
      end = true;
    }
    else
    {
      counter++;
      aux_vec[randPos] = auxInt;
    }
  } //end while
}

inline void SelectMama_and_Papa(std::vector<int> &VecPapa, std::vector<int> &VecMama, MKC_ConstraintWheelPopulate &PopWh,
                                const double &minPapa, const double &maxPapa,
                                const double &minMama, const double &maxMama, const int &SizePop)
{
  int realSize = SizePop;
  int random2;

  if (PopWh.size() < SizePop)
    realSize = PopWh.size();

  //Chose ramdonly two vectors in the population.
  if (PopWh.size() > 0)
  {

    //chosing papa
    random2 = (int)(minPapa * realSize) + rand() % ((int)(maxPapa * realSize - minPapa * realSize));
    VecPapa = PopWh.GetVectorWheel(random2);

    //chosing mama
    random2 = (int)(minMama * realSize) + rand() % ((int)(maxMama * realSize - minMama * realSize));
    VecMama = PopWh.GetVectorWheel(random2);
  }
}

void Construction_FirstPopulation_Genetic_Wheel(const T_Instance &instance, std::vector<int> &vec_vertices, const int &typeWheel, const int &SizeCycleInWheel,
                                                MKC_ConstraintWheelPopulate &PopWh, const double &rh_cst, const int &SizePop)
{
  double alphaGEN = 0.8; // percentage to be contructed by grasp
  int auxEscape = 0;     // escape of while

  while ((PopWh.size() < SizePop * alphaGEN) && (auxEscape < SizePop * 2))
  {
    GRASP_Wheel(instance, vec_vertices, typeWheel, SizeCycleInWheel, PopWh, rh_cst, 2);
    auxEscape++;
  }

  auxEscape = 0;
  while ((PopWh.size() < SizePop * alphaGEN) && (auxEscape < SizePop * 2))
  {
    CreatRandomWheelIneq(instance, vec_vertices, typeWheel, SizeCycleInWheel, PopWh, rh_cst, SizePop);
    auxEscape++;
  }
}

inline void CreatRandomWheelIneq(const T_Instance &instance, std::vector<int> &vec_vertices, const int &typeWheel, const int &SizeCycleInWheel,
                                 MKC_ConstraintWheelPopulate &PopWh, const double &rh_cst, const int &SizePop)
{
  set<int> frbdn_vertex;
  int auxEscape = 0; // escape of while and for
  int h;
  double sumTotal;

  frbdn_vertex.clear();
  for (std::size_t i = 0; i < vec_vertices.size(); i++)
    frbdn_vertex.insert(vec_vertices[i]);

  for (int i = 1; i < (int)vec_vertices.size(); i++) // it should start in the typewheel element of the vec_vertices (changed to 1 for speed sake )
  {
    h = rand() % instance.DIM + 1;
    if (!get_exceptionsWheel(instance, vec_vertices, frbdn_vertex, typeWheel, h, i))
    {
      vec_vertices[i] = h;
      frbdn_vertex.insert(h);
    }
    else
    {
      i--;
      auxEscape++;
      if (auxEscape > SizePop)
      {
        auxEscape = -1; //next break
        i = vec_vertices.size();
      }
    }
  } //end i

  if (auxEscape != -1)
  {
    sumTotal = LocalSearch_Wheel(instance, vec_vertices, typeWheel);
    PopWh.SetIneq_NoVerif(vec_vertices, sumTotal, sumTotal - rh_cst);
  }
}

void GRASP_Wheel(const T_Instance &instance, std::vector<int> &vec_vertices, const int &typeWheel, const int &SizeCycleInWheel,
                 MKC_ConstraintWheelPopulate &PopWh, const double &rh_cst, const int &NbItGrasp)
{
  // GRASP =
  //	1 - Construction with RCL list,
  //    2 - local search, then
  //	3-  Evaluation

  double sumTotal, sumTotal2;
  for (int i = 0; i < NbItGrasp; i++)
  {
    //printvector(vec_vertices);
    if (Construction_GRASP_Wheel(instance, vec_vertices, typeWheel, SizeCycleInWheel))
    { // the construction was well suceeded !!!

      sumTotal = Evaluation_Wheel(instance, vec_vertices, typeWheel);
      if (sumTotal > rh_cst + epslon) // Wheel inequality violated
        PopWh.SetIneq(vec_vertices, sumTotal, sumTotal - rh_cst);

      sumTotal2 = LocalSearch_Wheel(instance, vec_vertices, typeWheel);
      if ((sumTotal2 > rh_cst + epslon) && (sumTotal2 > sumTotal)) // Wheel inequality violated
        PopWh.SetIneq(vec_vertices, sumTotal2, sumTotal2 - rh_cst);
    }
  } //end for i
}

inline double Evaluation_Wheel(const T_Instance &instance, std::vector<int> &vec_vertices, const int &typeWheel)
{
  // here we calcule the left sizie of the inequality, i.e, sum(v,v_i) - sum(v_{i-1},v_i); here v = vec_vertices[0] --- for wheel;
  double sum = 0.0;

  //bicycle wheel: - (X_{u_1,u_2})
  for (int i = 0; i < typeWheel; i++)
    for (int j = i + 1; j < typeWheel; j++)
      sum -= ValueXij(vec_vertices[i], vec_vertices[j], instance);

  //+sum(v,v_i)
  for (int i = 0; i < typeWheel; i++)
    for (int j = typeWheel; std::size_t(j) < vec_vertices.size(); j++)
      sum += ValueXij(vec_vertices[i], vec_vertices[j], instance);

  //- sum(v_{i-1},v_i)
  for (int j = typeWheel; std::size_t(j) < vec_vertices.size(); j++)
    if (j == typeWheel)
    {
      sum -= ValueXij(vec_vertices[j], vec_vertices[vec_vertices.size() - 1], instance);
    }
    else
    {
      sum -= ValueXij(vec_vertices[j], vec_vertices[j - 1], instance);
    }

  return sum;
}

double LocalSearch_Wheel(const T_Instance &instance, std::vector<int> &vec_vertices, const int &typeWheel)
{
  double sum, best;
  vector<int> aux_vec;

  best = Evaluation_Wheel(instance, vec_vertices, typeWheel);

  //testting each vertex
  for (int i = 1; i < (int)vec_vertices.size(); i++) // it should start in the typewheel element of the vec_vertices (changed to 1 to make it faster to biWHeel)
    for (int h = 1; h <= instance.DIM; h++)
    {
      aux_vec = vec_vertices;
      aux_vec[i] = h;
      if (validationWheel(instance, aux_vec, typeWheel))
      {
        sum = Evaluation_Wheel(instance, aux_vec, typeWheel);
        if (sum > best)
        {
          h = instance.DIM; // break
          vec_vertices = aux_vec;
          best = sum;
        }
      } //end if exception AND
    }   //end FOR h

  return best;
} //end funciton

bool Construction_GRASP_Wheel(const T_Instance &instance, std::vector<int> &vec_vertices, const int &typeWheel, const int &SizeCycleInWheel)
{
  // here we try to minimize the left size of the inequality, i.e, sum(v,v_i) - sum(v_{i-1},v_i); here v = vec_vertices[0] --- for wheel;
  //Here we are constructing the the minimum cycle of the wheel or minimum sum(v_{i-1},v_i)
  // Remember that the vertices in the center it is (v) to wheel and u_1 and u_2 are already in vec_vertices.

  MKC_ConstraintRCLWheelSupport rcl; // List RCL to add a random element
  double sum;
  int AuxJ;
  double propRandom = 0.8;

  std::set<int> frbdn_vertex;

  frbdn_vertex.clear();

  //Construction of the cycle in the wheel
  for (int i = 1; i < (int)vec_vertices.size(); i++) // it should start in the typewheel element of the vec_vertices (change to make it faster to bicycle wheel)
  {
    //Creat the rcl
    rcl.clear();
    for (int h = 1; h <= instance.DIM; h++)
    {
      if (!get_exceptionsWheel(instance, vec_vertices, frbdn_vertex, typeWheel, h, i))
      {
        sum = 0.0;

        // value of (v,v_i)
        if (i >= typeWheel)
        {
          for (int j = 0; j < typeWheel; j++)
            sum += ValueXij(vec_vertices[j], h, instance);
        }
        else
        {
          for (int j = 0; j < i; j++)
            sum -= ValueXij(vec_vertices[j], h, instance); // edge in the hub of bicycle wheel
        }

        // value (v_i-1, v-i)
        if (i > typeWheel) // all execpt the first vertex of the cycle
          sum -= ValueXij(vec_vertices[i - 1], h, instance);

        //if last, subtract the edge to close the cycle
        if (std::size_t(i) == vec_vertices.size() - 1)
          sum -= ValueXij(vec_vertices[typeWheel], h, instance);

        //adding h in the rcl list
        rcl.addElement(h, sum);
      }
    } //end for h

    //rcl.printElement(0);
    //rcl.printAll();
    AuxJ = rcl.getRandomVertex(propRandom, std::rand()); // selecting a random vertex in RCL list with proportion propRandom of selection the first element

    if (AuxJ == -1)
    { // there is no vertex to be added in the wheel
      return false;
    }
    else
      vec_vertices[i] = AuxJ; // add in the vec_vertices

  } //end for i

  return true;
}

//will mostly vefify if the vertex "h" has a edge with each vertex in vec_vertices
inline bool get_exceptionsWheel(const T_Instance &instance, std::vector<int> &vec_vertices, std::set<int> &frbdn_vertex, const int &typeWheel,
                                const int &h, const int &PosInVec)
{
  int aux;
  //std::size_t aux = vec_vertices.size() -1; // last element of vec_vertices
  //looking for forbiden vertices
  {
    if (frbdn_vertex.find(h) != frbdn_vertex.end())
      return true;
  }

  if (!ISCOMPLETE_GRAPH)
  {   //just check if graph is not complete
    { // Verifing if there exist edge with the central vertex
      if (PosInVec >= typeWheel)
      {
        for (int i = 0; i < typeWheel; i++)
          if (instance.indice_cij[h + vec_vertices[i] * (instance.DIM + 1)] == -1)
          {
            frbdn_vertex.insert(h);
            return true;
          } //else
            //if (instance.cij.barc_v [instance.indice_cij[h+vec_vertices[i]*(instance.DIM+1)] ]== 0.0){ //edge is not important to optimization cause has 0 value
            //frbdn_vertex.insert(h);
            //return true;
            //}
      }
      else
      {
        for (int i = 0; i < PosInVec; i++)
          if (instance.indice_cij[h + vec_vertices[i] * (instance.DIM + 1)] == -1)
          {
            frbdn_vertex.insert(h);
            return true;
          } //else
            //if (instance.cij.barc_v[instance.indice_cij[h+vec_vertices[i]*(instance.DIM+1)]] == 0.0){ //edge is not important to optimization cause has 0 value
            //frbdn_vertex.insert(h);
            //return true;
            // }
      }
    }

    /// verifying the last vertex and possible the first
    {
      if (PosInVec > typeWheel)
        if (instance.indice_cij[h + vec_vertices[PosInVec - 1] * (instance.DIM + 1)] == -1)
        {
          frbdn_vertex.insert(h);
          return true;
        } //else

      //if (instance.cij.barc_v[instance.indice_cij[h+vec_vertices[PosInVec-1]*(instance.DIM+1)]] == 0.0){ //edge is not important to optimization cause has 0 value
      //frbdn_vertex.insert(h);
      //return true;
      //}
    }

    // verify if we are in the last
    if (PosInVec == (int)vec_vertices.size() - 1)
      if (instance.indice_cij[h + vec_vertices[typeWheel] * (instance.DIM + 1)] == -1)
      {
        frbdn_vertex.insert(h);
        return true;
      } //else
        //if (instance.cij.barc_v[instance.indice_cij[h+vec_vertices[typeWheel]*(instance.DIM+1)]] == 0.0){ //edge is not important to optimization cause has 0 value
        //frbdn_vertex.insert(h);
        //return true;
        //}
  }     // end IF ISCOMPLETE_GRAPH .

  for (int i = 0; i < PosInVec; i++)
    if (vec_vertices[i] == h)
    {
      frbdn_vertex.insert(h);
      return true;
    }

  //vefifying triangle ineq  (NEW)
  if (TypeTriangle > 0)
  {
    if (PosInVec > typeWheel + 1)
    {

      for (int j = 0; j < typeWheel; j++)
      {
        if (Find_ViolatedTri_in_wheel(instance, h, vec_vertices[j], vec_vertices[PosInVec - 1]))
        {
          frbdn_vertex.insert(h);
          return true;
        }

        if (PosInVec == (int)vec_vertices.size() - 1) // last position
          if (Find_ViolatedTri_in_wheel(instance, h, vec_vertices[j], vec_vertices[typeWheel]))
          {
            frbdn_vertex.insert(h);
            return true;
          } //else
      }
    }
  }

  return false; // the edge h is ok !!!
}

inline bool Find_ViolatedTri_in_wheel(const T_Instance &instance, const int &h, const int &j, const int &i)
{
  double auxRest = 0.0;
  double cst = 1.0;

  //first contraint -> X_ij + X_hj - X_hi <= 1/
  auxRest = ValueXij(i, j, instance) + ValueXij(h, j, instance) - ValueXij(i, h, instance);
  if (auxRest > 1.0 + epslon)
    return true;

  //second contraint -> X_ij - X_hj + X_hi <= 1/
  auxRest = ValueXij(i, j, instance) - 1.0 * ValueXij(h, j, instance) + ValueXij(i, h, instance);
  if (auxRest > 1.0 + epslon)
    return true;

  //thrid contraint -> -X_ij + X_hj + X_hi <= 1/
  auxRest = -1.0 * ValueXij(i, j, instance) + ValueXij(h, j, instance) + ValueXij(i, h, instance);
  if (auxRest > 1.0 + epslon)
    return true;

  return false;
}

void AddAll_TriangleInequality(T_Instance &instance, const int &NbIneqMax, const int &type, MKC_ConstraintTrianglePopulate &PopTri)
{
  // TRIANGLE CONTRAINT -> X_ij + X_hj - X_hi <= 1

  double auxRest = 0.0;
  double cst = 1.0;

  PopTri.clearAll();

  vector<int> aux_vec_vert; // help to find the exeption

  set<MKC_ConstraintTriangle>::iterator it;

  //Verifying all the combinations, i.e, all the triangles in edge
  for (int j = 1; j <= instance.DIM - 2; j++)
  { // it start by 1
    aux_vec_vert.clear();
    aux_vec_vert.push_back(j);
    for (int i = j + 1; i <= instance.DIM - 1; i++)
    { //j+1 pois nao qro xii
      if (!get_exceptionsVector_vertex(instance, aux_vec_vert, i))
      {
        aux_vec_vert.push_back(i);
        for (int h = i + 1; h <= instance.DIM; h++)
        { //i+1 pois nao qro xhh
          if (!get_exceptionsVector_vertex(instance, aux_vec_vert, h))
          { //verifing if the edges exist
            // Now it is ok ... it can analyse the triangle violation

            //first contraint -> X_ij + X_hj - X_hi <= 1/
            //auxRest = ValueXij(i,j,instance) +  ValueXij(h,j,instance) - ValueXij(i,h,instance);
            //if (auxRest > 1.0 + epslon)
            auxRest = 1.5;
            PopTri.addIneq(i, j, h, +1.0, +1.0, -1.0, auxRest);

            //second contraint -> X_ij - X_hj + X_hi <= 1/
            //auxRest = ValueXij(i,j,instance) - 1.0*ValueXij(h,j,instance) + ValueXij(i,h,instance);
            //if (auxRest > 1.0 + epslon)
            PopTri.addIneq(i, j, h, +1.0, -1.0, +1.0, auxRest);

            //thrid contraint -> -X_ij + X_hj + X_hi <= 1/
            //auxRest = -1.0*ValueXij(i,j,instance) +ValueXij(h,j,instance) + ValueXij(i,h,instance);
            //if (auxRest > 1.0 + epslon)
            PopTri.addIneq(i, j, h, -1.0, +1.0, +1.0, auxRest);

          }                      // IF EXECPTION H
        }                        //end for h
        aux_vec_vert.pop_back(); //delete last i in the aux_vec_vert;
      }                          // IF EXECPTION I
    }                            // end for i
  }                              // end for j

  //PopTri.printAll();

  //Putting the nbIneq Clique inequaliClique_InequalitySelective_Heuristicties in the SDP-relax
  int MAX = NbIneqMax;
  for (int i = 0; i < PopTri.size() && i < MAX; i++)
  {
    DesignTriangleIneq_Mosek(instance, PopTri.get_Inequality(i), cst);
  }
}

void TriangleInequalitySelective(T_Instance &instance, int &NbIneqMax, const int &type, MKC_ConstraintTrianglePopulate &PopTri)
{
  // TRIANGLE CONTRAINT -> X_ij + X_hj - X_hi <= 1

  double auxRest = 0.0;
  double cst = 1.0;

  PopTri.clearAll();

  vector<int> aux_vec_vert; // help to find the exeption

  set<MKC_ConstraintTriangle>::iterator it;

  //Verifying all the combinations, i.e, all the triangles in edge
  for (int j = 1; j <= instance.DIM - 2 && PopTri.size() <= NBMAXINEQ; j++)
  { // it start by 1
    aux_vec_vert.clear();
    aux_vec_vert.push_back(j);
    for (int i = j + 1; i <= instance.DIM - 1 && PopTri.size() <= NBMAXINEQ; i++)
    { //j+1 pois nao qro xii
      if (!get_exceptionsVector_vertex(instance, aux_vec_vert, i))
      {
        aux_vec_vert.push_back(i);
        for (int h = i + 1; h <= instance.DIM; h++)
        { //i+1 pois nao qro xhh
          if (!get_exceptionsVector_vertex(instance, aux_vec_vert, h))
          { //verifing if the edges exist
            // Now it is ok ... it can analyse the triangle violation

            //first contraint -> X_ij + X_hj - X_hi <= 1/
            auxRest = ValueXij(i, j, instance) + ValueXij(h, j, instance) - ValueXij(i, h, instance);
            if (auxRest > 1.0 + epslon)
              PopTri.addIneq(i, j, h, +1.0, +1.0, -1.0, auxRest);

            //second contraint -> X_ij - X_hj + X_hi <= 1/
            auxRest = ValueXij(i, j, instance) - 1.0 * ValueXij(h, j, instance) + ValueXij(i, h, instance);
            if (auxRest > 1.0 + epslon)
              PopTri.addIneq(i, j, h, +1.0, -1.0, +1.0, auxRest);

            //thrid contraint -> -X_ij + X_hj + X_hi <= 1/
            auxRest = -1.0 * ValueXij(i, j, instance) + ValueXij(h, j, instance) + ValueXij(i, h, instance);
            if (auxRest > 1.0 + epslon)
              PopTri.addIneq(i, j, h, -1.0, +1.0, +1.0, auxRest);

          }                      // IF EXECPTION H
        }                        //end for h
        aux_vec_vert.pop_back(); //delete last i in the aux_vec_vert;
      }                          // IF EXECPTION I
    }                            // end for i
  }                              // end for j

  //PopTri.printAll();

  //Putting the nbIneq Clique inequaliClique_InequalitySelective_Heuristicties in the SDP-relax
  int MAX = NbIneqMax;
  for (int i = 0; i < PopTri.size() && i < MAX; i++)
  {
    if (SepMethod2)
    {
      //set_IneqData_in_SepPop(origem_separation,origem_Position, violation, b (right side ineq) )
      PopBestIneq.set_IneqData_in_SepPop(type, i, (-(cst - epslon) + 1.0 * PopTri.get_Inequality(i)._val) / 3, cst); // there are 3 vertices
    }
    else
    {
      DesignTriangleIneq_Mosek(instance, PopTri.get_Inequality(i), cst);
      NbIneqMax--;
    }
  }
}

//Design and add in the MOSEK model
void DesignTriangleIneq_Mosek(T_Instance &instance, const MKC_ConstraintTriangle &IneqT, const double &cstT)
{
  // Disign in CPlex the inequality:
  //	sum{vec_vertices} <= 1.0; // for clique inequalty

  //cout << "entrou na inequacao " << endl;
  int I, J, H, aux_edge;
  TOTAL_TRI++;

  //creating a constraint
  T_constraint Const;

  Const.Origem = 1;          //triangle
  Const.bkc = MSK_BK_UP;     // Bound key (<=)
  Const.blc = -MSK_INFINITY; // Numerical value of Lowwer bound
  Const.buc = cstT;          // Numerical value of Upper bound

  //resize to 3 inequalities
  Const.vars.resize(3);
  Const.aval.resize(3);

  // vertex of triangle
  I = IneqT._i;
  J = IneqT._j;
  H = IneqT._h;

  //add x_ij
  aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
  Const.vars[0] = (int)aux_edge;
  Const.aval[0] = IneqT._xij;

  // add x_hj
  aux_edge = instance.indice_cij[J + H * (instance.DIM + 1)];
  Const.vars[1] = (int)aux_edge;
  Const.aval[1] = IneqT._xhj;

  // add x_ih
  aux_edge = instance.indice_cij[I + H * (instance.DIM + 1)];
  Const.vars[2] = (int)aux_edge;
  Const.aval[2] = IneqT._xhi;

  instance.CONST.push_back(Const);
}

//will mostly vefify if the vertex "h" has a edge with each vertex in vec_vertices
inline bool get_exceptionsVector_vertex(const T_Instance &instance, std::vector<int> &vec_vertices, const int &h)
{
  //printvector(vec_vertices);
  //cout << "h = " << h ;
  ///Avoiding inexistente edges (after it must add vertex h in forbinden set)
  for (std::size_t i = 0; i < vec_vertices.size(); i++)
  {
    int aux_edge = h + vec_vertices[i] * (instance.DIM + 1);
    //cout << "edge = " << instance.indice_cij[aux_edge] << ", val = " << instance.cij.barc_v[instance.indice_cij[aux_edge] ] ;
    if (instance.indice_cij[aux_edge] == -1)
    { // if true it means that it does not exist
      return true;
    } //else if (instance.cij.barc_v[instance.indice_cij[aux_edge] ] == 0.0){
      //cout << "Entrou bosta" << ", i=" << instance.cij.barc_i[instance.indice_cij[aux_edge] ] << ", j=" << instance.cij.barc_j[instance.indice_cij[aux_edge] ] << endl; cin.get();
      //return true;
      //}
  }
  //cout << endl;// cin.get();
  return false; // the edge h is ok !!!
}

void GenClique_Inequality(T_Instance &instance, int &NbIneqMax, const int &type, MKC_ConstraintCliquePopulate &PopGen)
{
  // ------------- from Chopra "The partitioning problem"
  // 	sum(x) >= z
  // 			where z = 1/2t(t-1)(k-q) + 1/2*t(t+1)q and sizeClique = p = tK +q;

  int Size_p, // size of clique
      t_GClique, q_GClique;
  double z;
  //After tests
  switch (K)
  {
  case 7:
    Size_p = 11;
    break;
  default:
    Size_p = K + 2;
  }

  //Calculating the "t" and "q" of the General_Clique
  t_GClique = (double)(Size_p) / K;
  q_GClique = Size_p - t_GClique * K;

  //Calcule of the right side of the inequality
  z = (1.0 / 2.0) * t_GClique * (t_GClique - 1) * (K - q_GClique);
  z += (1.0 / 2.0) * t_GClique * (t_GClique + 1) * q_GClique;

  //cout<< "Size_p = " << Size_p << ", t_GClique = " << t_GClique << ",q_GClique = "<< q_GClique<< ", z = " << z << endl;

  //This functions recives (instance = Instances data,NbClique = max number of inequalities,
  //			   Size_p = size clique, z = right hand size of ineq,PopGenClique=population );
  Clique_InequalitySelective_Heuristic(instance, NbIneqMax, Size_p, z, type, PopGen);

  //PopGenClique.printAll();
}

//It is the main function to the Clique inequality
void Clique_InequalitySelective_Heuristic(T_Instance &instance, int &NbIneqMax, const int &SizeClique, const double &cst, const int &type, MKC_ConstraintCliquePopulate &Population)
{
  int count = 0;
  double sum;

  std::vector<int> vec_vertices;
  std::vector<int> vec_aux;
  std::set<int> Fbd_edges;
  std::set<int>::iterator it;

  Population.clearAll();

  //cst = 1.0;
  //cout << "Aqui comeca o clique inequality " << endl;
  //cout << "cst =  " << cst<< " K = "<< K << ", instance.DIM = " << instance.DIM<< endl;

  Fbd_edges.clear();

  for (int i = 0; i < instance.DIM && Population.size() < NBMAXINEQ; i++)
  {

    vec_vertices.clear();
    vec_vertices.push_back(i + 1); //remember in data file vertices start by 1 not 0

    Clique_Heuristique_Selective2(instance, vec_vertices, SizeClique, Fbd_edges);

    if (vec_vertices.size() == std::size_t(SizeClique))
    {

      LocalSearch_CLIQUE(instance, vec_vertices, SizeClique, Fbd_edges); // determinitic descente method try to improve the previous
      sum = Evaluation_Clique(instance, vec_vertices);

      if (sum < cst - epslon)
      {                                                         // violation
        Population.addIneq(vec_vertices, SizeClique, sum, cst); // add new inequality in PopClique
        addFbdnEdges(instance, vec_vertices, Fbd_edges);
      }
    }
  }

  //cout << "Population.size() =" << Population.size() << endl;
  //Population.printAll();

  //Putting the nbIneq Clique inequaliClique_InequalitySelective_Heuristicties in the SDP-relax
  int MAX = NbIneqMax;
  for (int i = 0; i < Population.size() && i < MAX; i++)
  {
    if (SepMethod2)
    {
      //set_IneqData_in_SepPop(origem_separation,origem_Position, violation, b (right side ineq) )
      PopBestIneq.set_IneqData_in_SepPop(type, i, ((cst - epslon) + -1.0 * Population.get_Inequality(i).sum) / SizeClique, cst);
    }
    else
    {
      vec_aux = Population.get_vecVertx_Inequality(i);
      DesignCliqueIneq_MOSEK(instance, vec_aux, cst);
      NbIneqMax--;
    }
  }
}

// THis function is aimed to forumulate the following inequality:
//			r_j + sum_ {i=1}^{j-1} (x_{i,j}) >= 1
// THis inequality will garatee that there existe at least one representative per cluster
//see paper: AN extended edge-representative formulation for the k-partition problem , by Z. Ales, A. Knippel (see eq (9) )
void Design_Representative_OnePerCluster(T_Instance &instance)
{
  int I, J, aux_edge, counter;
  double Rhcst = 1.0;
  //creating a constraint

  for (unsigned j = 0; j < instance.DIM; j++)
  {
    counter = 0;

    T_constraint Const;

    Const.Origem = 21;         //Clique and General clique
    Const.bkc = MSK_BK_LO;     // Bound key (>=)
    Const.blc = Rhcst;         // Numerical value of Lowwer bound
    Const.buc = +MSK_INFINITY; // Numerical value of Upper bound

    //resize to vec_vertices.size() variables
    Const.vars.resize(1 + j); // r + sum until j
    Const.aval.resize(1 + j); // fix

    //Inserting the representative variable (rj)
    Const.vars[counter] = (int)instance.edge_nb + j; // it is the position of r_j in our variables vector
    Const.aval[counter] = +1.0;

    counter++;
    //setting the sum of inequality
    for (unsigned i = 0; i < j; i++)
    {
      I = i + 1;
      J = j + 1; // start with 1
      aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
      Const.vars[counter] = (int)aux_edge;
      Const.aval[counter] = +1.0;
      counter++;
    }

    //inserting constraint in instance
    instance.CONST.push_back(Const);
  }
}

// THis function is aimed to forumulate the following inequality:
//			r_j + x_{i,j}) >= 1 for all i < j
// THis inequality will garatee that there existe at least one representative per cluster
//see paper: AN extended edge-representative formulation for the k-partition problem , by Z. Ales, A. Knippel (see eq (9) )
void Design_RepresentativeIneq_MaxOnePerCluster(T_Instance &instance)
{
  int I, J, aux_edge;
  double Rhcst = 1.0;
  //creating a constraint

  for (unsigned j = 0; j < instance.DIM; j++)
    for (unsigned i = 0; i < j; i++)
    {

      T_constraint Const;

      Const.Origem = 22;         //Clique and General clique
      Const.bkc = MSK_BK_UP;     // Bound key (<=)
      Const.blc = -MSK_INFINITY; // Numerical value of Lowwer bound
      Const.buc = Rhcst;         // Numerical value of Upper bound

      //resize to vec_vertices.size() variables
      Const.vars.resize(2); // r + x_ij
      Const.aval.resize(2); // fix

      //Inserting the representative variable (rj)
      Const.vars[0] = (int)instance.edge_nb + j; // it is the position of r_j in our variables vector
      Const.aval[0] = +1.0;

      //setting the sum of inequality

      I = i + 1;
      J = j + 1; // start with 1
      aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
      Const.vars[1] = (int)aux_edge;
      Const.aval[1] = +1.0;

      //inserting constraint in instance
      instance.CONST.push_back(Const);
    }

  //cout << "Chegou aqui meu brother ";
  //for (unsigned i=0; i < 3; i++)
  //print_Inequality_Tconstraint(instance.CONST[i]);
  //cin.get();
}

// THis function is aimed to forumulate the following inequality:
//			sum (r_j)  <= K for all i < j
// THis inequality will garatee that there existe at least one representative per cluster
//see paper: AN extended edge-representative formulation for the k-partition problem , by Z. Ales, A. Knippel (see eq (9) )
void Design_RepresentativeIneq_SumRepresentative(T_Instance &instance, const int &startRepresVars)
{
  int I, J, aux_edge;
  //creating a constraint

  T_constraint Const;

  Const.Origem = 34;     //Clique and General clique
  Const.bkc = MSK_BK_FX; //MSK_BK_UP  ; // Bound key (<=)
  Const.blc = K;         //-MSK_INFINITY;	// Numerical value of Lowwer bound
  Const.buc = K;         // Numerical value of Upper bound

  //resize to vec_vertices.size() variables
  Const.vars.resize(instance.DIM); // Dimension
  Const.aval.resize(instance.DIM); // fix

  //setting the sum of inequality
  for (unsigned j = 0; j < instance.DIM; j++)
  {
    Const.vars[j] = (int)startRepresVars + j;
    Const.aval[j] = +1.0;
  }

  //inserting constraint in instance
  instance.CONST.push_back(Const);

  //cout << "Chegou aqui meu brother ";
  //for (unsigned i=0; i < 1; i++)
  //print_Inequality_Tconstraint(instance.CONST[i]);
  //cin.get();
}

// THis function is aimed to forumulate the following inequality:
//			x^_ij - x_{i,j} <= 0.0  for all i < j , where  x^_ij  is extended variable
//see paper: AN extended edge-representative formulation for the k-partition problem , by Z. Ales, A. Knippel (see eq (14) )
void Design_ExtendLimitEdge(T_Instance &instance)
{
  int I, J, aux_edge;
  double Rhcst = 0.0;
  int SizeCostMat = (instance.DIM) * (instance.DIM + 1);
  //creating a constraint

  for (unsigned j = 0; j < instance.DIM; j++)
    for (unsigned i = 0; i < j; i++)
      if (ExistEdge(instance, i, j) && i != j)
      {

        T_constraint Const;

        Const.Origem = 31;         //Clique and General clique
        Const.bkc = MSK_BK_UP;     // Bound key (<=)
        Const.blc = -MSK_INFINITY; // Numerical value of Lowwer bound
        Const.buc = Rhcst;         // Numerical value of Upper bound

        //resize to vec_vertices.size() variables
        Const.vars.resize(2); // x^_ij - x_{i,j}
        Const.aval.resize(2); // fix

        I = i + 1;
        J = j + 1; // start with 1

        //setting Extended variable
        aux_edge = instance.indice_cij[SizeCostMat + I + J * (instance.DIM + 1)];
        Const.vars[0] = (int)aux_edge;
        Const.aval[0] = +1.0;

        //setting edge variable (original variables)
        aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
        Const.vars[1] = (int)aux_edge;
        Const.aval[1] = -1.0;

        //     print_Inequality_Tconstraint(Const); cin.get();

        //inserting constraint in instance
        instance.CONST.push_back(Const);
      }

  //   cout << "Chegou aqui meu brother ";
  //for (unsigned i=0; i < 3; i++)
  //print_Inequality_Tconstraint(instance.CONST[i]);
  //   cin.get();
}

// THis function is aimed to forumulate the following inequality:
//			-x^_ij + r_i >= 0 for all i < j, , where  x^_ij  is extended variable
// THis inequality will garatee that there existe at least one representative per cluster
//see paper: AN extended edge-representative formulation for the k-partition problem , by Z. Ales, A. Knippel (see eq (15) )
void Design_ExtendLimitRepresentative(T_Instance &instance)
{
  int I, J, aux_edge;
  double Rhcst = 0.0;
  int StartExtendVars = (instance.DIM) * (instance.DIM + 1);
  int startRepresVars = 2 * (instance.DIM) * (instance.DIM + 1) + instance.DIM + 1 + 1;

  //creating a constraint

  for (unsigned j = 0; j < instance.DIM; j++)
    for (unsigned i = 0; i < j; i++)
      if (ExistEdge(instance, i, j) && i != j)
      {

        T_constraint Const;

        Const.Origem = 32;         //Clique and General clique
        Const.bkc = MSK_BK_UP;     // Bound key (<=)
        Const.blc = -MSK_INFINITY; // Numerical value of Lowwer bound
        Const.buc = Rhcst;         // Numerical value of Upper bound

        //resize to vec_vertices.size() variables
        Const.vars.resize(2); // r + x_ij
        Const.aval.resize(2); // fix

        //Inserting the representative variable (rj)
        aux_edge = instance.indice_cij[startRepresVars + i];
        Const.vars[0] = (int)aux_edge; // it is the position of r_j in our variables vector
        Const.aval[0] = -1.0;

        //setting Extended variables
        I = i + 1;
        J = j + 1; // start with 1
        aux_edge = instance.indice_cij[StartExtendVars + I + J * (instance.DIM + 1)];
        Const.vars[1] = (int)aux_edge;
        Const.aval[1] = +1.0;

        //    print_Inequality_Tconstraint(Const); cin.get();

        //inserting constraint in instance
        instance.CONST.push_back(Const);
      }

  //cout << "Chegou aqui meu brother ";
  //for (unsigned i=0; i < 3; i++)
  //print_Inequality_Tconstraint(instance.CONST[i]);
  //cin.get();
}

// THis function is aimed to forumulate the following inequality:
//			x^_ij + r_i >= 1 for all i < j, , where  x^_ij  is extended variable
// THis inequality will garatee that there existe at least one representative per cluster
//see paper: AN extended edge-representative formulation for the k-partition problem , by Z. Ales, A. Knippel (see eq (15) )
void Design_ExtendWithTwoVars(T_Instance &instance)
{
  int I, J, aux_edge;
  double Rhcst = 1.0;
  int StartExtendVars = (instance.DIM) * (instance.DIM + 1);
  int startRepresVars = 2 * (instance.DIM) * (instance.DIM + 1) + instance.DIM + 1 + 1;

  //creating a constraint

  for (unsigned j = 0; j < instance.DIM; j++)
    for (unsigned i = 0; i < j; i++)
      if (ExistEdge(instance, i, j) && i != j)
      {

        T_constraint Const;

        Const.Origem = 33;         //Clique and General clique
        Const.bkc = MSK_BK_UP;     // Bound key (<=)
        Const.blc = -MSK_INFINITY; // Numerical value of Lowwer bound
        Const.buc = Rhcst;         // Numerical value of Upper bound

        //resize to vec_vertices.size() variables
        Const.vars.resize(3); // r + x_ij
        Const.aval.resize(3); // fix

        //Inserting the representative variable (rj)
        aux_edge = instance.indice_cij[startRepresVars + i];
        Const.vars[0] = (int)aux_edge; // it is the position of r_j in our variables vector
        Const.aval[0] = +1.0;

        //setting Extended variables
        I = i + 1;
        J = j + 1; // start with 1
        aux_edge = instance.indice_cij[StartExtendVars + I + J * (instance.DIM + 1)];
        Const.vars[1] = (int)aux_edge;
        Const.aval[1] = -1.0;

        //setting edge variable (original variable)
        I = i + 1;
        J = j + 1; // start with 1
        aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
        Const.vars[2] = (int)aux_edge;
        Const.aval[2] = +1.0;

        //     print_Inequality_Tconstraint(Const); cin.get();

        //inserting constraint in instance
        instance.CONST.push_back(Const);
      }

  //cout << "Chegou aqui meu brother ";
  //for (unsigned i=0; i < 3; i++)
  //print_Inequality_Tconstraint(instance.CONST[i]);
  //cin.get();
}

// THis function is aimed to forumulate the following inequality:
//			r_j + sum_ {i=1}^{j-1} (x^_{i,j}) = 1
// THis inequality will garatee that there existe at least one representative per cluster
//see paper: AN extended edge-representative formulation for the k-partition problem , by Z. Ales, A. Knippel (see eq (17) )
void Design_ExtendEqualIneq(T_Instance &instance)
{
  int I, J, aux_edge, counter;
  double Rhcst = 1.0;
  int StartExtendVars = (instance.DIM) * (instance.DIM + 1);
  int startRepresVars = 2 * (instance.DIM) * (instance.DIM + 1) + instance.DIM + 1 + 1;
  //creating a constraint

  for (unsigned j = 0; j < instance.DIM; j++)
  {
    counter = 0;

    T_constraint Const;

    Const.Origem = 34;     //Clique and General clique
    Const.bkc = MSK_BK_FX; // Bound key (=)
    Const.blc = Rhcst;     // Numerical value of Lowwer bound
    Const.buc = Rhcst;     // Numerical value of Upper bound

    //resize to vec_vertices.size() variables
    Const.vars.resize(1); // r + sum until j
    Const.aval.resize(1); // fix

    //Inserting the representative variable (rj)
    aux_edge = instance.indice_cij[startRepresVars + j];
    Const.vars[0] = (int)aux_edge; // it is the position of r_j in our variables vector
    Const.aval[0] = +1.0;

    counter++;
    //setting the sum of inequality
    for (unsigned i = 0; i < j; i++)
      if (ExistEdge(instance, i, j) && i != j)
      {
        I = i + 1;
        J = j + 1; // start with 1
        aux_edge = instance.indice_cij[StartExtendVars + I + J * (instance.DIM + 1)];
        Const.vars.push_back((int)aux_edge);
        Const.aval.push_back(+1.0); //[counter] = +1.0;
                                    // 	counter++;
      }

    //     print_Inequality_Tconstraint(Const); cin.get();

    //inserting constraint in instance
    instance.CONST.push_back(Const);
  }
}

void DesignCliqueIneq_MOSEK(T_Instance &instance, const std::vector<int> &vec_vertices, const double &cstCliq)
{

  // Disign in MOSEK the inequality:
  //	sum{vec_vertices} >= 1.0; // for clique inequalty

  //cout << "entrou na inequacao " << endl;
  int I, J, aux_edge;
  if (cstCliq == 1.0)
    TOTAL_CLI++; //clique
  else
    TOTAL_GCL++; // generalclique

  //creating a constraint
  T_constraint Const;

  Const.Origem = 2;          //Clique and General clique
  Const.bkc = MSK_BK_LO;     // Bound key (>=)
  Const.blc = cstCliq;       // Numerical value of Lowwer bound
  Const.buc = +MSK_INFINITY; // Numerical value of Upper bound

  //resize to vec_vertices.size() variables
  int size = vec_vertices.size();
  Const.vars.resize(((size) * (size - 1)) / 2); // fix
  Const.aval.resize(((size) * (size - 1)) / 2); // fix

  int counter = 0;
  //setting the sum of inequality
  for (std::size_t i = 0; i < vec_vertices.size(); i++)
    for (std::size_t j = i + 1; j < vec_vertices.size(); j++)
    {
      I = vec_vertices[i];
      J = vec_vertices[j];
      aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
      Const.vars[counter] = (int)aux_edge;
      Const.aval[counter] = +1.0;
      counter++;
    }

  instance.CONST.push_back(Const);
}

inline double Evaluation_Clique(const T_Instance &instance, const std::vector<int> &vec_vertices)
{
  double sum = 0.0;

  for (std::size_t i = 0; i < vec_vertices.size(); i++)
    for (std::size_t j = i + 1; j < vec_vertices.size(); j++)
      sum += ValueXij(vec_vertices[j], vec_vertices[i], instance);

  return sum;
}

//inserting existent edges of the actual clique in the forbiden set (see get_exceptionsClique in Clique_Heuristique_Selective2)
void addFbdnEdges(const T_Instance &instance, std::vector<int> &vec_vertices, std::set<int> &Fbd_edges)
{
  int aux;
  for (std::size_t i = 0; i < vec_vertices.size(); i++)
    for (std::size_t j = i; j < vec_vertices.size(); j++)
    {
      aux = instance.indice_cij[vec_vertices[i] + vec_vertices[j] * (instance.DIM + 1)]; // where it is in the vals array
      Fbd_edges.insert(aux);
    }
}

//it is a deterministic decend method that tries to improve the clique
void LocalSearch_CLIQUE(const T_Instance &instance, std::vector<int> &vec_vertices,
                        const int &Size_Clique, const std::set<int> &fbdn_edges)
{
  double sum, best;
  vector<int> aux_vec;
  set<int> frbdn_vertex;

  frbdn_vertex.clear();

  for (std::size_t i = 0; i < vec_vertices.size(); i++)
    frbdn_vertex.insert(vec_vertices[i]);

  //testting each vertex
  for (int h = 1; h <= instance.DIM; h++)
  { //remember that in data file vertices start by 1 not 0
    if (!get_exceptionsClique(instance, vec_vertices, frbdn_vertex, h, fbdn_edges))
    {
      best = Evaluation_Clique(instance, vec_vertices);
      //chaging h for each vertex in vec_vertices
      for (std::size_t i = 0; i < vec_vertices.size(); i++)
      {
        aux_vec = vec_vertices;
        aux_vec[i] = h;
        sum = Evaluation_Clique(instance, aux_vec);
        if (sum < best)
        { //Testing the improvement
          vec_vertices = aux_vec;
          i = vec_vertices.size();
          frbdn_vertex.insert(h); // won't try to search in h any more
        }
      } //end for
    }   //end if and h
  }
} //end funciton

void Clique_Heuristique_Selective2(const T_Instance &instance, std::vector<int> &vec_vertices,
                                   const int &Size_Clique, const std::set<int> &fbdn_edges)
{

  double sum, best;
  int v; // vertex and edge

  std::set<int> frbdn_vertex;
  frbdn_vertex.insert(vec_vertices[0]);

  bool end = false;
  while ((vec_vertices.size() < std::size_t(Size_Clique)) && (!end))
  {
    v = -1;

    // loop for searching the best vertex of the graph
    for (int h = 1; h <= instance.DIM; h++) //remember that in data file vertices start by 1 not 0
    {
      best = MSK_INFINITY;
      if (!get_exceptionsClique(instance, vec_vertices, frbdn_vertex, h, fbdn_edges))
      {
        sum = 0.0;
        for (std::size_t i = 0; i < vec_vertices.size(); i++)
          sum += ValueXij(h, vec_vertices[i], instance);

        if (sum < best)
        {
          best = sum;
          v = h;

          if (sum == 0.0) // STOP because it cannot find best sum
            h = instance.DIM + 1;
        }
      } //end IF exception
    }   //end for

    if (v != -1)
    {
      if (vec_vertices.size() < std::size_t(Size_Clique))
      {
        vec_vertices.push_back(v);
        frbdn_vertex.insert(v);
        //cout << " " << v;
      }
      else
        end = true;
    }
    else
    {             // there is no edge with satisfying conditions for last edge
      end = true; //all the vertices are exceptions
    }
  } //end WHILE
} //end function

//return the value en val of the indice i and j.
inline double ValueXij(const int &i, const int &j, const T_Instance &instance)
{
  int aux = instance.indice_cij[i + j * (instance.DIM + 1)];
  if ((aux >= 0) && (aux < instance.edge_nb + 1))
  {
    return instance.varS[aux];
  }
  else
  {
    cout << "MAIN ERRO : Trying to acess in ValueXij the edge (" << i << ", " << j << ") non existent...sorry. End of programme" << endl;
    exit(1);
  }
}

inline bool get_exceptionsClique(const T_Instance &instance, std::vector<int> &vec_vertices,
                                 std::set<int> &frbdn_vertex, const int &h, const std::set<int> &fbdn_edges)
{
  int aux;
  //looking for forbiden vertices
  {
    if (frbdn_vertex.find(h) != frbdn_vertex.end())
      return true;
  }

  ///Avoiding inexistente edges (after it must add vertex h in forbinden set)
  for (std::size_t i = 0; i < vec_vertices.size(); i++)
  {
    aux = h + vec_vertices[i] * (instance.DIM + 1);
    if (instance.indice_cij[aux] == -1)
    {
      frbdn_vertex.insert(h);
      return true;
    } //else {
      //if (instance.cij.barc_v[instance.indice_cij[aux]] == 0.0){ //edge is not important to optimization cause has 0 value
      //frbdn_vertex.insert(h);
      //return true;
      //}
    //}
  }

  //looking for forbiden edges (I should verify if that procedure is efficient)
  {
    for (std::size_t i = 0; i < vec_vertices.size(); i++)
    {
      aux = instance.indice_cij[h + vec_vertices[i] * (instance.DIM + 1)];
      if (fbdn_edges.find(aux) != fbdn_edges.end())
      {
        frbdn_vertex.insert(h); // include vertex in the forbiden list (for speeed sack)
        return true;
      }
    }
  }
  return false; // the edge h is ok !!!
}

/*This function sets the kind of solver:
//   	if 	(Option = -1) mosek SDP
		(Option = -2) mosek LP stopping IPM before
//		(Option = 0 or greater) mosek LP (edge formulatio)
*/

inline void setSolver(T_Instance &instance, MSKrescodee &r, MSKtask_t &task, MSKenv_t &env, const int &Option)
{
  int flag;
  flag = Option;
  if ((flag >= 0) && (flag != 3))
    flag = 1;

  switch (flag)
  {
  case -1: //SDP
    SolveMosekSDP(instance);
    break;
  case -3:                    //SDP
    SolveMosekSDP2(instance); // SDP Stopping SDP before
    break;
  case -2: //LP stopping IPM before end
    SolveMosek2(instance, r, task, env);
    break;
  case 1: //LP
    SolveMosek(instance, r, task, env);
    break;
  case -4:
    SolveMosek(instance, r, task, env);
    break;
  default:
    cout << "IN switch OF setSolver: Wrong number of separation origem = " << Option << endl;
    exit(1);
  } //end switch
} //end function

inline void SolveMosek(T_Instance &instance, MSKrescodee &r, MSKtask_t &task, MSKenv_t &env, int Form_Type)
{

  //   cout << "CHegou SolveMosek" << endl ;
  double auxtime = getCurrentTime_Double(start);

  double *xx;
  double *Obj;
  int DoBasicIPM = false;
  int solveBySIMPLEX = false;

  //   SIMPLEXITE = 0;
  // int LastConstraint = 0;
  //    SIMPLEXITE = 1; // change to never do that again in simplex mode

  instance.ObSol = 0;

  instance.varS.clear();
  instance.varS.resize(instance.totalVars);

  if (Form_Type == 3)
  {
    Form_Type = 0;
  }
  if (Form_Type != 0)
    DoBasicIPM = true;

  // Create the mosek environment.
  r = MSK_makeenv(&env, NULL);

  if (r == MSK_RES_OK)
  {
    //NEW_TASK is a global variable
    if (/*NEW_TASK_LP*/ true)
    { //the idea of reusing the same task is not helpfull -
      // Create the optimization task.
      if (!solveBySIMPLEX || SIMPLEXITE == 0)
        r = MSK_maketask(env, (MSKint32t)instance.CONST.size(), (MSKint32t)instance.totalVars, &task);

      // Directs the log task stream to the 'printstr' function.
      //    if ( r==MSK_RES_OK )
      //      r = MSK_linkfunctotaskstream(task,MSK_STREAM_LOG,NULL,printstr);

      // Append 'numvar' variables.
      //The variables will initially be fixed at zero (x=0).
      if ((r == MSK_RES_OK) && (!solveBySIMPLEX || SIMPLEXITE == 0))
        r = MSK_appendvars(task, (MSKint32t)instance.totalVars);

      //Append obj function and variables bounds (0 <=x<=1)
      if (!solveBySIMPLEX || SIMPLEXITE == 0)
        setObjFunction_andVarBounds__LPmosek(instance, r, task, env, Form_Type);

      //set bounds and input row of constraint
      setConstraint_LPmosek(instance, r, task, LastConstraint);

      /* Maximize objective function. */
      if (r == MSK_RES_OK)
        r = MSK_putobjsense(task, MSK_OBJECTIVE_SENSE_MAXIMIZE);
    }
    else
    {
      //Just add new constraints
      Add_NewConstraintLP_in_taskMosek(instance, r, task);
      //DoBasicIPM = true;
    } //end fof IF new task

    if (r == MSK_RES_OK)
    {
      MSKrescodee trmcode;

      /* Directs the log task stream to the 'printstr' function. */
      //       r = MSK_linkfunctotaskstream(task,MSK_STREAM_LOG,NULL,printstr);

      //Changing number of MSK_IPAR_NUM_THREADS (it should be done after the optimization)
      MSK_putintparam(task, MSK_IPAR_NUM_THREADS, num_of_threads); /*Nber of cpus*/

      if (solveBySIMPLEX)
      { // Solve by Simplex
        r = MSK_putintparam(task, MSK_IPAR_OPTIMIZER, MSK_OPTIMIZER_FREE_SIMPLEX);
        LastConstraint = instance.CONST.size();
        SIMPLEXITE++;
      }
      else
      {                                                                      // solve by IPM
        r = MSK_putintparam(task, MSK_IPAR_OPTIMIZER, MSK_OPTIMIZER_INTPNT); // reinforce but it is the default in MOSEK
        if (!DoBasicIPM)
          r = MSK_putintparam(task, MSK_IPAR_INTPNT_BASIS, MSK_BI_NEVER); //Controls whether the interior-point optimizer also computes an optimal basis.
      }

      // cout << "MSK_RES_OK = " << MSK_RES_OK;
      //  cout << "before R = " << r<< endl;
      /* Run optimizer */
      if (r == MSK_RES_OK)
      {
        r = MSK_optimizetrm(task, &trmcode);
      }

      // cout << "R = " << r<< endl; cin.get();

      /* Print a summary containing information
	about the solution for debugging purposes*/
      //MSK_solutionsummary (task,MSK_STREAM_MSG);

      if (r == MSK_RES_OK)
      {

        MSKsolstae solsta;

        r = MSK_getsolsta(task, MSK_SOL_BAS, &solsta);

        if (solsta != MSK_SOL_STA_OPTIMAL)
        {
          //r = MSK_getsolsta (task,MSK_SOL_ITR,&solsta);
          solsta = MSK_SOL_STA_OPTIMAL;
        }

        switch (solsta)
        {

        case MSK_SOL_STA_OPTIMAL:
        {

          xx = (double *)calloc(instance.totalVars, sizeof(double));
          Obj = (double *)MSK_calloctask(task, 2, sizeof(MSKrealt));

          //if basic solution is activated
          if (solsta == MSK_SOL_STA_OPTIMAL)
          {
            MSK_getxx(task, MSK_SOL_BAS, xx);
            MSK_getprimalobj(task, MSK_SOL_BAS, Obj);
          }
          else
          {
            //Just interior point solution (to save time it is not going to calculate a basic solution)
            MSK_getxx(task, MSK_SOL_ITR, xx);
            MSK_getprimalobj(task, MSK_SOL_ITR, Obj);
          }

          //printf("Optimal primal solution\n");
          //for(i=0; i<NUMVAR; ++i)
          //printf("x[%d]   : % e\n",i,xx[i]);

          instance.ObSol = Obj[0];
          //cout << "instance.ObSol = " << instance.ObSol; cin.get();

          if (Form_Type == 0)
            for (int i = 0; i < instance.edge_nb; i++)
              instance.varS[i] = xx[i];

          break;
        }
        case MSK_SOL_STA_DUAL_INFEAS_CER:
        case MSK_SOL_STA_PRIM_INFEAS_CER:
          printf("Primal or dual infeasibility certificate found.\n");
          break;
        case MSK_SOL_STA_UNKNOWN:
        {
          char symname[MSK_MAX_STR_LEN];
          char desc[MSK_MAX_STR_LEN];
          /* If the solutions status is unknown, print the termination code
        indicating why the optimizer terminated prematurely. */
          MSK_getcodedesc(trmcode,
                          symname,
                          desc);
          printf("The solutuion status is unknown.\n");
          printf("The optimizer terminitated with code: %s\n", symname);
          break;
        }
        default:
          printf("Other solution status.\n");
          break;
        }
      }
      else
      {
        cout << "MOSEK error optimization ... Maybe it is OUT OF MEMORY .... (r != MSK_RES_OK), r = " << r << endl;
        if (r == 1051) // See: https://docs.mosek.com/7.0/capi/Response_codes.html
          cout << "It is DEFINITELY ---- OUT OF MEMORY ----- error !" << endl;

        exit(1);
      }
    }
  } //end first IF

  r = MSK_getdouinf(task, MSK_DINF_OPTIMIZER_TIME, &time_IPM_iteration);
  //time_IPM_iteration = -auxtime + getCurrentTime_Double(start);
  free(xx);

  if (!solveBySIMPLEX)
  {
    MSK_freetask(task, Obj);
    //MSK_deletetask (&task);
  }
  MSK_deleteenv(&env);

  Time_IPM += time_IPM_iteration; // sum time of IPM
}

inline void SolveMosek2(T_Instance &instance, MSKrescodee &r, MSKtask_t &task, MSKenv_t &env)
{
  double auxtime = getCurrentTime_Double(start);

  double *xx;
  double *Obj;
  bool NEW_TASK = true;

  instance.ObSol = 0;

  instance.varS.clear();
  instance.varS.resize(instance.totalVars);

  // Create the mosek environment.
  r = MSK_makeenv(&env, NULL);

  if (r == MSK_RES_OK)
  {
    //NEW_TASK is a global variable
    if (/*NEW_TASK_LP*/ true)
    { // the idea of NEW_TASK_LP is not good (here, it should be always true)
      // Create the optimization task.
      r = MSK_maketask(env, (MSKint32t)instance.CONST.size(), (MSKint32t)instance.totalVars, &task);

      // Directs the log task stream to the 'printstr' function.
      //if ( r==MSK_RES_OK )
      //r = MSK_linkfunctotaskstream(task,MSK_STREAM_LOG,NULL,printstr);

      // Append 'numcon' empty constraints.
      //The constraints will initially have no bounds.

      if (r == MSK_RES_OK)
        r = MSK_appendcons(task, (MSKint32t)instance.CONST.size());

      // Append 'numvar' variables.
      //The variables will initially be fixed at zero (x=0).
      if (r == MSK_RES_OK)
        r = MSK_appendvars(task, (MSKint32t)instance.totalVars);

      //Append obj function and variables bounds (0 <=x<=1)
      setObjFunction_andVarBounds__LPmosek(instance, r, task, env);

      //set bounds and input row of constraint
      setConstraint_LPmosek(instance, r, task);

      //     //set new redundant constraint with the objective value
      //     if (InsertRedu)
      //       Set_REDUNDANT_Ineq_of_FuncObj(instance, r, task);

      /* Maximize objective function. */
      if (r == MSK_RES_OK)
        r = MSK_putobjsense(task, MSK_OBJECTIVE_SENSE_MAXIMIZE);
    }
    else
    {
      //Just add new constraints
      Add_NewConstraintLP_in_taskMosek(instance, r, task); //works when we are reusing task
    }                                                      //end fof IF new task

    // 	r = MSK_linkfunctotaskstream(task,MSK_STREAM_LOG,NULL,printstr);

    if (r == MSK_RES_OK)
    {
      MSKrescodee trmcode;

      //selection of interior point method
      //
      // Parameter of Interior point method
      //
      //
      {
        if (r == MSK_RES_OK)
          r = MSK_putintparam(task, MSK_IPAR_OPTIMIZER, MSK_OPTIMIZER_INTPNT);

        //GAP of Interior point method
        //cout << "TOL = " << TOL; cin.get();
        MSKrealt gapTol = 0.9;
        //MSKrealt gapTolRel; global var

        //Dynamicfddf
        if ((gapImp < MINGAP) && (gapTolRel > 0.5))
        {
          gapTolRel *= 0.25;          //Divided by 4
          MINGAP = MINGAP * RATIOGAP; //Divided by 2
          gapImp = 1.0;               //reset gap of Improvement
        }

        //cout << gapTolRel; cin.get();

        //r = MSK_putdouparam(task,MSK_DPAR_INTPNT_TOL_REL_GAP, gapTol); /*Nber of cpus*/
        //r = MSK_putintparam(task, MSK_IPAR_INTPNT_MAX_ITERATIONS,5); //Controls the maximum number of iterations allowed in the interior-point optimizer.
        r = MSK_putintparam(task, MSK_IPAR_INTPNT_BASIS, MSK_BI_NEVER); //Controls whether the interior-point optimizer also computes an optimal basis.

        r = MSK_putdouparam(task, MSK_DPAR_INTPNT_TOL_MU_RED, gapTol);     //Relative complementarity gap tolerance.
        r = MSK_putdouparam(task, MSK_DPAR_INTPNT_TOL_PFEAS, gapPrimal);   // primal feasibility
        r = MSK_putdouparam(task, MSK_DPAR_INTPNT_TOL_DFEAS, gapTol);      // dual feasibility
        r = MSK_putdouparam(task, MSK_DPAR_INTPNT_TOL_REL_GAP, gapTolRel); // Tol to optimality

        //r = MSK_putdouparam(task,MSK_DPAR_OPTIMIZER_MAX_TIME, 1.0); // Tol to optimality
        //r = MSK_putintparam(task, MSK_IPAR_INTPNT_STARTING_POINT, 1); //best case is 0 (is by default) already tested
        //r = MSK_putdouparam(task,MSK_DPAR_INTPNT_TOL_PATH, TOL); // «best case is 0 (is by default) already tested

        //Directs the log task stream to the 'printstr' function.
        //if ( r==MSK_RES_OK )
        //r = MSK_linkfunctotaskstream(task,MSK_STREAM_LOG,NULL,printstr);
      }

      //Changing number of MSK_IPAR_NUM_THREADS (it should be done after the optimization)
      MSK_putintparam(task, MSK_IPAR_NUM_THREADS, num_of_threads); //Nber of cpus

      /* Run optimizer */
      if (r == MSK_RES_OK)
        r = MSK_optimizetrm(task, &trmcode);

      /* Print a summary containing information
	about the solution for debugging purposes*/
      //MSK_solutionsummary (task,MSK_STREAM_MSG);

      if (r == MSK_RES_OK)
      {

        MSKsolstae solsta;

        //if ( r==MSK_RES_OK )
        //r = MSK_getsolsta (task,MSK_SOL_BAS,&solsta);
        if (r == MSK_RES_OK)
          r = MSK_getsolsta(task, MSK_SOL_ITR, &solsta);

        //cout << "entrou aqui = " << solsta << endl;
        //  cin.get();
        solsta = MSK_SOL_STA_OPTIMAL;

        switch (solsta)
        {
        case MSK_SOL_STA_OPTIMAL:
        {

          xx = (double *)calloc(instance.totalVars, sizeof(double));
          Obj = (double *)MSK_calloctask(task, 2, sizeof(MSKrealt));

          MSK_getxx(task,
                    MSK_SOL_ITR,
                    xx);

          MSK_getprimalobj(task,
                           MSK_SOL_ITR,
                           Obj);

          instance.ObSol = Obj[0];

          for (int i = 0; i < instance.edge_nb; i++)
            instance.varS[i] = xx[i];

          break;
        }
        case MSK_SOL_STA_DUAL_INFEAS_CER:
        case MSK_SOL_STA_PRIM_INFEAS_CER:
          printf("Primal or dual infeasibility certificate found.\n");
          break;
        case MSK_SOL_STA_UNKNOWN:
        {
          char symname[MSK_MAX_STR_LEN];
          char desc[MSK_MAX_STR_LEN];
          /* If the solutions status is unknown, print the termination code
	    indicating why the optimizer terminated prematurely. */
          MSK_getcodedesc(trmcode,
                          symname,
                          desc);
          printf("The solutuion status is unknown.\n");
          printf("The optimizer terminitated with code: %s\n", symname);
          break;
        }
        default:
          printf("Other solution status.\n");
          break;
        }
      }
    }
  } //end first IF

  free(xx);
  MSK_freetask(task, Obj);

  r = MSK_getdouinf(task, MSK_DINF_OPTIMIZER_TIME, &time_IPM_iteration);

  Time_IPM += time_IPM_iteration; // sum time of IPM

  //cout << "Saiu da function "<<endl;
}

//set initial solution (it will be cut (0.0) if its value is greater than 0.0)

inline double setInitial_LP_Solution_antigo(T_Instance &instance)
{
  instance.varS.clear();
  instance.varS.resize(instance.edge_nb);
  double val, obj = 0.0;

  for (int i = 0; i < instance.edge_nb; i++)
  {
    val = instance.cij.barc_v[i];
    if (val > 0.0)
    {
      instance.varS[i] = 0.0;
      obj += val;
    }
    else
      instance.varS[i] = 1.0;
  }

  instance.ObSol = obj + instance.sum_cost;

  return 0.0;
}

inline void setInitial_LP_Solution_justVar(T_Instance &instance)
{
  instance.varS.clear();
  instance.varS.resize(instance.edge_nb);
  double val, obj = 0.0;

  for (int i = 0; i < instance.edge_nb; i++)
  {
    val = instance.cij.barc_v[i];
    if (val > 0.0)
    {
      instance.varS[i] = 0.0;
      obj += val;
    }
    else
      instance.varS[i] = 1.0;
  }

  instance.ObSol = obj + instance.sum_cost;
}

inline double setInitial_LP_Solution(T_Instance &instance)
{

  //cout << "Chegou aqui brother ... first LP bound based on k" << endl;
  instance.varS.clear();
  instance.varS.resize(instance.edge_nb, 0.0);
  double val, obj = 0.0, objOrig = 0.0, up;
  double aux = (pow(((double)instance.DIM / K), 2.0)) * ((K * (K - 1)) / 2.0);
  int maxNbEdgeCUT = (int)aux;

  //cout << " maxNbEdgeCUT =" <<maxNbEdgeCUT << ", before it was =" << instance.edge_nb  ; //cin.get();
  //printvector(instance.varS);
  //cin.get();

  set<MKC_InstanceRankVertices>::iterator it;
  set<MKC_InstanceRankVertices> ranked;

  for (int i = 0; i < instance.edge_nb; i++)
  {
    val = instance.cij.barc_v[i];
    if (val > 0)
    {
      MKC_InstanceRankVertices rkVer(i, val);
      ranked.insert(rkVer);
    }
  }

  int counter = 0;
  obj = 0.0;
  for (it = ranked.begin(); it != ranked.end() && counter < maxNbEdgeCUT; ++it)
  {
    obj += (*it).weight;
    counter++;
  }

  //Upper bound based on articles of Nikiforov and Dam&Sotirov
  if (true)
  {
    up = UpperBoundSmallestEigenvalue(instance);
    if (up < obj)
    {
      obj = up;
      COMPLET_EIG = true;
    }
  }
  //settig variables and objOrig (Maximum (all possitive edges are cut))
  objOrig = 0.0;
  for (int i = 0; i < instance.edge_nb; i++)
  {
    val = instance.cij.barc_v[i];
    if (val > 0.0)
    {
      objOrig += val;
      instance.varS[i] = 0.0;
    }
    else
      instance.varS[i] = 1.0;
  }
  //cin.get();
  instance.ObSol = objOrig + instance.sum_cost;
  //cout <<"objOrig = " << objOrig << ", Obj = " << obj; cin.get();
  //cout << " maxNbEdgeCUT =" <<maxNbEdgeCUT << ", print instance.varS antes =";
  //printvector(instance.varS);
  //cin.get();

  return objOrig - obj; //(return the savings)
}

double Hybrid_GraspAndTabu(const T_Instance &instance)
{
  clock_t start2 = std::clock(); // mesure time
  bool resp = true;
  int NbIte_Tabu = 20, NbIte_GRASP = 5, tabuSize = 7;
  double BestPartitionsValue = 0.0;
  std::vector<std::vector<int>> Partitions;
  std::vector<std::vector<int>> BestPartitions;
  std::vector<int> tabuList(instance.DIM, 0);

  BestPartitionsValue = 0.0;

  for (unsigned j = 0; j < NbIte_GRASP; j++)
  {
    Partitions.clear();
    if (j % 2 == 0)
      Initial_solution_to_GraspFeasible(instance, 0, 0.5, Partitions);
    else
      GenerateRandomSolution(instance, Partitions);
    LocalSearch_Solution(instance, Partitions);
    AnalyzeNewPartition(instance, Partitions, BestPartitions, &BestPartitionsValue);

    //The tabu partie
    resp = true;
    for (unsigned i = 0; i < NbIte_Tabu && resp; i++)
    {
      resp = Tabu_ModifyOneVertex_best(instance, Partitions, tabuList, tabuSize);
      if (resp)
        AnalyzeNewPartition(instance, Partitions, BestPartitions, &BestPartitionsValue);
      if (resp)
        ChangeTabu_decrease(tabuList);
    } //end FOR I
  }   // end for J

  LocalSearch_Solution(instance, BestPartitions);

  BestPartitionsValue = Get_valueOfPartition(instance, BestPartitions);

  //Printing solution
  //cout << "Solution of Hybrid_GraspAndTabu" << endl;
  cout << "Hybrid Value = " << BestPartitionsValue << ", time = " << getCurrentTime_Double(start2) << endl;
  //PrintPartitionICH(BestPartitions); cin.get();
  return BestPartitionsValue;
}

double VNS_Heuristic_FeasibleSoltion(const T_Instance &instance)
{
  clock_t start2 = std::clock(); // mesure time
  bool resp = true;
  int NbIte = 10, p, p_max = 8, p_initial = 2;

  //small instances we can do more
  if (instance.DIM <= 300)
  {
    NbIte = 300;
    p_max = 12;
  }

  double BestPartitionsValue = 0.0, Newval;
  std::vector<std::vector<int>> Partitions;
  std::vector<std::vector<int>> BestPartitions;
  std::vector<int> tabuList(instance.DIM, 0);

  GenerateRandomSolution(instance, Partitions);
  LocalSearch_Solution(instance, Partitions);

  BestPartitions = Partitions;
  BestPartitionsValue = Get_valueOfPartition(instance, BestPartitions);

  for (unsigned j = 0; j < NbIte; j++)
  {
    p = p_initial;
    while (p < p_max)
    {

      for (unsigned i = 0; i < tabuList.size(); i++)
        tabuList[i] = 0;

      for (unsigned i = 0; i < p; i++)
        resp = Tabu_ModifyOneVertex_best(instance, Partitions, tabuList, p);

      LocalSearch_Solution(instance, Partitions);

      Newval = Get_valueOfPartition(instance, Partitions);

      if (Newval > BestPartitionsValue)
      {
        // 	cout << "BestPartitionsValue = " << BestPartitionsValue << endl;
        // 	cout << " Changed ... to Newval = " << Newval << endl;
        //PrintPartitionICH(BestPartitions); cin.get();

        // 	cout << "Value is = " << Get_valueOfPartition (instance,Partitions) +  instance.sum_cost;
        // 	cin.get();

        BestPartitions = Partitions;
        BestPartitionsValue = Newval;
        p = p_initial;
      }
      else
      {
        // 	cout << "P = " << p; cin.get();
        p++;
      }
    } //end WHILE p

    // cout << " Best val = " << BestPartitionsValue << endl;

    //genereate random solution
    if (j < NbIte - 1)
    {
      Partitions.clear();
      GenerateRandomSolution(instance, Partitions);
      LocalSearch_Solution(instance, Partitions);
    }

  } // end FOR itertation

  //Printing solution
  //cout << "Solution of Hybrid_GraspAndTabu" << endl;
  //  cout << "VNS Value = " << BestPartitionsValue << ", time = " << getCurrentTime_Double(start2)<<  endl;
  //PrintPartitionICH(BestPartitions); cin.get();
  //cin.get();
  return BestPartitionsValue + instance.sum_cost;
}

//akkiii
//void NewLocalSearch_Solution (const T_Instance &instance,std::vector< std::vector<int> > &Partitions)
//{
//  int    bestNewPartition, I, J, aux_edge,
// 	  counter = 0, max_iteration = 20;
//  double sumEdge_SP, sumEdge, bestSumEdge;
//  //PrintPartitionICH(Partitions); //cin.get();
//  //cout << "Initial value = " << Get_valueOfPartition (instance,Partitions ) << endl;

//    vector<int> FindPartition_origem(instance.DIM+1, -1);
//  vector<int> FindPartitionPosition_origem(instance.DIM+1, -1);
//  double gain;

//  //vector of origem partition
//  for (unsigned p=0; p < Partitions.size(); ++p )
//    for ( unsigned pp =0; pp< Partitions[p].size(); ++pp){
//		FindPartition_origem[Partitions[p][pp]] = p;
//		FindPartitionPosition_origem[Partitions[p][pp]] = pp;
//	}
////Create gain per partition
// while ((CHANGES) && (counter <max_iteration)){
//   counter ++;
//   CHANGES = false;
//	 for (unsigned i=0; i < instance.DIM-1; ++i ){
//	 	P_orig = FindPartition_origem[i];
//	 	for (unsigned p=0; p < Partitions.size(); ++p ){
//			gain = gain_movePartition(instance, Partitions, i,P_orig, p);
//			if (gain > 0){
//	         Partitions[p].push_back(i);	//add element ii in partition "bestNewPartition"
//		  	 Partitions[P_orig].erase (Partitions[P_orig].begin()+FindPartitionPosition_origem[i]);			//erase ii from i partition
//			 FindPartition_origem[i] = p;
//			 FindPartitionPosition_origem[i] = Partitions[p].size()-1;
//			}
//		}
//	 }//end for i
// }//end while

//  if (Partitions.size() < K)
//	Partitions.resize(K);

//  bool  CHANGES = true;

//  while ((CHANGES) && (counter <max_iteration)){
//   counter ++;
//   CHANGES = false;
//  for (unsigned i=0; i < Partitions.size(); i++ ){

//    for ( unsigned ii =0; ii< Partitions[i].size(); ii++){
//      //calcule if we take vertex ii from partition i
//      sumEdge_SP = 0.0;
//      for ( unsigned iii =0; iii< Partitions[i].size(); iii++){
//	if (iii == ii) continue;

//	I = Partitions[i][ii] +1; // start by 1
//	J = Partitions[i][iii] +1; // start by 1

//	aux_edge = instance.indice_cij[I+J*(instance.DIM+1)];
//	if (aux_edge != -1)
//		sumEdge_SP += instance.cij.barc_v[aux_edge];

//      }

//      //cout << "sumEdge_SP = " << sumEdge_SP << endl;

//      {//find best partition to be put

//	bestNewPartition =  -1;
//	bestSumEdge = D_INFINITY;
//	I = Partitions[i][ii] +1; // start by 1

//	for (unsigned j=0; j < Partitions.size(); j++ ){
//	  if (i == j) continue; // do nothing
//         sumEdge = 0.0;
//	  for ( unsigned jj =0; jj< Partitions[j].size(); jj++){
//	    J = Partitions[j][jj] +1; // start by 1
//	      aux_edge = instance.indice_cij[I+J*(instance.DIM+1)];
//	      if (aux_edge != -1)
//	      	sumEdge += instance.cij.barc_v[aux_edge];
//	  }//end 	FOR ii and jj

//	  if (((sumEdge < sumEdge_SP) &&(sumEdge < bestSumEdge )) ){
//  		//cout << "Partition " << j << ",  sumEdge_SP ="<<  sumEdge_SP<< ",  sumEdge = " << sumEdge << endl ; cin.get();
//	    bestSumEdge  = sumEdge ;
//	    bestNewPartition  = j;
//	  }
//	 }//end for J

//      }//end find best partition

//      if (bestNewPartition != -1){
//         Partitions[bestNewPartition].push_back(Partitions[i][ii]);	//add element ii in partition "bestNewPartition"
//	  Partitions[i].erase (Partitions[i].begin()+ii);			//erase ii from i partition
//	  ii--;
//  	  CHANGES = true;
//      }//end of if Bestpartition

//    }//end FOR ii
//  }//end FOR i
//  }//end of WHILE

//   //cout << "Finalllll value = " << Get_valueOfPartition (instance,Partitions ) << endl;
//   //PrintPartitionICH(Partitions); cin.get();

//}//end function

double MultipleSearch_Heuristic_FeasibleSoltion(const T_Instance &instance)
{
  clock_t start2 = std::clock(); // mesure time
  bool resp = true;
  int NbIte_Tabu = 10, tabuSize = 10, MaxNbRound = 50;
  double BestPartitionsValue = 0.0,
         Newval;
  std::vector<std::vector<int>> Partitions;
  std::vector<std::vector<int>> BestPartitions;
  std::vector<int> tabuList(instance.DIM, 0);

  for (int round = 0; round < MaxNbRound; ++round)
  {
    Partitions.clear();
    GenerateRandomSolution(instance, Partitions);
    LocalSearch_Solution(instance, Partitions);
    LocalSearch_TwoMoves_Solution(instance, Partitions);

    for (unsigned i = 0; i < NbIte_Tabu && resp; i++)
    {
      resp = Tabu_ModifyOneVertex_best(instance, Partitions, tabuList, tabuSize);
      if (resp)
        AnalyzeNewPartition(instance, Partitions, BestPartitions, &BestPartitionsValue);
      if (resp)
        ChangeTabu_decrease(tabuList);
    }

    for (unsigned i = 0; i < instance.DIM; ++i)
      tabuList[i] = 0;

    LocalSearch_Solution(instance, Partitions);
    LocalSearch_TwoMoves_Solution(instance, Partitions);
    Newval = Get_valueOfPartition(instance, Partitions);

    AnalyzeNewPartition(instance, Partitions, BestPartitions, &BestPartitionsValue);
  } //end for ROUND

  //	cout << "MOH Value = " << BestPartitionsValue << ", time = " << getCurrentTime_Double(start2)<<  endl;

  return BestPartitionsValue + instance.sum_cost;
}

double TabuOptimum_Heuristic_FeasibleSoltion(const T_Instance &instance)
{

  clock_t start2 = std::clock(); // mesure time
  bool resp = true;
  int NbIte_Tabu = 1000, tabuSize = 10;
  double BestPartitionsValue = 0.0;
  std::vector<std::vector<int>> Partitions;
  std::vector<std::vector<int>> BestPartitions;
  std::vector<int> tabuList(instance.DIM, 0);

  GenerateRandomSolution(instance, Partitions);
  LocalSearch_Solution(instance, Partitions);

  BestPartitions = Partitions;
  BestPartitionsValue = Get_valueOfPartition(instance, BestPartitions);
  //  cout << "Before Value = " << BestPartitionsValue << endl;

  for (unsigned i = 0; i < NbIte_Tabu && resp; i++)
  {
    resp = Tabu_ModifyOneVertex_best(instance, Partitions, tabuList, tabuSize);
    if (resp)
      AnalyzeNewPartition(instance, Partitions, BestPartitions, &BestPartitionsValue);
    if (resp)
      ChangeTabu_decrease(tabuList);
  }

  LocalSearch_Solution(instance, Partitions);
  LocalSearch_TwoMoves_Solution(instance, Partitions);
  BestPartitionsValue = Get_valueOfPartition(instance, Partitions);

  // Printing values
  //cout << "Solution of Tabu" << endl;

  cout << "Tabu Value = " << BestPartitionsValue << ", time = " << getCurrentTime_Double(start2) << endl;
  //PrintPartitionICH(BestPartitions); cin.get();
  return BestPartitionsValue;
}

inline void ChangeTabu_decrease(std::vector<int> &tabuList)
{
  for (unsigned i = 0; i < tabuList.size(); i++)
    if (tabuList[i] > 0)
      tabuList[i]--;
}
bool Tabu_ModifyOneVertex_best(const T_Instance &instance, std::vector<std::vector<int>> &Partitions, std::vector<int> &TabuList, const int &TabuListSize)
{
  int I, J, aux_edge, bestVertex_i, posBestVertex_i, OrigemPartition_i, DestinationPatition_j;
  double sumEdge, bestSumEdge;

  bestSumEdge = D_INFINITY; //
  bestVertex_i = -1;

  for (unsigned i = 0; i < Partitions.size(); i++)
  {
    if (Partitions[i].size() <= 1)
      continue; //Will change just if Partition size is bigger than 1

    for (unsigned j = 0; j < Partitions.size(); j++)
    {
      if (i == j)
        continue; //do nothing

      for (unsigned ii = 0; ii < Partitions[i].size(); ii++)
      {
        if (TabuList[Partitions[i][ii]] != 0)
          continue; //not autorized move
        sumEdge = 0.0;
        for (unsigned jj = 0; jj < Partitions[j].size(); jj++)
        {

          I = Partitions[i][ii] + 1; // start by 1
          J = Partitions[j][jj] + 1; // start by 1

          aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];

          if (aux_edge != -1)
            sumEdge += instance.cij.barc_v[aux_edge];

          //        cout << "x{"<< I <<","<< J <<  "} -- cost = " << instance.cij.barc_v[aux_edge] << endl; cin.get();
        } //end FOR jj

        if (sumEdge < bestSumEdge)
        {
          bestSumEdge = sumEdge;
          posBestVertex_i = ii;
          bestVertex_i = Partitions[i][ii];
          OrigemPartition_i = i;
          DestinationPatition_j = j;
        }

      } //end FOR ii
    }   //end FOR j
  }     //end FOR i

  //Modify Partitions
  if (bestVertex_i != -1)
  {
    Partitions[OrigemPartition_i].erase(Partitions[OrigemPartition_i].begin() + posBestVertex_i); //erase vertex from origem
    Partitions[DestinationPatition_j].push_back(bestVertex_i);
    TabuList[bestVertex_i] = TabuListSize;
  }
  else
  {
    return false;
  }
}

//random generator of feasible soltion for MKC
inline void GenerateRandomSolution(const T_Instance &instance, std::vector<std::vector<int>> &Partitions)
{
  int random;
  Partitions.resize(K); //resize partition to K partitions

  for (unsigned i = 0; i < instance.DIM; i++)
  {
    random = (rand() % K);
    Partitions[random].push_back(i);
  }

  //Solution of random generator
  //cout << "Solution of random generator" << endl;
  //cout << "Value of random solution = " << Get_valueOfPartition (instance,Partitions ) << endl;
  //PrintPartitionICH(Partitions); cin.get();
}

double Grasp_Heuristic_FeasibleSoltion(const T_Instance &instance)
{
  clock_t start2 = std::clock(); // mesure time
  double BestPartitionsValue = 0.0;
  //cout << "Begin of  Grasp_Heuristic_FeasibleSoltion " << endl;

  int NbIte_Grasp = 100;
  std::vector<std::vector<int>> Partitions;
  std::vector<std::vector<int>> BestPartitions;

  for (unsigned i = 0; i < NbIte_Grasp; i++)
  {
    Partitions.clear();

    if (i % 2 == 0)
      Initial_solution_to_GraspFeasible(instance, 0, 0.5, Partitions);
    else
      GenerateRandomSolution(instance, Partitions);
    LocalSearch_Solution(instance, Partitions);

    if (i % 5 == 0)
      LocalSearch_TwoMoves_Solution(instance, Partitions);

    AnalyzeNewPartition(instance, Partitions, BestPartitions, &BestPartitionsValue);
  }

  // cout << "GRASP value = " << BestPartitionsValue<< ", time = " << getCurrentTime_Double(start2) << endl;
  //PrintPartitionICH(BestPartitions); //cin.get();
  return BestPartitionsValue + instance.sum_cost;
}

//check if nez partition is the best one found
inline double AnalyzeNewPartition(const T_Instance &instance, const std::vector<std::vector<int>> &Partitions, std::vector<std::vector<int>> &BestPartitions, double *BestPartitionsValue)
{
  double newVal = Get_valueOfPartition(instance, Partitions);
  if (newVal > (*BestPartitionsValue))
  {
    *BestPartitionsValue = newVal;
    BestPartitions = Partitions;
  }
}

void LocalSearch_TwoMoves_Solution(const T_Instance &instance, std::vector<std::vector<int>> &Partitions)
{
  int bestNewPartition, I, J, aux_edge,
      counter = 0, max_iteration = 3,
      target_i, target_j, gama,
      Ver_i, Ver_j, finalTarget_i, finalTarget_j;
  double sumEdge_SP, sumEdge, bestSumEdge,
      val_i, val_j, bestVal_j, bestVal_i,
      Best_Delta, Delta_dij;
  vector<int> FindPartition_origem(instance.DIM + 1, -1);
  vector<int> FindPartitionPosition_origem(instance.DIM + 1, -1);
  vector<vector<double>> gain_perPartition;

  //  PrintPartitionICH(Partitions); //cin.get();
  //  cout << "Initial value = " << Get_valueOfPartition (instance,Partitions ) << endl;

  if (Partitions.size() < K)
    Partitions.resize(K);

  gain_perPartition.resize(instance.DIM);
  for (int i = 0; i < instance.DIM; ++i)
    gain_perPartition[i].resize(K);

  //vector of origem partition
  for (unsigned p = 0; p < Partitions.size(); ++p)
    for (unsigned pp = 0; pp < Partitions[p].size(); ++pp)
    {
      FindPartition_origem[Partitions[p][pp]] = p;
      FindPartitionPosition_origem[Partitions[p][pp]] = pp;
    }
  //Create gain per partition
  for (unsigned i = 0; i < instance.DIM - 1; ++i)
    for (unsigned p = 0; p < Partitions.size(); ++p)
      gain_perPartition[i][p] = gain_movePartition(instance, Partitions, i, FindPartition_origem[i], p);

  bool CHANGES = true;

  while ((CHANGES) && (counter < max_iteration))
  {
    counter++;
    CHANGES = false;
    Best_Delta = -D_INFINITY;
    Ver_i = -1;
    Ver_j = -1;
    for (unsigned i = 0; i < instance.DIM - 1; ++i)
    {
      I = i + 1;
      for (unsigned j = i + 1; j < instance.DIM; ++j)
      {
        J = j + 1;
        aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
        if (aux_edge == -1 || instance.cij.barc_v[aux_edge] == 0.0)
          continue;

        bestVal_i = -D_INFINITY;
        bestVal_j = -D_INFINITY;
        for (unsigned p = 0; p < Partitions.size(); ++p)
        {
          val_i = gain_perPartition[i][p]; //gain_movePartition(instance, Partitions, i,FindPartition_origem[i], p);
          val_j = gain_perPartition[j][p]; //gain_movePartition(instance, Partitions, j,FindPartition_origem[j], p);

          if (val_i > bestVal_i)
          {
            bestVal_i = val_i;
            target_i = p;
          }

          if (val_j > bestVal_j)
          {
            bestVal_j = val_j;
            target_j = p;
          }
        }

        gama = Set_gama(FindPartition_origem[i], target_i, FindPartition_origem[j], target_j);

        Delta_dij = bestVal_i + bestVal_j + gama * instance.cij.barc_v[aux_edge];

        if (Delta_dij > 0 && Delta_dij > Best_Delta)
        {
          Best_Delta = Delta_dij;
          Ver_i = i;
          Ver_j = j;
          finalTarget_i = target_i;
          finalTarget_j = target_j;
        }
      } //end for j
    }   //end for i

    if (Ver_i >= 0)
    {

      CHANGES = true;
      //vertex i
      Partitions[finalTarget_i].push_back(Ver_i);                                                                                           //add element ii in partition "bestNewPartition"
      Partitions[FindPartition_origem[Ver_i]].erase(Partitions[FindPartition_origem[Ver_i]].begin() + FindPartitionPosition_origem[Ver_i]); //erase ii from i partition
      //vertex j
      Partitions[finalTarget_j].push_back(Ver_j); //add element ii in partition "bestNewPartition"
      //erase (must do by long way to eras ver_j)
      for (unsigned p = 0; p < Partitions[FindPartition_origem[Ver_j]].size(); ++p)
        if (Partitions[FindPartition_origem[Ver_j]][p] == Ver_j)
        {
          Partitions[FindPartition_origem[Ver_j]].erase(Partitions[FindPartition_origem[Ver_j]].begin() + p);
          p = Partitions[FindPartition_origem[Ver_j]].size();
        }

      //actualizing the valus of partitions
      for (unsigned p = 0; p < Partitions.size(); ++p)
        for (unsigned pp = 0; pp < Partitions[p].size(); ++pp)
        {
          FindPartition_origem[Partitions[p][pp]] = p;
          FindPartitionPosition_origem[Partitions[p][pp]] = pp;
        }
      //new gain for i
      //Create gain per partition
      for (unsigned v = 0; v < instance.DIM - 1; ++v)
        for (unsigned p = 0; p < Partitions.size(); ++p)
          gain_perPartition[v][p] = gain_movePartition(instance, Partitions, v, FindPartition_origem[v], p);
    }
  } //end while
  //   PrintPartitionICH(Partitions); //cin.get();
  // 		 cout << "=======final llllll  value = " << Get_valueOfPartition (instance,Partitions ) << endl; cin.get();
} //end of function

//See operator 2 in MOH article
double Set_gama(const int &Ori_p_i, const int &target_i, const int &Ori_p_j, const int &target_j)
{
  if (Ori_p_i == Ori_p_j && target_j == target_i)
    return -2;
  if (Ori_p_i == Ori_p_j && target_j != target_i)
    return -1;
  if (Ori_p_i != Ori_p_j && target_j == target_i)
    return -1;

  if (Ori_p_i == target_j && Ori_p_j == target_i)
    return 2;
  if (Ori_p_i != target_j && Ori_p_j == target_i)
    return 1;
  if (Ori_p_i == target_j && Ori_p_j != target_i)
    return 1;

  //else
  return 0;
}

//Calcule the
double gain_movePartition(const T_Instance &instance, const std::vector<std::vector<int>> &Partitions, const int &vert, const int &Par_or, const int &Par_target)
{
  int I, J;
  long aux_edge;
  double sumEdge_SP, sumEdge;

  if (Par_or == Par_target)
    return -D_INFINITY;

  //Calcule same partition
  sumEdge_SP = 0.0;
  I = vert + 1;
  for (unsigned iii = 0; iii < Partitions[Par_or].size(); iii++)
  {
    if (Partitions[Par_or][iii] == vert)
      continue;

    J = Partitions[Par_or][iii] + 1; // start by 1

    aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
    if (aux_edge != -1)
      sumEdge_SP += instance.cij.barc_v[aux_edge];
  }

  //Calcule target
  sumEdge = 0.0;
  for (unsigned jj = 0; jj < Partitions[Par_target].size(); jj++)
  {
    J = Partitions[Par_target][jj] + 1; // start by 1
    aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
    if (aux_edge != -1)
      sumEdge += instance.cij.barc_v[aux_edge];
  } //end 	FOR ii and jj

  return sumEdge_SP - sumEdge;
}

//Improve a feasible solution
void LocalSearch_Solution(const T_Instance &instance, std::vector<std::vector<int>> &Partitions)
{
  int bestNewPartition, I, J, aux_edge,
      counter = 0, max_iteration = 20;
  double sumEdge_SP, sumEdge, bestSumEdge;
  //PrintPartitionICH(Partitions); //cin.get();
  //cout << "Initial value = " << Get_valueOfPartition (instance,Partitions ) << endl;

  if (Partitions.size() < K)
    Partitions.resize(K);

  bool CHANGES = true;

  while ((CHANGES) && (++counter < max_iteration))
  {
    CHANGES = false;
    for (unsigned i = 0; i < Partitions.size(); i++)
    {
      for (unsigned ii = 0; ii < Partitions[i].size(); ii++)
      {
        I = Partitions[i][ii] + 1; // start by 1
        //calcule if we take vertex ii from partition i
        sumEdge_SP = 0.0;
        for (unsigned iii = 0; iii < Partitions[i].size(); iii++)
        {
          if (iii == ii)
            continue;
          J = Partitions[i][iii] + 1; // start by 1

          aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
          if (aux_edge != -1)
            sumEdge_SP += instance.cij.barc_v[aux_edge];
        }
        //cout << "sumEdge_SP = " << sumEdge_SP << endl;

        { //find best partition to be put
          bestNewPartition = -1;
          bestSumEdge = D_INFINITY;

          for (unsigned j = 0; j < Partitions.size(); j++)
          {
            if (i == j)
              continue; // do nothing
            sumEdge = 0.0;
            for (unsigned jj = 0; jj < Partitions[j].size(); jj++)
            {
              J = Partitions[j][jj] + 1; // start by 1
              aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
              if (aux_edge != -1)
                sumEdge += instance.cij.barc_v[aux_edge];
            } //end 	FOR ii and jj

            if (((sumEdge < sumEdge_SP) && (sumEdge < bestSumEdge)))
            {
              //cout << "Partition " << j << ",  sumEdge_SP ="<<  sumEdge_SP<< ",  sumEdge = " << sumEdge << endl ; cin.get();
              bestSumEdge = sumEdge;
              bestNewPartition = j;
            }
          } //end for J
        }   //end find best partition

        if (bestNewPartition != -1)
        {
          Partitions[bestNewPartition].push_back(Partitions[i][ii]); //add element ii in partition "bestNewPartition"
          Partitions[i].erase(Partitions[i].begin() + ii);           //erase ii from i partition
          ii--;
          CHANGES = true;
        } //end of if Bestpartition

      } //end FOR ii
    }   //end FOR i
  }     //end of WHILE

  // //   cout << "Final counter" << counter; cin.get();
  //cout << "Finalllll value = " << Get_valueOfPartition (instance,Partitions ) << endl;
  //PrintPartitionICH(Partitions); cin.get();
} //end function

//Calcule value of given partition
inline double Get_valueOfPartition(const T_Instance &instance, const std::vector<std::vector<int>> &Partitions)
{
  int I, J, aux_edge;
  double sumEdge = 0.0;

  for (unsigned i = 0; i < Partitions.size(); i++)
    for (unsigned j = i + 1; j < Partitions.size(); j++)
      for (unsigned ii = 0; ii < Partitions[i].size(); ii++)
        for (unsigned jj = 0; jj < Partitions[j].size(); jj++)
        {
          I = Partitions[i][ii] + 1; // start by 1
          J = Partitions[j][jj] + 1; // start by 1

          aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
          if (aux_edge != -1)
            sumEdge += instance.cij.barc_v[aux_edge];
        }

  //}//end I and J

  return sumEdge;
}

//Construction of Initial feasible solution
void Initial_solution_to_GraspFeasible(const T_Instance &instance, const int &type_val, const double &RCL_prop, std::vector<std::vector<int>> &Partitions)
{
  int TYPE_COST = 0, TYPE_RELEXEDSOL = 1,
      s_Part, Counter, sumEdge, max_sizeRCL = 7;
  double ValPartition, random;
  int I, J, aux_edge;
  //srand (time(NULL));//to use the srand

  std::set<RankPartition> RCL_List;
  std::set<RankPartition>::iterator it;

  //set a vertex for patation initially
  Partitions.clear();
  for (unsigned i = 0; i < instance.DIM; i++)
  {
    std::vector<int> newPartition(1, i);
    Partitions.push_back(newPartition);
  }

  //  PrintPartitionICH(Partitions); cin.get();

  //reducing number of partitions
  while (Partitions.size() > K)
  {
    //    cout << "Partitions.size() = " << Partitions.size(); cin.get();
    for (unsigned i = 0; i < Partitions.size() && Partitions.size() > K; i++)
    {
      RCL_List.clear();
      for (unsigned j = i + 1; j < Partitions.size(); j++)
      {
        sumEdge = 0.0;

        for (unsigned ii = 0; ii < Partitions[i].size(); ii++)
          for (unsigned jj = 0; jj < Partitions[j].size(); jj++)
          {
            I = Partitions[i][ii] + 1; // start by 1
            J = Partitions[j][jj] + 1; // start by 1
            aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
            if (aux_edge != -1)
            {
              if (type_val == TYPE_COST)
              {

                sumEdge += instance.cij.barc_v[aux_edge]; //Non decreasing order
              }
              else
              {
                sumEdge -= ValueXij(I, J, instance); // minus because it should be in no decreasing order
              }
            }
            //    	  cout << "x{"<< I <<","<< J <<  "} -- cost = " << instance.cij.barc_v[aux_edge] << endl; cin.get();
          } //end 	FOR ii and jj

        RankPartition newRanked;
        newRanked.Part_origem = j;
        newRanked.sumEdge = sumEdge;

        RCL_List.insert(newRanked);
      } //end for J

      //       for (it=RCL_List.begin(); it != RCL_List.end( ) ; ++it)
      // 	cout << " partition =" <<  (*it).Part_origem << ",val = "<< (*it).sumEdge<< endl;
      //       cin.get();

      //select the biggest element (or not)
      bool select = false;
      Counter = 0;
      while ((!select) && (Counter < max_sizeRCL) && (Counter < RCL_List.size()))
      {
        //radom number btw 0.1 and 1
        random = ((rand() % 10) + 1) / 10.0;
        if (random <= RCL_prop)
        { //selected
          select = true;
          it = RCL_List.begin();

          for (int zz = 0; zz < Counter; zz++)
            ++it;
          s_Part = (*it).Part_origem;

          //join two partitions i with s_Part
          for (unsigned j = 0; j < Partitions[s_Part].size(); j++)
            Partitions[i].push_back(Partitions[s_Part][j]);

          //erase last partition
          Partitions.erase(Partitions.begin() + s_Part);
        }
        else
        {
          Counter++;
        }

      } //end of WHILE select

    } //end for I

  } //end WHILE k

  //cout << "Partition after initial construction: "<<endl;
  //    PrintPartitionICH(Partitions); cin.get();
}

double ICH_Heuristique_Ghaddar_main(const T_Instance &instance)
{
  //Execute Cutting plane using SDP
  double val_ICH;
  int OriginalSDP = SDP_SEP;
  double OriginalMAXTIME = MAXTIME;
  double OriginalMAXTIME_ITE = MAXTIME_ITE;
  T_SDP_EdgeConstraint sdpEdgeConst_origem = sdpEdgeConst;

  SDP_SEP = -1; // fOr Ghaddar it works better when executed SDP solver(SDP_SEP)
  //chaging time of iteration and total time of cutting plane (reduce )
  if (MAXTIME > 30.0)
    MAXTIME = 30.0;
  MAXTIME_ITE = 1.5; //(unespeceted ) for some instances, larger values are worse....

  T_Instance instance2 = instance;
  instance2.clear_Constraint();
  instance2.totalVars = instance.edge_nb; //(Need to do) because of Extended and other formulations (Does change anything in the Original Instance)

  CuttingPlane_Optimization(instance2, false); //Solving using SDP solver

  std::vector<int> OriginVertices2;
  val_ICH = ICH_Heuristique_Ghaddar(instance2, OriginVertices2, instance2); //Envoyer Cutting plane

  SDP_SEP = OriginalSDP;
  MAXTIME = OriginalMAXTIME;
  MAXTIME_ITE = OriginalMAXTIME_ITE;
  sdpEdgeConst = sdpEdgeConst_origem;

  //   cout << "val_ICH="<< val_ICH; cin.get();
  return val_ICH;
}

double ICH_Heuristique_Ghaddar(const T_Instance &instance, std::vector<int> &OriginVertices, const T_Instance &instanceCONST, const double &epslonICH)
{

  //printSolution_screen(instance);
  bool CONTINUE;
  int counter, bestVertex, I, H, J;
  double sumVal, bestSumVal;
  std::vector<std::vector<int>> Partitions;
  std::vector<int> vecVertex;
  std::set<ICH_Var> setICHvar;
  std::set<ICH_Var>::iterator it;

  //if initial we assign which vertex to a partition
  if (OriginVertices.size() == 0)
  {
    OriginVertices.resize(instance.DIM);
    for (unsigned i = 0; i < instance.DIM; i++)
      OriginVertices[i] = i;
  }

  vecVertex.resize(instance.DIM, -1);

  setICHvar.clear();
  //   cout << "instance.DIM = " << instance.DIM; cin.get();
  for (unsigned i = 0; i < instance.DIM - 2; i++)
    for (unsigned j = i + 1; j < instance.DIM - 1; j++)
      for (unsigned h = j + 1; h < instance.DIM; h++)
        if (!GetExceptionTriangle(instance, i, j, h))
        {
          sumVal = ValueXij(i + 1, j + 1, instance);
          sumVal += ValueXij(i + 1, h + 1, instance);
          sumVal += ValueXij(h + 1, j + 1, instance);

          // make new ich var
          ICH_Var Tijh;
          Tijh.sumEdge = sumVal;
          Tijh.vertices.push_back(i);
          Tijh.vertices.push_back(j);
          Tijh.vertices.push_back(h);

          setICHvar.insert(Tijh);
        } //end FOR I, J, H

  // select the best
  for (it = setICHvar.begin(); it != setICHvar.end(); ++it)
  {
    if ((*it).sumEdge < epslonICH)
      break; // exit for iteration

    I = (*it).vertices[0];
    J = (*it).vertices[1];
    H = (*it).vertices[2];

    //cout << ", I =" << I << ", H =" << H << ", J =" << J << ", (*it).sumEdge =" << (*it).sumEdge ; cin.get();
    //cout << ", vecVertex[I] =" << vecVertex[J] << ", vecVertex[I] =" << vecVertex[I]<< ", vecVertex[I] =" << vecVertex[H] << ", (*it).sumEdge =" << (*it).sumEdge ; cin.get();

    if (vecVertex[I] == -1 || vecVertex[J] == -1 || vecVertex[H] == -1)
    {
      if (vecVertex[I] == -1 && vecVertex[J] == -1 && vecVertex[H] == -1)
      {
        std::vector<int> newPartition;
        newPartition.push_back(I);
        newPartition.push_back(J);
        newPartition.push_back(H);
        Partitions.push_back(newPartition);
        vecVertex[I] = vecVertex[J] = vecVertex[H] = Partitions.size() - 1; //
      }
      else if (vecVertex[I] == vecVertex[J])
      {
        if (vecVertex[I] == -1)
        {
          Partitions[vecVertex[H]].push_back(I);
          Partitions[vecVertex[H]].push_back(J);
          vecVertex[I] = vecVertex[J] = vecVertex[H];
        }
        else
        {
          Partitions[vecVertex[I]].push_back(H);
          vecVertex[H] = vecVertex[I];
        }
      }
      else if (vecVertex[I] == vecVertex[H])
      {
        if (vecVertex[I] == -1)
        {
          Partitions[vecVertex[J]].push_back(I);
          Partitions[vecVertex[J]].push_back(H);
          vecVertex[I] = vecVertex[H] = vecVertex[J];
        }
        else
        {
          Partitions[vecVertex[I]].push_back(J);
          vecVertex[J] = vecVertex[I];
        }
      }
      else if (vecVertex[J] == vecVertex[H])
      {
        if (vecVertex[J] == -1)
        {
          Partitions[vecVertex[I]].push_back(J);
          Partitions[vecVertex[I]].push_back(H);
          vecVertex[J] = vecVertex[J] = vecVertex[I];
        }
        else
        {
          Partitions[vecVertex[J]].push_back(I);
          vecVertex[I] = vecVertex[J];
        }
      }
    } //end of main if
  }   //end for it

  for (unsigned i = 0; i < instance.DIM; i++)
  {
    if (vecVertex[i] == -1)
    {
      std::vector<int> newPartition;
      newPartition.push_back(i);
      Partitions.push_back(newPartition);
    }
  }

  //PrintPartitionICH(Partitions); cin.get();

  if ((Partitions.size() > K) && (Partitions.size() < instance.DIM))
  {

    Change_VecVertexOrigem_by_Partition_ICH(Partitions, OriginVertices);

    T_Instance instance2;
    instance2 = instance;

    set_newInstance_forICH(&instance2, Partitions);

    //       cout <<"cegou33"; cin.get();
    //reoptimize the new graph
    {
      clock_t start_time = clock();
      bool PRINT_ITERATIONS = false;
      MAXTIME = 10.0;
      //send to cutting plane optimization
      double val = CuttingPlane_Optimization(instance2, PRINT_ITERATIONS, start_time);
    }

    //send to ICH again
    return ICH_Heuristique_Ghaddar(instance2, OriginVertices, instanceCONST); // recuursive
  }
  else
  { // (end of recursion !!!)
    if (Partitions.size() < instance.DIM)
    { //it is the final feasible solution
      LocalSearch_Solution(instance, Partitions);
      Change_VecVertexOrigem_by_Partition_ICH(Partitions, OriginVertices);

      Change_Partition_by_VecVertexOrigem_ICH(OriginVertices, Partitions); //to apply local search
      LocalSearch_Solution(instanceCONST, Partitions);

      //PrintPartitionICH(Partitions); //cin.get();
      //cout << "ICH heuristic Ghaddar value  olha doidera = " << Get_valueOfPartition (instanceCONST,Partitions )  +  instance.sum_cost; //cin.get();

      return Get_valueOfPartition(instanceCONST, Partitions) + instance.sum_cost;
    }
    else
    {                             // it is Partitions.size() == instance.DIM and we need to increase the epslonICH
      if (epslonICH * 0.75 < 1.0) // Error the method is not accurate at all !!
        return 0.0;
      else
        return ICH_Heuristique_Ghaddar(instance, OriginVertices, instanceCONST, epslonICH * 0.75); // reduir the epslon  recuursive
    }
  }
} //end function

//function to change the partition of each vector in OriginVertices
void Change_VecVertexOrigem_by_Partition_ICH(const std::vector<std::vector<int>> &Partitions, std::vector<int> &OriginVertices)
{
  bool RESUME = true;
  //passing informations of partition to OriginVertices
  for (unsigned i = 0; i < OriginVertices.size(); i++)
  {
    RESUME = true;
    for (unsigned j = 0; j < Partitions.size() && RESUME; j++)
      for (unsigned z = 0; z < Partitions[j].size() && RESUME; z++)
      {
        if (Partitions[j][z] == OriginVertices[i])
        {
          OriginVertices[i] = j;
          RESUME = false;
        } //end IF
      }   //end for z
  }       //end FOR i

} //end of function

void Change_Partition_by_VecVertexOrigem_ICH(const std::vector<int> &OriginVertices, std::vector<std::vector<int>> &Partitions)
{
  int NbPartition = 0;

  //calculate size of partition
  for (unsigned i = 0; i < OriginVertices.size(); i++)
    if (OriginVertices[i] > NbPartition)
      NbPartition = OriginVertices[i];

  //change partition
  Partitions.clear();
  Partitions.resize(NbPartition + 1);

  for (unsigned i = 0; i < OriginVertices.size(); i++)
    Partitions[OriginVertices[i]].push_back(i);

} //end of function Change_Partition_by_VecVertexOrigem_ICH

//if epslonICH  is not declared it is 0.4 by defaut
// epslonICH  is the tolerance used to include two vertecies in the same partition
// higher epslonICH  means that it is less tolerante and can accept vertices not very close ( approx 0) to be in the same partition.
// Due to bad behavior of ICH heuristic sometimes we have to increse epslonICH to abtain a feasible solution.
double ICH_Heuristique(const T_Instance &instance, const double &epslonICH)
{

  //printSolution_screen(instance);
  bool CONTINUE;
  int counter, bestVertex;
  long aux_edge;
  double sumVal, bestSumVal;
  std::vector<std::vector<int>> Partitions;
  std::vector<bool> setVertex;

  setVertex.resize(instance.DIM, false);
  //   cout << "instance.DIM = " << instance.DIM; cin.get();
  for (unsigned i = 0; i < instance.DIM; i++)
  {
    if (setVertex[i] == true)
      continue;

    std::vector<int> newPartition;
    //insert i in new partition
    newPartition.push_back(i);
    setVertex[i] = true;
    CONTINUE = true;
    while (CONTINUE)
    {
      bestSumVal = -1.0;
      bestVertex = -1;
      for (unsigned j = i + 1; j < instance.DIM; j++)
      {
        if (setVertex[j] == true)
          continue; // if already set we are not going to verify it
        counter = 0;
        sumVal = 0.0;

        for (unsigned z = 0; z < newPartition.size(); z++)
        {
          aux_edge = instance.indice_cij[(newPartition[z] + 1) + (j + 1) * (instance.DIM + 1)];
          if (aux_edge != -1)
            sumVal += ValueXij(newPartition[z] + 1, j + 1, instance);
        }
        // 	  cout << "x{"<< i+1 <<","<< j+1 <<  "} -- sumVal = " << sumVal << endl; //cin.get();

        if (sumVal > bestSumVal)
        {
          bestVertex = j;
          bestSumVal = sumVal;
        }

      } //end FOR j

      if ((bestVertex != -1) && (bestSumVal > newPartition.size() - epslonICH))
      {
        CONTINUE = true;
        newPartition.push_back(bestVertex);
        setVertex[bestVertex] = true;
      }
      else
      {
        CONTINUE = false;
      }
    }

    Partitions.push_back(newPartition);
  } // END FOR I

  //PrintPartitionICH(Partitions); cin.get();

  if ((Partitions.size() > K) && (Partitions.size() < instance.DIM))
  {

    T_Instance instance2;
    instance2 = instance;

    set_newInstance_forICH(&instance2, Partitions);

    //reoptimize the new traph
    {
      clock_t start_time = clock();
      bool PRINT_ITERATIONS = false;
      MAXTIME = 1.0 * Partitions.size(); // seconds to optimize
      if (MAXTIME > 20.0)
        MAXTIME = 20.0;
      double val = CuttingPlane_Optimization(instance2, PRINT_ITERATIONS, start_time);
    }

    //send to ICH again
    ICH_Heuristique(instance2); // recuursive
  }
  else
  { // if
    if (Partitions.size() < instance.DIM)
    { //it is the final feasible solution
      cout << "ICH heuristic value = " << Get_valueOfPartition(instance, Partitions) + instance.sum_cost;
      //PrintPartitionICH(Partitions); //cin.get();
      return Get_valueOfPartition(instance, Partitions) + instance.sum_cost;
    }
    else
    {                                              // it is Partitions.size() == instance.DIM and we need to increase the epslonICH
      ICH_Heuristique(instance, epslonICH * 1.25); // recuursive
    }
  }
}

inline void PrintPartitionICH(const std::vector<std::vector<int>> &Partitions)
{

  cout << "Partitions.size()" << Partitions.size() << endl
       << "Printing partions .." << endl;

  for (int i = 0; i < Partitions.size(); i++)
  {
    cout << i + 1 << ": ";
    for (int j = 0; j < Partitions[i].size(); j++)
      cout << Partitions[i][j] << " , ";
    cout << endl;
  }
}
void set_newInstance_forICH(T_Instance *instance, std::vector<std::vector<int>> Partitions)
{
  int aux_edge, index = 0, I, J;
  double sumEdge, summALl = 0.0;
  T_cost_Matrix costMatrixBefore;
  std::vector<int> indice_cijBefore; //indice of where we can find the cost

  costMatrixBefore = (*instance).cij;
  indice_cijBefore = (*instance).indice_cij;
  int DIMbefore = (*instance).DIM;

  //clear instance
  ClearInstance((*instance));

  (*instance).DIM = Partitions.size();
  (*instance).edge_nb = (((*instance).DIM - 1) * ((*instance).DIM)) / 2;
  //Allocate memory for cost
  (*instance).indice_cij.resize(((*instance).DIM + 1) * ((*instance).DIM + 1), -1);
  (*instance).ant_indice_cij.resize((((*instance).DIM + 1) * ((*instance).DIM)) / 2); // variables i and j of indice_cij

  //remeber that edges start with 1 not 0 !!!
  for (unsigned i = 0; i < Partitions.size() - 1; i++)
    for (unsigned j = i + 1; j < Partitions.size(); j++)
    {
      sumEdge = 0.0;

      for (unsigned ii = 0; ii < Partitions[i].size(); ii++)
        for (unsigned jj = 0; jj < Partitions[j].size(); jj++)
        {
          I = Partitions[i][ii] + 1; // start by 1
          J = Partitions[j][jj] + 1; // start by 1

          aux_edge = indice_cijBefore[I + J * (DIMbefore + 1)];
          sumEdge += costMatrixBefore.barc_v[aux_edge];

          // 	  cout << "x{"<< I <<","<< J <<  "} -- cost = " << costMatrixBefore.barc_v[aux_edge] << endl; cin.get();
        } //end 	FOR ii and jj

      summALl += sumEdge;
      I = i + 1; // start by 1
      J = j + 1; // start by 1

      //set new values
      (*instance).indice_cij[(I) + (J) * ((*instance).DIM + 1)] = index;
      (*instance).indice_cij[(J) + (I) * ((*instance).DIM + 1)] = index;

      (*instance).ant_indice_cij[index].idx_i = J; // start from 1
      (*instance).ant_indice_cij[index].idx_j = I; // in SDP idx j should be the smaller one
      index++;

      (*instance).cij.barc_j.push_back(I);
      (*instance).cij.barc_i.push_back(J);
      (*instance).cij.barc_v.push_back(sumEdge); // it is the edge weight

    } //end FOR i and j

  //   ISCOMPLETE_GRAPH = true;

  (*instance).totalVars = (*instance).edge_nb;

  //cout << "end of set_newInstance_forICH summALl = " << summALl << endl; cin.get();
}

inline void Set_REDUNDANT_Ineq_of_FuncObj(T_Instance &instance, MSKrescodee &r, MSKtask_t &task)
{
  //creating a constraint
  T_constraint Const;
  int I, J, i, counter;
  double aux_edge;
  int *asub;
  double *aval;

  double sumTotal = 0;
  for (int j = 0; j < instance.edge_nb; ++j)
    sumTotal += instance.cij.barc_v[j];

  Const.Origem = 77;               //triangle
  Const.bkc = MSK_BK_UP;           // Bound key (<=)
  Const.blc = -MSK_INFINITY;       // Numerical value of Lowwer bound
  Const.buc = MAXVALUE - sumTotal; // Numerical value of Upper bound

  //resize by number of edge in a complete graph = n*(n-1)/2
  Const.vars.resize(((instance.DIM) * (instance.DIM - 1)) / 2); // fix
  Const.aval.resize(((instance.DIM) * (instance.DIM - 1)) / 2); // fix

  //creating instance
  counter = 0;
  // making sum(u,v_i)
  for (int i = 0; i < instance.DIM; i++)
    for (int j = i + 1; j < instance.DIM; j++)
    {
      I = i + 1;
      J = j + 1;
      //putting in xpr
      aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
      if (aux_edge != -1)
      {
        Const.vars[counter] = (int)aux_edge;
        Const.aval[counter] = -1.0 * instance.cij.barc_v[aux_edge]; //
        counter++;
      }
    } //end j

  //Inserting in mosek
  i = instance.CONST.size();
  {
    r = MSK_putconbound(task,
                        i,          // Index of constraint.
                        Const.bkc,  // Bound key.
                        Const.blc,  // Numerical value of lower bound.
                        Const.buc); // Numerical value of upper bound.
    // Input row i of A
    asub = &Const.vars[0];
    aval = &Const.aval[0];

    if (r == MSK_RES_OK)
      r = MSK_putarow(task,
                      i,                 // Row index.
                      Const.vars.size(), // Number of non-zeros in row i.
                      asub,              // Pointer to column indexes of row i.
                      aval);             // Pointer to values of row i.
  }
}

inline void Add_NewConstraintLP_in_taskMosek(T_Instance &instance, MSKrescodee &r, MSKtask_t &task)
{
  int *asub;
  double *aval;
  int conidx;

  /* Get index of new constraint*/
  if (r == MSK_RES_OK)
    r = MSK_getnumcon(task, &conidx);
  /*
	if (r != MSK_RES_OK){
	cout << "conidx = " << conidx;
	cout << "instance.CONST.size()" << instance.CONST.size();
	}*/
  //iterate all fixed variables
  for (; conidx < instance.CONST.size(); conidx++)
  {
    /* Append a new constraint */
    if (r == MSK_RES_OK)
      r = MSK_appendcons(task, 1);

    r = MSK_putconbound(task,
                        conidx,                      // Index of constraint.
                        instance.CONST[conidx].bkc,  // Bound key.
                        instance.CONST[conidx].blc,  // Numerical value of lower bound.
                        instance.CONST[conidx].buc); // Numerical value of upper bound.
    // Input row i of A
    asub = &instance.CONST[conidx].vars[0];
    aval = &instance.CONST[conidx].aval[0];

    if (r == MSK_RES_OK)
      r = MSK_putarow(task,
                      conidx,                             // Row index.
                      instance.CONST[conidx].vars.size(), // Number of non-zeros in row i.
                      asub,                               // Pointer to column indexes of row i.
                      aval);                              // Pointer to values of row i.

  } // end FOR conidx

  //     if ( r!=MSK_RES_OK ){
  // 	    cout << "Erro " << NEW_TASK_LP; cin.get();
  // 	  }
}

inline void setConstraint_LPmosek(T_Instance &instance, MSKrescodee &r, MSKtask_t &task, int start)
{
  int *asub;
  double *aval;

  //   SIMPLEXITE = 0;
  // int LastConstraint = 0;
  //    SIMPLEXITE = 1; // change to never do that again in simplex mode

  // Append 'numcon' empty constraints.
  //The constraints will initially have no bounds.
  if (start == 0)
  { // not a simplex type
    if (r == MSK_RES_OK)
      r = MSK_appendcons(task, (MSKint32t)instance.CONST.size());
  }
  else
  {
    /* Append a new constraint */
    if (r == MSK_RES_OK)
      r = MSK_appendcons(task, (MSKint32t)(instance.CONST.size() - start));
  }

  for (int i = start; i < (int)instance.CONST.size() && r == MSK_RES_OK; ++i)
  {

    r = MSK_putconbound(task,
                        i,                      // Index of constraint.
                        instance.CONST[i].bkc,  // Bound key.
                        instance.CONST[i].blc,  // Numerical value of lower bound.
                        instance.CONST[i].buc); // Numerical value of upper bound.
    // Input row i of A
    asub = &instance.CONST[i].vars[0];
    aval = &instance.CONST[i].aval[0];

    if (r == MSK_RES_OK)
      r = MSK_putarow(task,
                      i,                             // Row index.
                      instance.CONST[i].vars.size(), // Number of non-zeros in row i.
                      asub,                          // Pointer to column indexes of row i.
                      aval);                         // Pointer to values of row i.
  }
}

inline void setObjFunction_andVarBounds__LPmosek(const T_Instance &instance, MSKrescodee &r, MSKtask_t &task, MSKenv_t &env, int Form_Type)
{

  //Max Sum(ij) (wij*(1-xij)) ===>  Sum(ij) (wij) - Sum(ij)(xij)

  double sumTotal = 0;
  if (Form_Type == 0)
    for (int j = 0; j < instance.edge_nb && r == MSK_RES_OK; ++j)
      sumTotal += instance.cij.barc_v[j];

  // add a constant term to the objective.
  if (r == MSK_RES_OK)
    r = MSK_putcfix(task, sumTotal + instance.sum_cost);

  //Check the type
  if (Form_Type == 0)
  { //Edge only formulation
    for (int j = 0; j < instance.totalVars && r == MSK_RES_OK; ++j)
    {
      // Set the linear term c_j in the objective.
      r = MSK_putcj(task, j, -1.0 * instance.cij.barc_v[j]);

      // Set the bounds on variable j.
      //  blx[j] <= x_j <= bux[j]
      if (r == MSK_RES_OK)
        r = MSK_putvarbound(task,
                            (MSKint32t)j, // Index of variable.
                            MSK_BK_RA,    // Bound key.
                            0.0,          // Numerical value of lower bound.
                            1.0);         // Numerical value of upper bound.
    }                                     //end FOR
  }
  else
  { // new  formulation

    for (int j = 0; j < instance.totalVars && r == MSK_RES_OK; ++j)
    {
      // Set the linear term c_j in the objective.
      r = MSK_putcj(task, j, +1.0 * instance.cij.barc_v[j]);

      // Set the bounds on variable j.
      //  blx[j] <= x_j <= bux[j]
      if (r == MSK_RES_OK)
      { //edge only formulation
        if (j < instance.DIM * K)
        { // Xip variables
          r = MSK_putvarbound(task,
                              (MSKint32t)j, // Index of variable.
                              MSK_BK_RA,    // Bound key.
                              0.0,          // Numerical value of lower bound.
                              1.0);         // Numerical value of upper bound.
          if (Form_Type == 2)
            r = MSK_putvartype(task, (MSKint32t)j, MSK_VAR_TYPE_INT); //integer
        }
        else
        {
          r = MSK_putvarbound(task,
                              (MSKint32t)j,   // Index of variable.
                              MSK_BK_LO,      // Bound key.
                              0.0,            // Numerical value of lower bound.
                              +MSK_INFINITY); // Numerical value of upper bound.
        }
      }
    } //end FOR
  }   //end ELSE new formulation
}

template <typename T>
inline void printvector(const std::vector<T> &v)
{
  cout << "Print vector, of size = " << v.size() << " : {";
  for (std::size_t j = 0; j < v.size(); j++)
  {
    cout << v[j];
    if (j < v.size() - 1)
      cout << ", ";
  }
  cout << "};" << endl;
}

inline void PrintSDPtypeSolution(const T_Instance &instance)
{
  //     std::cout << std::setprecision(2);
  for (unsigned i = 0; i < instance.DIM; i++)
  {
    for (unsigned j = 0; j < instance.DIM; j++)
    {
      if (i == j)
        cout << "1, ";
      else
        cout << ValueXij(i + 1, j + 1, instance) << ", ";
    }
    cout << endl;
  }
}

template <typename T>
inline void printDoubleVector(const std::vector<std::vector<T>> &v)
{
  cout << "Print vector, of size = " << v.size() << " :" << endl;
  for (std::size_t i = 0; i < v.size(); ++i)
  {
    cout << i << "  : {";
    for (std::size_t j = 0; j < v[i].size(); ++j)
    {
      cout << v[i][j] << ", ";
    }
    cout << "};" << endl;
  }
}

template <typename T>
inline void printMatrix(const std::vector<T> &v, const int &dim)
{
  cout << "Print vector, of size = " << v.size() << " :" << endl;
  for (std::size_t j = 0; j < v.size(); j++)
  {
    cout << v[j] << ", ";
    if (((j + 1) % dim == 0) && (j != 0))
      cout << "| " << endl;
  }
  cout << "| ;" << endl;
}

inline void noCompleteGraph(T_Instance &instance)
{
  int index = instance.edge_nb - 1; // remember that we start at 0;
  //actulizing the number of edges
  instance.edge_nb = (instance.DIM * (instance.DIM - 1)) / 2;

  //remeber that edges start with 1 not 0 !!!
  for (int i = 1; i < instance.DIM + 1; i++)
    for (int j = i + 1; j < instance.DIM + 1; j++)
    {
      index = instance.indice_cij[i + (j) * (instance.DIM + 1)];
      if (instance.cij.barc_v[index] == 0.0)
      { //there is no edge (thus we need to add one with cost of 0)

        instance.indice_cij[i + (j) * (instance.DIM + 1)] = -1;
        instance.indice_cij[j + (i) * (instance.DIM + 1)] = -1;

        instance.edge_nb--;

        ISCOMPLETE_GRAPH = false;
      }
    } //end FOR

  instance.totalVars = instance.edge_nb;
}

void set_newFormulation(T_Instance &instance)
{

  int index = 0; // remember that we start at 0;
  T_cost_Matrix costMatrix;
  std::vector<int> ind_cij;
  std::vector<double> posM;
  std::vector<double> negM;

  costMatrix = instance.cij;
  ind_cij = instance.indice_cij;

  calcule_IncidenteEdges(instance, &posM, &negM, ind_cij, costMatrix);

  ClearInstance(instance);
  instance.sum_cost = 0.0;

  //Allocate memory in
  instance.indice_cij.resize((instance.DIM + 1) * K + K + 1);      //+1 because we start from 1 not 0
  instance.ant_indice_cij.resize(instance.edge_nb + instance.DIM); // variables i and j of indice_cij

  // Set objective cost = 0 to all the representative variables
  //remeber that edges start with 1 not 0 !!!
  for (int p = 1; p < K + 1; p++)
  {
    for (int i = 1; i < instance.DIM + 1; i++)
    {
      instance.indice_cij[index] = index;
      instance.ant_indice_cij[index].idx_i = i; //
      instance.ant_indice_cij[index].idx_j = p; // in SDP idx j should be the smaller one
      index++;

      instance.cij.barc_j.push_back(p);
      instance.cij.barc_i.push_back(i);
      instance.cij.barc_v.push_back(0.0); // it is the edge weight
    }                                     //end of P
  }                                       //end FOR

  // Set objective cost = 0 to all the representative variables
  //remeber that edges start with 1 not 0 !!!
  for (int p = 1; p < K + 1; p++)
  {
    instance.indice_cij[index] = index;
    instance.ant_indice_cij[index].idx_i = p; //
    instance.ant_indice_cij[index].idx_j = p; // in SDP idx j should be the smaller one
    index++;

    instance.cij.barc_j.push_back(p);
    instance.cij.barc_i.push_back(p);
    instance.cij.barc_v.push_back(1.0); // it is the variable weight
  }                                     //end of P

  instance.totalVars = index;

  Design_NewFormulation_bigIneq(&instance, costMatrix, ind_cij, negM);
  //
  Design_NewFormulation_upperboundpartition(&instance, posM);

  Design_NewFormulation_sumEqualOne(&instance);

  cout << "End of set_newFormulation ";
  cin.get();
}

//Design of inequality for new formulation
// sum_p (x_ip) = 1.0
// it will assure each vertex are in one partition
void Design_NewFormulation_sumEqualOne(T_Instance *instance)
{
  int aux_edge, Counter;
  double qt_var = K;
  double Rhcst = 1.0;

  for (unsigned i = 0; i < (*instance).DIM; i++)
  {

    T_constraint Const;

    Const.Origem = 54;     //Clique and General clique
    Const.bkc = MSK_BK_FX; // Bound key (==)
    Const.blc = Rhcst;     // Numerical value of Lowwer bound
    Const.buc = Rhcst;     // Numerical value of Upper bounds

    //resize to vec_vertices.size() variables

    Const.vars.resize(qt_var); // x^_ij - x_{i,j}
    Const.aval.resize(qt_var); // fix

    Counter = 0;
    for (unsigned p = 0; p < K; p++)
    {

      aux_edge = (*instance).indice_cij[i + p * ((*instance).DIM)];
      //       cout << ", i = "<<(*instance).cij.barc_i[aux_edge]  << ", p = "<<(*instance).cij.barc_j[aux_edge] << endl ; cin.get();
      Const.vars[Counter] = (int)aux_edge;
      Const.aval[Counter] = 1.0; // value
      Counter++;

    } //end P

    (*instance).CONST.push_back(Const);

  } //end FOR i

  //cout << "Chegou aqui meu brother ";
  for (unsigned i = 0; i < 3; i++)
    print_Inequality_Tconstraint(instance->CONST[i]);
  cin.get();
} //end functon

//Design of inequality for new formulation
// y_p <= sum_i (x_ip*M_i^+)
// where M_i^+ is the sum of all the positive incidente edges in vertex i
void Design_NewFormulation_upperboundpartition(T_Instance *instance, const std::vector<double> &posM)
{
  int aux_edge, aux_edge_cij, Counter;
  double qt_var = (*instance).DIM + 1;

  for (unsigned p = 0; p < K; p++)
  {
    T_constraint Const;

    Const.Origem = 51;         //Clique and General clique
    Const.bkc = MSK_BK_UP;     // Bound key (<=)
    Const.blc = -MSK_INFINITY; // Numerical value of Lowwer bound
    Const.buc = 0.0;           // Numerical value of Upper bound

    //resize to vec_vertices.size() variables

    Const.vars.resize(qt_var); // x^_ij - x_{i,j}
    Const.aval.resize(qt_var); // fix

    Counter = 0;
    for (unsigned i = 0; i < (*instance).DIM; i++)
    {
      aux_edge = (*instance).indice_cij[i + p * ((*instance).DIM)];
      //       cout << ", i = "<<(*instance).cij.barc_i[aux_edge]  << ", p = "<<(*instance).cij.barc_j[aux_edge] << endl ; cin.get();
      Const.vars[Counter] = (int)aux_edge;
      Const.aval[Counter] = -1.0 * posM[i]; // change it for |positiv|
      Counter++;
    }

    //Include y_p
    aux_edge = (*instance).indice_cij[K * ((*instance).DIM) + p];
    Const.vars[Counter] = (int)aux_edge;
    Const.aval[Counter] = 1.0;

    //   	cout << ", p = "<<(*instance).cij.barc_i[aux_edge]  << ", p = "<<(*instance).cij.barc_j[aux_edge] << endl ; cin.get();

    (*instance).CONST.push_back(Const);
  } //end FOR p

  //   //cout << "Chegou aqui meu brother ";
  //   for (unsigned i=0; i < K; i++)
  //   print_Inequality_Tconstraint((*instance).CONST[i]); cin.get();
}

//This function design inequality that limit sum of each partition to edge of vertices that are in dif. paratitions
// y_p <= sum_i[ sum_j (aij(1 - x_{j,p})) + M^-_i(1 - x_{i,p})]
void Design_NewFormulation_bigIneq(T_Instance *instance, const T_cost_Matrix &costMatrix, const std::vector<int> &ind_cij, const std::vector<double> &negM)
{
  int I, J, P, aux_edge, aux_edge_cij, Counter;
  int qt_var = (*instance).DIM * (*instance).DIM + (*instance).DIM;
  double val_aij, Rhcst = 0.0;
  //   int SizeCostMat = (instance.DIM)*(instance.DIM+1);
  //creating a constraint

  for (unsigned p = 0; p < K; p++)
  {
    Rhcst = 0.0;
    P = p + 1;
    T_constraint Const;

    Const.Origem = 51;         //Clique and General clique
    Const.bkc = MSK_BK_UP;     // Bound key (<=)
    Const.blc = -MSK_INFINITY; // Numerical value of Lowwer bound
    Const.buc = 0.0;           // Numerical value of Upper bound (will change in the future)

    //resize to vec_vertices.size() variables

    Const.vars.resize(qt_var); // x^_ij - x_{i,j}
    Const.aval.resize(qt_var); // fix

    Counter = 0;
    for (unsigned i = 0; i < (*instance).DIM - 1; i++)
    {
      I = i + 1;
      aux_edge = (*instance).indice_cij[i + p * ((*instance).DIM)];

      Const.vars[Counter] = (int)aux_edge;
      Const.aval[Counter] = -1.0 * negM[i]; // change it for |negatives|
      Rhcst += -1.0 * negM[i];

      Counter++;

      //       cout << ", i = "<< (*instance).cij.barc_i[aux_edge]  << ", p = "<< (*instance).cij.barc_j[aux_edge] << "negM[i]= " << negM[i]; //cin.get();

      for (unsigned j = i + 1; j < (*instance).DIM; j++)
      {

        //setting Extended variable
        J = j + 1; // start with 1
        aux_edge_cij = ind_cij[I + (J) * ((*instance).DIM + 1)];
        aux_edge = (*instance).indice_cij[j + p * ((*instance).DIM)];
        val_aij = costMatrix.barc_v[aux_edge_cij];
        Rhcst += val_aij;

        // 	cout << ", j = "<<(*instance).cij.barc_i[aux_edge]  << ", p = "<<(*instance).cij.barc_j[aux_edge] << ", val_aij = " << val_aij ; // cin.get();

        Const.vars[Counter] = (int)aux_edge;
        Const.aval[Counter] = val_aij;
        Counter++;

      } // END FOR j
    }   //end FOR i
    //print_Inequality_Tconstraint(Const); cin.get();

    //Include y_p
    aux_edge = (*instance).indice_cij[K * ((*instance).DIM) + p];
    Const.vars[Counter] = (int)aux_edge;
    Const.aval[Counter] = 1.0;

    cout << ", p= " << (*instance).cij.barc_i[aux_edge] << ", p = " << (*instance).cij.barc_j[aux_edge] << "aux_edge = " << aux_edge << endl;
    cin.get();

    //inserting constraint in instance
    Const.vars.resize(Counter + 1); // x^_ij - x_{i,j}
    Const.aval.resize(Counter + 1); // fix

    Const.buc = Rhcst; // Numerical value of Upper bounds

    (*instance).CONST.push_back(Const);

  } //

  //cout << "Chegou aqui meu brother ";
  //   for (unsigned i=0; i < 3; i++)
  //   print_Inequality_Tconstraint(instance->CONST[i]);
  //   cin.get();
}

inline double calcule_IncidenteEdges(const T_Instance &instance, std::vector<double> *posM, std::vector<double> *negM, const std::vector<int> &ind_cij, const T_cost_Matrix &costMatrix)
{
  int I, J, aux_edge_cij;
  double val;
  (*posM).resize(instance.DIM);
  (*negM).resize(instance.DIM);

  for (unsigned i = 0; i < instance.DIM - 1; i++)
  {
    I = i + 1;
    (*posM)[i] == 0;
    (*negM)[i] == 0;
    for (unsigned j = i + 1; j < instance.DIM; j++)
      if (i != j)
      {
        J = j + 1;
        aux_edge_cij = ind_cij[I + (J) * (instance.DIM + 1)];
        val = costMatrix.barc_v[aux_edge_cij];

        //       cout << "I = "<< I << ",J =" << J << ", val =" << val; cin.get();

        if ((val > 0))
        {
          (*posM)[i] += val;
        }
        else if (val < 0)
          (*negM)[i] += val;
      } //end FOR j

  } //end FOR i

  printvector(*negM);
  printvector(*posM);
  cin.get();

} //end of function calcule

//THis is the node and edge formulation
// Node variables are of type y_{vp} for vertex v and partition p
// moreover, we add the edge variables that are of type x_{ij} for edge ij
// both variables are integer {0,1}
//See formulation in my Thesis 2.5 - 2.11 (In the literature review)
inline void set_NodeEdge_Formulation(T_Instance &instance)
{
  int index = instance.totalVars; // remember that we start at 0;
  int SizeCostMat = (instance.DIM) * (instance.DIM + 1);

  //   cout << "instance.DIM = " << instance.DIM; cin.get();
  //   printMatrix(instance.indice_cij, instance.DIM+1); cin.get();

  //Allocate memory in
  instance.indice_cij.resize((instance.DIM + 1) * (instance.DIM + 1) + (instance.DIM + 1) * K + 1, -1); //+1 because we start from 1 not 0
  instance.ant_indice_cij.resize(instance.edge_nb + (instance.DIM + 1) * K + 1);                        // variables i and j of indice_cij

  //Variable y_vp (node) ... it wil assume the value 1 if vertex v is in partition p, 0 otherwise
  // Set objective cost = 0 to all the node variables
  //remeber that edges start with 1 not 0 !!!
  for (int p = 1; p < K + 1; p++)
    for (int i = 1; i < instance.DIM + 1; i++)
    {
      instance.indice_cij[SizeCostMat + i + (p) * (instance.DIM + 1)] = index;

      index++;

      instance.cij.barc_j.push_back(i);
      instance.cij.barc_i.push_back(i);
      instance.cij.barc_v.push_back(0.0); // it is the edge weight

    } //end FOR

  instance.totalVars = index;
  //     printMatrix(instance.indice_cij, instance.DIM+1); cin.get();

  /*Design of extra constraints*/
  Design_SumNodes_equal_One(instance);

  Design_Impose_EdgeEqualOne(instance); //Ineq 2.7 in thesis

  Design_Impose_EdgeCut_if_DifferentPartition(instance); //Ineq 2.8 and 2.9 together in thesis

} //end of node and edge formulations function

// This function will insert sum_{p=1 to k} = 1; into the node_and_edge formulation
inline void Design_SumNodes_equal_One(T_Instance &instance)
{
  int P, V, aux_edge, counter;
  double Rhcst = 1.0;
  int StartNodeVars = (instance.DIM) * (instance.DIM + 1);

  for (unsigned v = 0; v < instance.DIM; v++)
  {
    counter = 0;

    T_constraint Const;

    Const.Origem = 41;     //Clique and General clique
    Const.bkc = MSK_BK_FX; // Bound key (=)
    Const.blc = Rhcst;     // Numerical value of Lowwer bound
    Const.buc = Rhcst;     // Numerical value of Upper bound

    //resize to vec_vertices.size() variables
    Const.vars.resize(K); // r + sum until j
    Const.aval.resize(K); // fix

    //setting the sum of inequality
    for (unsigned p = 0; p < K; p++)
    {
      P = p + 1;
      V = v + 1; // start with 1
      aux_edge = instance.indice_cij[StartNodeVars + V + P * (instance.DIM + 1)];
      Const.vars[counter] = (int)aux_edge;
      Const.aval[counter] = +1.0;
      counter++;
    }

    //     print_Inequality_Tconstraint(Const); cin.get();

    //inserting constraint in instance
    instance.CONST.push_back(Const);
  }
}

//Design ineqs 2.8 and 2.9 of Thesis
//			x_{i,j} + y_{ip} - y_{jp}
// we do both at same time
void Design_Impose_EdgeCut_if_DifferentPartition(T_Instance &instance) //Ineq 2.8 and 2.9 together in thesis
{
  int P, V, I, J, aux_edge, counter;
  double Rhcst = 1.0;
  int StartNodeVars = (instance.DIM) * (instance.DIM + 1);

  for (unsigned j = 0; j < instance.DIM; j++)
    for (unsigned i = 0; i < instance.DIM; i++)
      if (ExistEdge(instance, i, j) && i != j)
      {
        for (unsigned p = 0; p < K; p++)
        {

          T_constraint Const;

          Const.Origem = 42;         //Clique and General clique
          Const.bkc = MSK_BK_UP;     // Bound key (<=)
          Const.blc = -MSK_INFINITY; // Numerical value of Lowwer bound
          Const.buc = Rhcst;         // Numerical value of Upper bound

          //resize to vec_vertices.size() variables
          Const.vars.resize(3); // r + x_ij
          Const.aval.resize(3); // fix

          //setting edge variable (original variable)
          I = i + 1;
          J = j + 1; // start with 1
          aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
          Const.vars[0] = (int)aux_edge;
          Const.aval[0] = +1.0;

          //First node variable
          P = p + 1;
          V = i + 1; // start with 1
          aux_edge = instance.indice_cij[StartNodeVars + V + P * (instance.DIM + 1)];
          Const.vars[1] = (int)aux_edge;
          if (i < j)
            Const.aval[1] = +1.0;
          else
            Const.aval[1] = -1.0;
          counter++;

          //second node varirable
          P = p + 1;
          V = j + 1; // start with 1
          aux_edge = instance.indice_cij[StartNodeVars + V + P * (instance.DIM + 1)];
          Const.vars[2] = (int)aux_edge;
          if (i < j)
            Const.aval[2] = -1.0;
          else
            Const.aval[2] = +1.0;
          counter++;

          // 	cout << "j=" << j << " i=" << i << " ";
          // 	print_Inequality_Tconstraint(Const); cin.get();

          //inserting constraint in instance
          instance.CONST.push_back(Const);
        }
      }

  //cout << "Chegou aqui meu brother ";
  //for (unsigned i=0; i < 3; i++)
  //print_Inequality_Tconstraint(instance.CONST[i]);
  //cin.get();
}

//Deisign inequ 2.7 of thesis
// we impose that Edge (ij) of graph to be 1 if nodes i and j are in the same partition
void Design_Impose_EdgeEqualOne(T_Instance &instance) //Ineq 2.7 in thesis
{
  int P, V, I, J, aux_edge, counter;
  double Rhcst = 1.0;
  int StartNodeVars = (instance.DIM) * (instance.DIM + 1);

  for (unsigned j = 0; j < instance.DIM; ++j)
    for (unsigned i = j + 1; i < instance.DIM; ++i)
      if (ExistEdge(instance, i, j) && i != j)
      {
        for (unsigned p = 0; p < K; p++)
        {

          T_constraint Const;

          Const.Origem = 43;         //Clique and General clique
          Const.bkc = MSK_BK_UP;     // Bound key (<=)
          Const.blc = -MSK_INFINITY; // Numerical value of Lowwer bound
          Const.buc = Rhcst;         // Numerical value of Upper bound

          //resize to vec_vertices.size() variables
          Const.vars.resize(3); // r + x_ij
          Const.aval.resize(3); // fix

          //setting edge variable (original variable)
          I = i + 1;
          J = j + 1; // start with 1
          aux_edge = instance.indice_cij[I + J * (instance.DIM + 1)];
          Const.vars[0] = (int)aux_edge;
          Const.aval[0] = -1.0;

          //First node variable
          P = p + 1;
          V = i + 1; // start with 1
          aux_edge = instance.indice_cij[StartNodeVars + V + P * (instance.DIM + 1)];
          Const.vars[1] = (int)aux_edge;
          Const.aval[1] = +1.0;
          counter++;

          //second node varirable
          P = p + 1;
          V = j + 1; // start with 1
          aux_edge = instance.indice_cij[StartNodeVars + V + P * (instance.DIM + 1)];
          Const.vars[2] = (int)aux_edge;
          Const.aval[2] = +1.0;
          counter++;

          // 	print_Inequality_Tconstraint(Const); cin.get();

          //inserting constraint in instance
          instance.CONST.push_back(Const);
        }
      }

  //cout << "Chegou aqui meu brother ";
  //for (unsigned i=0; i < 3; i++)
  //print_Inequality_Tconstraint(instance.CONST[i]);
  //cin.get();
}

inline void Set_ExtendedRepresentativeVar(T_Instance &instance)
{
  int index = instance.totalVars; // remember that we start at 0;
  int SizeCostMat = (instance.DIM) * (instance.DIM + 1);

  //   cout << "instance.DIM = " << instance.DIM; cin.get();
  //   printMatrix(instance.indice_cij, instance.DIM+1); cin.get();

  //Allocate memory in
  instance.indice_cij.resize(2 * (instance.DIM + 1) * (instance.DIM + 1) + instance.DIM + 1, -1); //+1 because we start from 1 not 0
  instance.ant_indice_cij.resize(2 * instance.edge_nb + instance.DIM + 1);                        // variables i and j of indice_cij

  //Extended variables
  // Set objective cost = 0 to all the extended variables
  //remeber that edges start with 1 not 0 !!!
  for (int i = 1; i < instance.DIM + 1; i++)
    for (int j = i + 1; j < instance.DIM + 1; j++)
      if (ExistEdge(instance, i - 1, j - 1))
      {
        instance.indice_cij[SizeCostMat + i + (j) * (instance.DIM + 1)] = index;
        instance.indice_cij[SizeCostMat + j + (i) * (instance.DIM + 1)] = index;

        instance.ant_indice_cij[index].idx_i = j; // start from 1
        instance.ant_indice_cij[index].idx_j = i; // in SDP idx j should be the smaller one
        index++;

        instance.cij.barc_j.push_back(i);
        instance.cij.barc_i.push_back(j);
        instance.cij.barc_v.push_back(0.0); // it is the edge weight

      } //end FOR

  //Representative variables
  // Set objective cost = 0 to all the representative variables
  //remeber that edges start with 1 not 0 !!!
  for (int j = 1; j < instance.DIM + 1; j++)
  {

    instance.indice_cij[2 * SizeCostMat + instance.DIM + 1 + j] = index;
    instance.ant_indice_cij[index].idx_i = j; //
    instance.ant_indice_cij[index].idx_j = j; // in SDP idx j should be the smaller one
    index++;

    instance.cij.barc_j.push_back(j);
    instance.cij.barc_i.push_back(j);
    instance.cij.barc_v.push_back(0.0); // it is the edge weight

  } //end FOR

  instance.totalVars = index;
  //     printMatrix(instance.indice_cij, instance.DIM+1); cin.get();

  //Design inequalities for extended formulation
  Design_ExtendLimitEdge(instance);

  //     cout << "Cbegou.." << cin.get();
  Design_ExtendLimitRepresentative(instance);

  Design_ExtendWithTwoVars(instance);

  Design_ExtendEqualIneq(instance); //impose k partitions ??

  //limit in sum of representative (It is just one ineq)
  //     Design_RepresentativeIneq_SumRepresentative(instance, 2*instance.edge_nb);

} //End of extended constraints and variables insertion

///stoped here !!!
//That inequality is based on  paper: AN extended edge-representative formulation for the k-partition problem , by Z. Ales, A. Knippel (see eq (9) - (13))  )
inline void Set_RepresentativeVar(T_Instance &instance)
{
  int index = instance.edge_nb; // remember that we start at 0;

  //Allocate memory in
  instance.indice_cij.resize((instance.DIM + 1) * (instance.DIM + 1) + instance.DIM + 1); //+1 because we start from 1 not 0
  instance.ant_indice_cij.resize(instance.edge_nb + instance.DIM);                        // variables i and j of indice_cij

  // Set objective cost = 0 to all the representative variables
  //remeber that edges start with 1 not 0 !!!
  for (int j = 1; j < instance.DIM + 1; j++)
  {

    instance.indice_cij[(instance.DIM + 1) * (instance.DIM + 1) + j] = index;
    instance.ant_indice_cij[index].idx_i = j; //
    instance.ant_indice_cij[index].idx_j = j; // in SDP idx j should be the smaller one
    index++;

    instance.cij.barc_j.push_back(j);
    instance.cij.barc_i.push_back(j);
    instance.cij.barc_v.push_back(0.0); // it is the edge weight

  } //end FOR

  instance.totalVars = instance.edge_nb + instance.DIM;

  //At least one presentatvie per Cluster (we have instance.DIM inequalities)
  Design_Representative_OnePerCluster(instance);

  //At most one representative per cluster (here, we have instance.nb_edge inequalities)
  Design_RepresentativeIneq_MaxOnePerCluster(instance);

  //limit in sum of representative (It is just one ineq)
  Design_RepresentativeIneq_SumRepresentative(instance, instance.edge_nb);
}

//Block separation of instance
// May be usefull in a sparse instances
// It will separate into independ blocks (useful for sparse graphs)
// Complexity is O(n^3) to separate into block then it send to Optmizations
void Block_Optimization(T_Instance &instance)
{
  cout << "Inside  Block_Optimization " << endl;
  int maxSizeBlock,
      I, J;
  double val;
  vector<int> aux_vertex;
  vector<int> vert_in_block;
  vector<T_Instance> vec_instances;
  vector<int> origen_indice; //useful to creat the valid inequalities in the instance
  origen_indice.resize(instance.edge_nb + 1);
  bool Neg;

  //first thing is to do a K-core elimination
  K_coreElimination(instance);

  if (instance.DIM == 0)
  {
    cout << "Nothing to do in block optmization instance.DIM = 0" << endl;
    cout << "Trivial solution is = " << instance.sum_cost << endl
         << " Thks ! (Press any thing to finish)";
    cin.get();
    return; // get out of function.
  }

  maxSizeBlock = instance.DIM;

  aux_vertex.resize(instance.DIM, 0); // each spot of vector represent a vertex if it is 1 them it shoul be eliminate because

  //try for each vertex of the graph to creat a block
  for (int i = 0; i < instance.DIM; i++)
    if (aux_vertex[i] == 0)
    { // edges in
      vert_in_block.clear();
      Neg = false;

      //set vertex i in block
      aux_vertex[i] = 1;
      vert_in_block.push_back(i + 1); // Start with 1, first vertex into a new block

      for (int z = 0; z < vert_in_block.size(); z++)
      { //for each vertex of the block we will see if they have any
        //printvector(vert_in_block); cin.get();
        for (int j = 0; j < instance.DIM; j++)
        {

          I = vert_in_block[z];
          J = j + 1;
          //check if j is ok for test
          if ((vert_in_block[z] - 1 == j) || (aux_vertex[j] != 0) || ((val = GetEdgeWeight(instance, I, J)) == 0))
            continue; // iterate j ++ do not continue

          aux_vertex[j] = 1;
          vert_in_block.push_back(j + 1); // Start with 1, inserting j in vertex block because has a edge that link one of elemeents of j

          if (val < 0) // just positive
            Neg = true;
          //restart z and j
          z = 0; // it means that z will be (z= 1) in next iteration. Thus the first vertex is not verified again
          break; //get out of for J
        }        //end FOR j
        if (vert_in_block.size() == maxSizeBlock)
          break;
      } //end FOR z

      // the vertex i should return to 0
      aux_vertex[i] = 0;
      //cout << "vert_in_block.size() =" << vert_in_block.size() <<", and maxSizeBlock =" << maxSizeBlock ; cin.get();

      if ((vert_in_block.size() < maxSizeBlock) || (i == instance.DIM - 1))
      { // it catches one block (it is good, so we shoould create a new instance)

        //Creat new instance
        T_Instance instance2;
        Copy_by_rank(instance, instance2, vert_in_block.size(), 0, origen_indice, vert_in_block);

        //inserting in vec
        vec_instances.push_back(instance2);

        maxSizeBlock -= instance2.DIM - 1;
        for (int jj = 0; jj < aux_vertex.size(); jj++)
          if (aux_vertex[jj] == 1)
            aux_vertex[jj] = 2; // Fixed
      }
      else
      {
        for (int jj = 0; jj < aux_vertex.size(); jj++)
          if (aux_vertex[jj] != 2)
            aux_vertex[jj] = 0; // restart all the vector
      }                         // end IF size bloc
    }                           //end FOR i

  //if (vec_instances.size() > 1)
  // 	cout << "It was possible to divide the graph in  (" <<  vec_instances.size() << ") subgraphs ";

  //OPTIMIZE all  blocks
  clock_t start_time = clock();
  bool PRINT_ITERATIONS = true;
  MAXTIME /= vec_instances.size(); //divide the time between our methods
  for (int i = 0; i < vec_instances.size(); i++)
  {
    if (i == vec_instances.size() - 1)
      PRINT_ITERATIONS = true; //just the last block will appear into the optimization
    else
      PRINT_ITERATIONS = false;

    if (vec_instances.size() > 1)
      K_coreElimination(vec_instances[i]); //try again to eliminate more vertices

    //send block_i to optmization
    if (vec_instances[i].DIM > 0)
      val = CuttingPlane_Optimization(vec_instances[i], PRINT_ITERATIONS, start_time);

    if (i < vec_instances.size() - 1)
    {
      vec_instances[i + 1].sum_cost = val; // put the objective function a block into the cost of next block;
      MAXTIME += MAXTIME;                  //global variable to limit the CPA time.
    }
  }
  //cout << "End of the function Block " << endl; cin.get();

} //end function block

//
//THis function will Eliminate(delete) vertices if 2 things happen:
//	1) The vertex has one edge (negative or posittive it does not matter)...
//	      	- if edge is positif we should sum to Objec. Function (ObF).
//		- if edge is negatif we do nothing with it
//	2) THe vertex has at most k-1 edges and all of them are positive, then we should add the sum of all edges into the ObF.
inline void K_coreElimination(T_Instance &instance)
{
  if (instance.DIM <= K)
  {
    cout << "Nothing to do IN  K_coreElimination !!! Because (instance.DIM = " << instance.DIM << ") and K = " << K << endl;
    return; //get out of function
  }
  int counter, I, J, aux_edge, cont_elim = 0;
  bool Neg, repeat = true;
  double val, sumVertex, sumTotalEliminated;
  vector<int> aux_vertex;
  vector<int> allowed_vertex;

  aux_vertex.resize(instance.DIM, 0); // each spot of vector represent a vertex if it is 1 them it shoul be eliminate because
  //remeber that edges start with 1 not 0 !!

  //printMatrix( instance.indice_cij,instance.DIM+1 );
  // printCostMatrix_CompleteGraph (instance);
  sumTotalEliminated = 0;
  while (repeat)
  {
    repeat = false;
    for (int i = 0; i < instance.DIM; i++)
      if (aux_vertex[i] == 0)
      {
        counter = 0;
        Neg = false;
        sumVertex = 0;
        for (int j = 0; j < instance.DIM; j++)
          if ((i != j) && (aux_vertex[j] == 0))
          {
            I = i + 1;
            J = j + 1;
            aux_edge = instance.indice_cij[I + (J) * (instance.DIM + 1)];
            if (aux_edge != -1 && (val = GetEdgeWeight(instance, I, J)) != 0)
            { //there existe an edge
              counter++;
              if (val > 0) // just positive
                sumVertex += val;
            }
            if (val < 0)
              Neg = true;
          } //end FOR j

        //cout << "i=" << i<<"counter =" << counter <<  " Neg =" << Neg << endl; //cin.get();
        //Elimanate
        if ((counter <= 1) || ((counter < K) && (!Neg)))
        {
          //cout << "i=" << i<<", counter =" << counter <<  " Neg =" << Neg << endl; cin.get();
          aux_vertex[i] = 1;
          cont_elim++;

          //if positive we should sum
          sumTotalEliminated += sumVertex;
        }
      } // end FOR i
  }     //end while

  //  cout << "We could delete ("<< cont_elim << ") vertices ... It is quite good, isn't it ? " <<  ", sumVertex = " << sumTotalEliminated <<endl;
  //  cin.get();

  instance.sum_cost += sumTotalEliminated;

  if (cont_elim > 0)
  {
    //vector with all the
    allowed_vertex.resize(instance.DIM - cont_elim);
    counter = 0;
    for (int i = 0; i < instance.DIM; i++)
      if (aux_vertex[i] == 0)
      {
        allowed_vertex[counter] = i + 1;
        counter++;
      }

    //printvector(allowed_vertex) ;
    int nbEdges = instance.DIM * (instance.DIM - 1) / 2;
    vector<int> origen_indice; //useful to creat the valid inequalities in the instance
    origen_indice.resize(nbEdges + 1);

    //cout << " Before instance.DIM = " << instance.DIM<< endl;

    T_Instance instance2; //Store information of instance
    Copy_by_rank(instance, instance2, allowed_vertex.size(), 0, origen_indice, allowed_vertex);
    //printCostMatrix_CompleteGraph (instance2);
    //Tranfome instance in new instance without the
    instance = instance2;
  }
  //printCostMatrix_CompleteGraph (instance);
  //printMatrix( instance.indice_cij,instance.DIM+1 );
  //cin.get();
}

inline void printCostMatrix_SparseGraph(const T_Instance &instance)
{
  int aux_edge;
  cout << endl
       << "printCostMatrix  with Dim = " << instance.DIM << ", and edge_nb= " << instance.edge_nb << endl;
  for (int i = 0; i < instance.DIM; i++)
    for (int j = i + 1; j < instance.DIM; j++)
    {
      if (aux_edge = instance.indice_cij[(j + 1) + (i + 1) * (instance.DIM + 1)] != -1)
        cout << "( " << i + 1 << "," << j + 1 << ") =" << GetEdgeWeight(instance, i + 1, j + 1) << endl;
    }

  cout << endl;
}
inline void printCostMatrix_CompleteGraph(const T_Instance &instance)
{
  cout << endl
       << "printCostMatrix  with Dim = " << instance.DIM << endl;
  for (int i = 0; i < instance.DIM; i++)
    for (int j = i + 1; j < instance.DIM; j++)
      cout << "( " << i + 1 << "," << j + 1 << ") =" << GetEdgeWeight(instance, i + 1, j + 1) << endl;

  cout << endl;
}

//Using Minimum Degree triagulation to creat a chordal extention of our graph
inline void Chordal_extention(T_Instance &instance)
{
  bool R;
  int vertex;
  int MinSizeClique_chordal = 3; //minimum size to consider a clique (otherwise non triangle inequaliti is going to be violated)

  vector<int> deegre_vertice(instance.DIM, 0);
  vector<int> neighb_vertice;
  vector<bool> used_v(instance.DIM, false); // true if vertex already analaysed

  //Initial degree
  for (int i = 0; i < instance.DIM; ++i)
    for (int j = 0; j < instance.DIM; ++j)
      if (instance.indice_cij[(i + 1) + (j + 1) * (instance.DIM + 1)] != -1)
        deegre_vertice[i]++;

  for (int i = 0; i < instance.DIM - 2; ++i)
  { // (-2) because it can creat at most
    vertex = MinimumDegree(instance, deegre_vertice, used_v);
    used_v[vertex] = true; // remove vertex from next analyzes

    Set_Neighbourvertices(instance, vertex, used_v, &neighb_vertice);

    if (neighb_vertice.size() + 1 >= MinSizeClique_chordal)
      R = Set_Clque_in_MaximalClique(vertex, neighb_vertice, &maximalCliques);
    else
      R = false;

    if (R == true)
    {
      //			printvector(neighb_vertice) ;
      //make  clique with all neighbours  of vertex
      for (int n_i = 0; n_i < neighb_vertice.size() - 1; ++n_i)
        for (int n_j = n_i + 1; n_j < neighb_vertice.size(); ++n_j)
        {
          if (!ExistEdge(instance, neighb_vertice[n_j], neighb_vertice[n_i]))
          {
            Add_edgeInstance(instance, neighb_vertice[n_j], neighb_vertice[n_i]);
            //Increase their degree
            deegre_vertice[neighb_vertice[n_j]]++;
            deegre_vertice[neighb_vertice[n_i]]++;
          }
        } //End for clique insertion
    }     //end IF enter to inserction
          //		printDoubleVector(maximalCliques);	cin.get();
  }       //End of FOR i

} //end function

inline bool ExistEdge(const T_Instance &instance, const int &n_j, const int &n_i)
{
  if (instance.indice_cij[(n_j + 1) + (n_i + 1) * (instance.DIM + 1)] != -1)
    return true;
  else
    return false;
}
inline bool Set_Clque_in_MaximalClique(const int &vertex, std::vector<int> neighb_vertice, std::vector<std::vector<int>> *maximalCliques)
{
  int sizeV = neighb_vertice.size();

  for (int i = 0; i < (*maximalCliques).size(); ++i)
  {
    if ((*maximalCliques)[i].size() >= sizeV) //must be (>=) cause we have to add the last vertex
      for (int j = 0; j < (*maximalCliques)[i].size(); ++j)
        if ((*maximalCliques)[i][j] == vertex)
          return false;
  } //end FOR i maximal clique size

  //if sizeV is maximal (thus include in Maximal clique)
  (*maximalCliques).push_back(neighb_vertice);
  (*maximalCliques)[(*maximalCliques).size() - 1].push_back(vertex);
  return true;
}
inline void Set_Neighbourvertices(const T_Instance &instance, const int &vertex, std::vector<bool> select_v, std::vector<int> *neighb_vertice)
{
  (*neighb_vertice).clear();

  for (int i = 0; i < instance.DIM; ++i)
  {
    if (instance.indice_cij[(i + 1) + (vertex + 1) * (instance.DIM + 1)] != -1 && select_v[i] == false)
      (*neighb_vertice).push_back(i);
  }
}
inline int MinimumDegree(const T_Instance &instance, std::vector<int> deegre_vertice, std::vector<bool> select_v)
{
  int min_val = instance.DIM,
      min_ver = -1;
  for (int i = 0; i < instance.DIM; ++i)
    if (deegre_vertice[i] < min_val && select_v[i] == false)
    {
      min_val = deegre_vertice[i];
      min_ver = i;
    }
  return min_ver;
}

inline void Add_edgeInstance(T_Instance &instance, const int &J, const int &I)
{
  int index = instance.edge_nb; // remember that we start at 0;
  int i = I + 1,
      j = J + 1;

  if (instance.indice_cij[i + (j) * (instance.DIM + 1)] == -1)
  { //there is no edge (thus we need to add one with cost of 0)

    instance.indice_cij[i + (j) * (instance.DIM + 1)] = index;
    instance.indice_cij[j + (i) * (instance.DIM + 1)] = index;
    instance.ant_indice_cij[index].idx_i = j; // start from 1
    instance.ant_indice_cij[index].idx_j = i; // in SDP idx j should be the smaller one
    index++;

    instance.cij.barc_j.push_back(i);
    instance.cij.barc_i.push_back(j);
    instance.cij.barc_v.push_back(0.0); // it is the edge weight

    ++instance.edge_nb;
  }

  instance.totalVars = instance.edge_nb;

  if (instance.edge_nb == (instance.DIM * (instance.DIM - 1)) / 2)
    ISCOMPLETE_GRAPH = true;
}
inline void BuildCompleteGraph(T_Instance &instance)
{
  int index = instance.edge_nb; // remember that we start at 0;
  //actulizing the number of edges
  instance.edge_nb = (instance.DIM * (instance.DIM - 1)) / 2;

  //remeber that edges start with 1 not 0 !!!
  for (int i = 1; i < instance.DIM + 1; i++)
    for (int j = i + 1; j < instance.DIM + 1; j++)
    {
      if (instance.indice_cij[i + (j) * (instance.DIM + 1)] == -1)
      { //there is no edge (thus we need to add one with cost of 0)

        instance.indice_cij[i + (j) * (instance.DIM + 1)] = index;
        instance.indice_cij[j + (i) * (instance.DIM + 1)] = index;
        instance.ant_indice_cij[index].idx_i = j; // start from 1
        instance.ant_indice_cij[index].idx_j = i; // in SDP idx j should be the smaller one
        index++;

        instance.cij.barc_j.push_back(i);
        instance.cij.barc_i.push_back(j);
        instance.cij.barc_v.push_back(0.0); // it is the edge weight
      }
    } //end FOR

  ISCOMPLETE_GRAPH = true;

  instance.totalVars = instance.edge_nb;
  //cout << instance.indice_cij[50 + (50)*(instance.DIM+1)]<<  ", dim ="<<instance.DIM <<endl;
  //printMatrix(instance.indice_cij, 51);

  //cin.get();
}

void Lire_Instance(const std::string nomFichier, T_Instance &instance)
{
  int auxInt = 0;
  double weight;
  int fst, scon;

  cout << nomFichier;
  ifstream fichier(nomFichier.c_str(), ios::in); // reading open

  if (!fichier) //si le fichier n'est existe pas
  {
    cout << "It did not find the input file to optimize\n ";
    exit(1);
  }

  fichier >> instance.DIM;
  auxInt = instance.DIM;
  instance.LENBARVAR = (auxInt * (auxInt + 1) / 2);
  fichier >> instance.edge_nb;

  //Allocate memory for cost
  instance.indice_cij.resize((instance.DIM + 1) * (instance.DIM + 1), -1);
  instance.ant_indice_cij.resize(((instance.DIM + 1) * (instance.DIM)) / 2); // variables i and j of indice_cij

  instance.sum_cost = 0.0;
  /*Read the cij values - should inicialize with 1 because [0] is edge x00 */
  int index = 0;
  int reductionNbEdge = 0;
  for (int i = 0; i < instance.edge_nb; i++)
  {
    /*first j (menor numero),  after i */
    fichier >> fst;
    fichier >> scon;
    fichier >> weight;

    //indice
    if (fst != scon && (weight < -epslon || weight > epslon))
    { // do not accept loop or weight==0
      if (instance.indice_cij[fst + scon * (instance.DIM + 1)] == -1)
      {

        //index of edge to find it easely
        instance.indice_cij[fst + scon * (instance.DIM + 1)] = index;
        instance.indice_cij[scon + fst * (instance.DIM + 1)] = index;

        //For the SDP
        if (fst < scon)
        {
          instance.cij.barc_j.push_back(fst);
          instance.cij.barc_i.push_back(scon);
          instance.ant_indice_cij[index].idx_j = fst;
          instance.ant_indice_cij[index].idx_i = scon;
        }
        else
        {
          instance.cij.barc_j.push_back(scon);
          instance.cij.barc_i.push_back(fst);
          instance.ant_indice_cij[index].idx_i = fst;
          instance.ant_indice_cij[index].idx_j = scon;
        }
        instance.cij.barc_v.push_back(weight);

        index++;
      }
      else
      { //@TODO:Execption if edge is repeated

        int indc = instance.indice_cij[fst + scon * (instance.DIM + 1)];
        instance.cij.barc_v[indc] += weight;
        reductionNbEdge++; // Nb of edges (x variables must be reduced)
      }
    }
    else
    {
      reductionNbEdge++; // we do not accept loop
    }                    //if fst diff scon
  }

  instance.edge_nb -= reductionNbEdge;
  instance.totalVars = instance.edge_nb;

  //printMatrix( instance.indice_cij,instance.DIM+1 );

  /*End of fichier*/
  fichier.close();
}

/* Old parts of CPLEX

inline void PrintValsVarCplex(const T_Instance &instance, const IloNumArray &vals)
{
  int counter = 0;
for (int j=1; j<=instance.DIM; j++){
  for (int z=j+1; z<=instance.DIM; z++)
    if (instance.indice_cij[j+z*(instance.DIM+1)] != -1){
    cout << "x("<<j << "," << z<< ")= " << vals[instance.indice_cij[j+z*(instance.DIM+1)]]<< "; " ;
    }
   cout << endl;
}
}

*/
