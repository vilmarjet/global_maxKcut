# Instruction to execute branch and bound for max-k-cut

## Application designed for Unix systems, only.

Let <global_maxkcut> be the path folder of where you saved the this project.

The following are steps to build, personallyze and execute the application.
### 1) MOSEK:
    Mosek is the solver chosen for solve SDP and LP formulations.  
    For installation: https://www.mosek.com/

### 2) Set-up of "Makefile":
    In file <global_maxkcut>/Makefile set mosek directories,
    MOSEK_IPATHS and MOSEK_BINPATH set the head and bin folder of mosek, for example:
    
    MOSEK_IPATHS=-I ../../mosek/9.1/tools/platform/linux64x86/h
    MOSEK_BINPATH=../../mosek/9.1/tools/platform/linux64x86/bin

### 3) Set-up parameters files:
    Go to resource/parameters, set the following files with your peferences:
     * BranchAndBoundParameters.txt: parameters for branch and bound method,
     * CuttingPlaneParameters.txt: parameters for cutting plane method used to find upper bound,
     * HeuristicParameters.txt: set heuristic and other parameters to be used in branch and bound,
     * ProblemParameters.txt: parameters of problem, for example: input_instance, partition_number, solver_type 
     
### 4)Installation of application (bb_mkc_run)
    In terminal, go to <global_maxkcut> (where Makefile is saved), then execute the following command:
    
    $ make install 

### 5) Execute example and parameter
    The application can be executed with 0, 1 or 2 arguments:
    a) run without arguments, this way, all the parameters in ProblemParameters.txt will be considered:

        $ make run
            or 
        $./bb_mkc_run

    b) The bb_mkc_run can receive in the first argument the path for instance test. Thus, the file_name set in ProblemParameters.txt will be disconsidered.

        $ ./bb_mkc_run ./resource/instance4.txt 
        
        the ./resource/instance4.txt  can be changed to path of folder where you set your graph instances.
    
    c) The application can also receive as arguments the number of partions. Thus, partition_number will be disconsidered.

        $ ./bb_mkc_run ./resource/instance4.txt 3

        Then test in instance4.txt will be executed for 3 partitions. 

### 6) Results:
    In BranchAndBoundParameters.txt it is possible to set the name of file where result of branch-and-bound is output. 
    The result file will be created at the end of first execution and updated at each execution
        <global_maxkcut>/target/results
        

## References:

V.J. Rodrigues de Sousa, M.F. Anjos, and S. Le Digabel,
[Improving the linear relaxation of maximum $k$-cut with semidefinite-based constraints](http://dx.doi.org/10.1007/s13675-019-00110-y).
[EURO Journal on Computational Optimization](https://www.scimagojr.com/journalsearch.php?q=21100827466&tip=sid&clean=0), 7(2), p. 123-151, 2019.
[bibtex](https://www.gerad.ca/Sebastien.Le.Digabel/Publications/bibtex/dSAnLed2018.bib)

V.J. Rodrigues de Sousa, M.F. Anjos, and S. Le Digabel,
[Computational Study of Valid Inequalities for the Maximum k-Cut Problem](http://dx.doi.org/10.1007/s10479-017-2448-9).
[Annals of Operations Research](http://www.scimagojr.com/journalsearch.php?q=23090&tip=sid&clean=0), 265(1), p. 5-27, 2018.
[bibtex](https://www.gerad.ca/Sebastien.Le.Digabel/Publications/bibtex/dSAnLed2016.bib)

V.J. Rodrigues de Sousa, M.F. Anjos, and S. Le Digabel,
[Computational study of a branching algorithm for the maximum $k$-cut problem](http://www.optimization-online.org/DB_HTML/2020/02/7629.html).
Technical report, Les Cahiers du GERAD G-2020-11.
[bibtex](https://www.gerad.ca/Sebastien.Le.Digabel/Publications/bibtex/dSAnLed2020.bib)
