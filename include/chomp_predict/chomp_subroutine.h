#include <eigen3/Eigen/Core>
#include "chomp_utils.h"

#define DIM_EXCEPTION 0; // dimenstion mismatch  

using namespace std;
using namespace Eigen;


/**
 * @brief This script solves the CHOMP programming : 1/2 x'Ax + bx + f(x)
 * This requires evaluation function for f(x) and grad_f(x) as a function pointer and other parameter
 */

namespace CHOMP{

// parametter for optimization 
struct OptimParam{
    int max_iter;
    double termination_cond; // norm of innovation to be terminated  
    double descending_rate; // step rate      
    double weight_prior; // weight for prior
};

// information after optimization (record for history)
struct OptimInfo{
    vector<double> prior_cost_history;
    vector<double> nonlinear_cost_history;
    vector<double> total_cost_history;
};

// result to be obtained after optimization 
struct OptimResult{
    OptimInfo result_verbose;
    Eigen::VectorXd solution;
};

// solver class 
class Solver{

    private: 
        bool is_problem_set; // should be checked whether programming is set
        MatrixXd prior_inverse; 
        double (*nonlinear_cost)(VectorXd);
        VectorXd (*nonlinear_cost_grad)(VectorXd);
        MatrixXd A;
        MatrixXd b;
        
    public:
        //constructor     
        Solver();
        // setting optimization problem
        void set_problem(MatrixXd A,VectorXd b,
            double (*nonlinear_cost)(VectorXd),
            VectorXd (*nonlinear_cost_grad)(VectorXd)); 
        
        // solve and return the result
        OptimResult solve(VectorXd x0, OptimParam optimization_param); // run optimization routine                 
        Vector2d evaluate_costs(VectorXd x); // evaluate costs at x (not weigthed). {prior,nonlinear}           
};


}