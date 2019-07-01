#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include "chomp_utils.h"


#define DIM_EXCEPTION 0; // dimenstion mismatch  

using namespace std;
using namespace Eigen;


/**
 * @brief This script solves the CHOMP programming : 1/2 x'Ax + bx + f(x)
 * This requires evaluation function for f(x) and grad_f(x) as a function pointer and other parameter
 */

namespace CHOMP{

// parameter to make a nonlinear cost (obstacle) 
struct CostParam{
    double r_safe;  // extent to which gradient will effct 
    double dx; // perturbation 
    double ground_height; // height for 2D problem formulation  
};

// parameter for optimization 
struct OptimParam{
    int max_iter;
    double termination_cond; // norm of innovation to be terminated  
    double descending_rate; // step rate      
    double weight_prior; // weight for prior
    double gamma; // discount
    int n_step; // discount
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
        MatrixXd A;
        MatrixXd b;
        DynamicEDTOctomap* edf_ptr; 
        CostParam cost_param; // cost param

    public:
        //constructor     
        Solver();
        // setting optimization problem
        void set_problem(MatrixXd A,VectorXd b,DynamicEDTOctomap*,CostParam);       
        // solve and return the result
        OptimResult solve(VectorXd x0, OptimParam optimization_param); // run optimization routine                 
        // cost (edf should be given first)
        Vector2d evaluate_costs(VectorXd x); // evaluate costs at x (not weigthed). {prior,nonlinear} if provided           
        double cost_at_point(geometry_msgs::Point p); // c(x)
        double cost_obstacle(VectorXd x); 
        VectorXd grad_cost_obstacle(VectorXd x);
};

}