#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include "chomp_utils.h"


#define DIM_EXCEPTION 0; // dimenstion mismatch  

using namespace std;
using namespace Eigen;
/**
 * @brief Functor for shaping function from distance_raw
 * 
 */
class shaping_functor{
    private: 
        double r_safe;

    public:
        shaping_functor(double r_safe):r_safe(r_safe){};
        double operator()(double distance_raw){
        if (distance_raw <=0 )
            return (-distance_raw + 0.5*r_safe); 
        else if((0<distance_raw) and (distance_raw < r_safe) ){
            return 1/(2*r_safe)*pow(distance_raw - r_safe,2);                
        }else        
            return 0;
    };
};

class shaping_functor_grad{
    private:
        double r_safe;
    public: 
        shaping_functor_grad(double r_safe):r_safe(r_safe){};
        double operator()(double distance_raw){
            if (distance_raw <=0 )
                return -1 ; 
            else if((0<distance_raw) and (distance_raw < r_safe) ){
                return 1/(r_safe)*(distance_raw - r_safe);                
            }else        
                return 0;
        }
};


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
    double descending_rate_min; // step rate
    double descending_rate_max; // step rate        
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
    double distance_min = 1; // minimum distance value (negative = containing obstacle point)
};

/**
 * @brief gradient containing prior and nonlinear together 
 * 
 */
struct OptimGrad{
    VectorXd prior_gard;
    VectorXd nonlinear_grad;
};

// solver class 
class Solver{

    private: 
        int map_type;     
        bool is_problem_set; // should be checked whether programming is set
        MatrixXd prior_inverse; 
        MatrixXd A;
        MatrixXd b;
        // only one of the followings will be used for distnace query 
        DynamicEDTOctomap* edf_ptr = NULL; 
        voxblox::EsdfServer* esdf_ptr = NULL; 
        CostParam cost_param; // cost param


        // sub moudles 
        // cost (edf should be given first)
        Vector2d evaluate_costs(VectorXd x); // evaluate costs at x (not weigthed). {prior,nonlinear} if provided      
        Vector2d evaluate_costs2(VectorXd x); // evaluate costs at x  using voxblox batch evalution 
                
        double cost_at_point(geometry_msgs::Point p); // c(x)
        double cost_obstacle(VectorXd x); 
        VectorXd grad_cost_obstacle(VectorXd x);        
        inline VectorXd grad_and_cost_obstacle2(VectorXd x,Vector2d &  costs,OptimGrad& grad);        
        
    public:
        //constructor     
        Solver();
        // setting optimization problem
        void set_problem(MatrixXd A,VectorXd b,DynamicEDTOctomap*,CostParam);
        void set_problem(MatrixXd A,VectorXd b,voxblox::EsdfServer*,CostParam);                      
        // solve and return the result
        OptimResult solve(VectorXd x0, OptimParam optimization_param); // run optimization routine          
        OptimResult solve2(VectorXd x0, OptimParam optimization_param); // run optimization routine using voxblox built-in function : batch evaluation                 

};

}