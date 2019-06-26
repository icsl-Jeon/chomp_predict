#include "chomp_predict/chomp_subroutine.h"

using namespace CHOMP;

Solver::Solver(){};

void Solver::set_problem(MatrixXd A,VectorXd b,double (*nonlinear_cost)(VectorXd),VectorXd (*nonlinear_cost_grad)(VectorXd)){
    // compute inverse in advance for the future use in optimization loop
    prior_inverse = A.inverse();
    
    // set non-quadratic term
    this->nonlinear_cost=nonlinear_cost;
    this->nonlinear_cost_grad=nonlinear_cost_grad;

    // set quadratic term
    this->A = A;
    this->b = b;
}

OptimResult Solver::solve(VectorXd x0, OptimParam optim_param){
    // history 
    OptimInfo optim_info;

    // parsing 
    double term_cond = optim_param.termination_cond;
    int max_iter = optim_param.max_iter;
    double weight_prior = optim_param.weight_prior;
    double learning_rate = optim_param.descending_rate;


    VectorXd x_prev = x0;
    VectorXd x;    


    // dimension of optimization variable 
    int N = x0.size();    
    double innovation = 1e+4;
    int iter = 0;

    while (iter <= max_iter && innovation > term_cond){

        // cost computation                 
        Vector2d costs = evaluate_costs(x_prev);
        double prior_cost = costs(0), nonlinear_cost = costs(1);
        
        // gradient computation                 
        VectorXd grad_cost = weight_prior * (A*x_prev + b) + nonlinear_cost_grad(x_prev);                

        // record
        optim_info.nonlinear_cost_history.push_back(nonlinear_cost);
        optim_info.prior_cost_history.push_back(prior_cost);
        optim_info.total_cost_history.push_back(weight_prior*prior_cost + nonlinear_cost);
        
        // update
        VectorXd update = -learning_rate*(this->prior_inverse)*(grad_cost);
        x = x_prev + update; 

        iter++;
        innovation = (x-x0).norm();  

        // is it mature?
        if (iter > max_iter)
            std::cout<<"[CHOMP] reached maximum number of iteration."<<std::endl;

    }
    OptimResult optim_result;
    optim_result.result_verbose = optim_info;
    optim_result.solution = x;
    return optim_result;
}

Vector2d Solver::evaluate_costs(VectorXd x){
    Vector2d costs;
    double prior_term = (0.5*x.transpose()*A*x + b.transpose()*x)(0,0); // (0,0) is for conversion from matrix to a scalar
    double nonlinear_term = nonlinear_cost(x); 
    costs(0) = prior_term;
    costs(1) = nonlinear_term;
    return costs;
} 

