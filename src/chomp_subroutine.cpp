#include "chomp_predict/chomp_subroutine.h"

using namespace CHOMP;

Solver::Solver(){};

void Solver::set_problem(MatrixXd A,VectorXd b,DynamicEDTOctomap* edf,CostParam cost_param){
    // compute inverse in advance for the future use in optimization loop
    prior_inverse = A.inverse();
    // set quadratic term
    this->A = A;
    this->b = b;
    this->edf_ptr = edf;
    if(this->edf_ptr!=NULL)
        is_problem_set = true;
    map_type = 0; // octomap         
    this->cost_param = cost_param;
}

void Solver::set_problem(MatrixXd A,VectorXd b,voxblox::EsdfServer* esdf_ptr,CostParam cost_param){
    // compute inverse in advance for the future use in optimization loop 
    prior_inverse = A.inverse();
    // set quadratic term
    this->A = A;
    this->b = b;
    this->esdf_ptr = esdf_ptr;
    if(this->esdf_ptr!=NULL)
        is_problem_set = true;
    map_type = 1; // voxblox        
    this->cost_param = cost_param;      
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
        VectorXd grad_cost = weight_prior * (A*x_prev + b) + grad_cost_obstacle(x_prev);                

        // record
        optim_info.nonlinear_cost_history.push_back(nonlinear_cost);
        optim_info.prior_cost_history.push_back(prior_cost);
        optim_info.total_cost_history.push_back(weight_prior*prior_cost + nonlinear_cost);


        // update
        VectorXd update = -learning_rate*(this->prior_inverse)*(grad_cost);
        x = x_prev + update; 

        iter++;
        innovation = (x-x_prev).norm();  
        if((iter % 15 == 0) or (iter == 1))
        // print 
        printf("[CHOMP] iter %d = obst_cost : %f / prior_cost %f / total_cost %f // innovation: %f\n",iter,
                nonlinear_cost,weight_prior*prior_cost,weight_prior*prior_cost + nonlinear_cost,innovation);

        // is it mature?
        if (iter > max_iter)
            std::cout<<"[CHOMP] reached maximum number of iteration."<<std::endl;
        x_prev = x;
    }

    std::cout<<"[CHOMP] optimization finished at "<<iter<<" iteration."<<std::endl;

    OptimResult optim_result;
    optim_result.result_verbose = optim_info;
    optim_result.solution = x;
    return optim_result;
}

Vector2d Solver::evaluate_costs(VectorXd x){
    Vector2d costs;
    double prior_term = (0.5*x.transpose()*A*x + b.transpose()*x)(0,0); // (0,0) is for conversion from matrix to a scalar
    double nonlinear_term = cost_obstacle(x); 
    costs(0) = prior_term;
    costs(1) = nonlinear_term;
    // cout<<"[in function evaluate_costs] evaluted cost for nonlinear term: "<<nonlinear_term<<endl;

    return costs;
} 

// evaluate cost at one point 
double Solver::cost_at_point(geometry_msgs::Point p){    
    try{
        if (!is_problem_set)
            throw 1;
        double distance_raw = 0;

        if (map_type == 0)
            distance_raw = edf_ptr->getDistance(octomap::point3d(p.x,p.y,p.z));
        else{
             Vector3d position(p.x,p.y,p.z);
             esdf_ptr->getEsdfMapPtr()->getDistanceAtPosition(position,&distance_raw);
        }
        // compute real cost from distance value 
        if (distance_raw <=0 )
           return (-distance_raw + 0.5*cost_param.r_safe); 
        else if((0<distance_raw) and (distance_raw < cost_param.r_safe) ){
            return 1/(2*cost_param.r_safe)*pow(distance_raw - cost_param.r_safe,2);                
        }else        
            return 0;

    }catch(exception e){
        std::cout<<"error in evaulating EDT value. Is edf completely loaded?"<<std::endl;
    }
}


// evalute cost of a path 
double Solver::cost_obstacle(VectorXd x){    
    // length(x) = dim x H
    double cost = 0;
    int H = x.size()/2;
    for(int h = 0;h<H;h++){
            geometry_msgs::Point p;
            p.x = x(2*h);
            p.y = x(2*h+1);      
            p.z = cost_param.ground_height+1e-2;                          
            cost += cost_at_point(p);        
        }

    return cost;
}
// evaluate gradient of cost of a path 
VectorXd Solver::grad_cost_obstacle(VectorXd x){
    // length(x) = dim x H
    int H = x.size()/2;
    double cost0 = cost_obstacle(x); // original cost 
    VectorXd grad(x.size());    
    for(int h = 0;h<H;h++){
        VectorXd pert_x(x.size()); pert_x.setZero(); pert_x(2*h) = cost_param.dx; grad(2*h) = (cost_obstacle(x+pert_x) - cost0)/cost_param.dx;
        VectorXd pert_y(x.size()); pert_y.setZero(); pert_y(2*h+1) = cost_param.dx; grad(2*h+1) = (cost_obstacle(x+pert_y) - cost0)/cost_param.dx;
    }
    return grad;
}
