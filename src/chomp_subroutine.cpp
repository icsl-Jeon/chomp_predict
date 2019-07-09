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
    ros::Time tic,toc; 
    double time_sum = 0;

    while (iter <= max_iter && innovation > term_cond){        
        // cost computation                 
        tic = ros::Time::now();    
        Vector2d costs = evaluate_costs(x_prev);
        double prior_cost = costs(0), nonlinear_cost = costs(1);
        
        // gradient computation                 
        VectorXd grad_nonlinear = grad_cost_obstacle(x_prev);
        VectorXd grad_cost = weight_prior * (A*x_prev + b) + grad_nonlinear;                
        toc = ros::Time::now();

        time_sum += (toc - tic).toSec();

        // record
        optim_info.nonlinear_cost_history.push_back(nonlinear_cost);
        optim_info.prior_cost_history.push_back(prior_cost);
        optim_info.total_cost_history.push_back(weight_prior*prior_cost + nonlinear_cost);


        // update
        VectorXd update = -learning_rate*(this->prior_inverse)*(grad_cost);
        x = x_prev + update; 

        iter++;
        innovation = (x-x_prev).norm();  
        if((iter % 15 == 0) or (iter == 1)){
        // print 
        printf("[CHOMP] iter %d = obst_cost : %f / prior_cost %f / total_cost %f // innovation: %f\n",iter,
                nonlinear_cost,weight_prior*prior_cost,weight_prior*prior_cost + nonlinear_cost,innovation);
        
        printf("    grad : prior %f / obstacle %f / total %f / final update %f // learning rate: %f  /// eval_time = %f \n ",
        weight_prior * (A*x_prev + b).norm(),(grad_nonlinear).norm(),grad_cost.norm(),update.norm(),
        learning_rate,(time_sum /15.0));
        time_sum = 0;
        }
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


/**
 * @brief modified optimization solving routine  
 * @details momentum 
 * @param x0 : initial guess 
 * @param optim_param :
 * @return OptimResult 
 */
OptimResult Solver::solve2(VectorXd x0, OptimParam optim_param){
    // history 
    OptimInfo optim_info;

    // parsing 
    double term_cond = optim_param.termination_cond;
    int max_iter = optim_param.max_iter;
    double weight_prior = optim_param.weight_prior;
    double learning_rate = optim_param.descending_rate;


    VectorXd x_prev = x0; double cost_prev = 1e+4; double cost; double delta_cost;
    VectorXd x;    
    double dist_min = -1e+2;
    MatrixXd obst_grad_weight = MatrixXd::Identity(x_prev.size(),x_prev.size());
    // double  obst_grad_weight = 1; // dimension of optimization variable 
    int N = x0.size();    
    double innovation = 1e+4;
    int iter = 0;
    ros::Time tic,toc; 
    double time_sum = 0;
    while ((iter <= max_iter) && ((innovation > term_cond) or (dist_min < 0.)) ){        
        // cost computation              

        Vector2d costs;
        OptimGrad optim_grad;
        tic = ros::Time::now();
        VectorXd distance_set = grad_and_cost_obstacle2(x_prev,costs,optim_grad); // evaluate cost and grad simultanously 
        toc = ros::Time::now();
        time_sum += (toc - tic).toSec();

        double prior_cost = costs(0), nonlinear_cost = costs(1);
       // cout<<"distance_set"<<endl;
       // cout<<distance_set.transpose()<<endl;        
        // gradient computation                 
        VectorXd grad_cost = weight_prior * optim_grad.prior_gard + obst_grad_weight*optim_grad.nonlinear_grad;                
        
        cost = weight_prior*prior_cost + nonlinear_cost;
        delta_cost = abs(cost - cost_prev);

        
        // record
        optim_info.nonlinear_cost_history.push_back(nonlinear_cost);
        optim_info.prior_cost_history.push_back(prior_cost);
        optim_info.total_cost_history.push_back(weight_prior*prior_cost + nonlinear_cost);

        // update
        VectorXd update = -learning_rate*(this->prior_inverse)*(grad_cost);
        x = x_prev + update; 

        iter++;
        // innovation = (x-x_prev).norm();  // this innovation is gradient norm version
        innovation = delta_cost;  
        // adaptation of  step size by total cost                  
        // if (cost - cost_prev < 0){
        //     learning_rate = (1.2 * learning_rate > optim_param.descending_rate_max) ?  optim_param.descending_rate_max : (1.2*learning_rate);

        // }
        // else{
        //     learning_rate = (0.6*learning_rate < optim_param.descending_rate_min) ?  optim_param.descending_rate_min : (0.6*learning_rate) ;
        // }

        Index min_idx;
        dist_min = distance_set.minCoeff(&min_idx);
        //cout<<"distance min value"<<endl;
        //cout<< dist_min<<endl;
        // adaptation of obstacle weight by obstacle cost                  
        // if (dist_min < 0){
        //     obst_grad_weight = 1.5 * obst_grad_weight;
        // }
        // else
        //    obst_grad_weight = MatrixXd::Identity(x_prev.size(),x_prev.size()); 
        if((iter % 15 == 0) or (iter == 2)){
        // print 
        printf("[CHOMP] iter %d = obst_cost : %f / prior_cost %f / total_cost %f // cost_diff: %f / min_dist_val %f \n",iter,
                nonlinear_cost,weight_prior*prior_cost,weight_prior*prior_cost + nonlinear_cost,innovation,dist_min);

        printf("    grad : prior %f / obstacle %f / total %f / final update %f // learning rate: %f  /// eval_time = %f \n ",
        weight_prior*optim_grad.prior_gard.norm(),(obst_grad_weight*optim_grad.nonlinear_grad).norm(),grad_cost.norm(),update.norm(),
        learning_rate,(time_sum /15.0));
        time_sum = 0;
        }
		// is it mature?
        if (iter > max_iter)
            std::cout<<"[CHOMP] reached maximum number of iteration."<<std::endl;
        x_prev = x;

        cost_prev = cost;
	}

    std::cout<<"[CHOMP] optimization finished at "<<iter<<" iteration."<<std::endl;

    OptimResult optim_result;
    optim_result.result_verbose = optim_info;
    optim_result.solution = x;
    optim_result.distance_min = dist_min;
    return optim_result;
}





// private modules  
Vector2d Solver::evaluate_costs(VectorXd x){
    Vector2d costs;
    double prior_term = (0.5*x.transpose()*A*x + b.transpose()*x)(0,0); // (0,0) is for conversion from matrix to a scalar
    double nonlinear_term = cost_obstacle(x); 
    costs(0) = prior_term;
    costs(1) = nonlinear_term;
    // cout<<"[in function evaluate_costs] evaluted cost for nonlinear term: "<<nonlinear_term<<endl;

    return costs;
} 

/**
 * @brief Use this function only in voxblox mode 
 * 
 * @param x 
 * @return Vector2d 
 */
Vector2d Solver::evaluate_costs2(VectorXd x){
    Vector2d costs;
    double prior_term = (0.5*x.transpose()*A*x + b.transpose()*x)(0,0); // (0,0) is for conversion from matrix to a scalar
    
    // replaced with batch evaluation 
    int H = x.size()/2; // this will be cols of Matrix3Xd matrix 
    MatrixXd xy_path = x; xy_path.resize(2,H);
    MatrixXd z_path = MatrixXd::Ones(1,H) * cost_param.ground_height;    
    
    Matrix3Xd batch_mat(3,H);
    batch_mat << xy_path , z_path; // final evaluation batch (3 x H) 
    Ref<const Matrix<double,3,-1,ColMajor>,0,Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> batch_mat_ref(batch_mat);
    // cout<<"batch_mat_ref:"<<endl;
    // cout<<batch_mat_ref<<endl;
    VectorXd distance_set(H);
    VectorXi observation_set(H); 
    esdf_ptr->getEsdfMapPtr()->batchGetDistanceAtPosition(batch_mat_ref,distance_set,observation_set);
    // cout<<distance_set<<endl;
    costs(0) = prior_term;
    costs(1) = distance_set.sum();
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
            p.z = cost_param.ground_height;                          
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


/**
 * @brief evalute cost gradient for prior and nonlinear 
 * 
 * @param x optim_var (2H x 1)
 * @param prior_and_obstacle_costs  
 * @param grad 
 * @return distance set. This is sometines needed as the obstacle detouring condition   
 */
inline VectorXd Solver::grad_and_cost_obstacle2(VectorXd x,Vector2d & prior_and_obstacle_costs,OptimGrad& grad){
    // length(x) = dim x H

    int H = x.size()/2; // this will be cols of Matrix3Xd matrix 
    MatrixXd xy_path = x; xy_path.resize(2,H);
    MatrixXd z_path = MatrixXd::Ones(1,H) * cost_param.ground_height;    
    
    Matrix3Xd batch_mat(3,H);
    batch_mat << xy_path , z_path; // final evaluation batch (3 x H) 
    Ref<const Matrix<double,3,-1,ColMajor>,0,Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> batch_mat_ref(batch_mat);
    // cout<<"batch_mat_ref:"<<endl;
    // cout<<batch_mat_ref<<endl;
    VectorXd distance_set(H);
    VectorXi observation_set(H); 

    Matrix3Xd batch_grad(3,H);
    Ref<Matrix<double,3,-1,ColMajor>,0,Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> batch_grad_ref(batch_grad);
    esdf_ptr->getEsdfMapPtr()->batchGetDistanceAndGradientAtPosition(batch_mat_ref,distance_set,batch_grad_ref,observation_set);    

    // shaping function 
    shaping_functor shp_fun(cost_param.r_safe);
    shaping_functor_grad shp_fun_grad(cost_param.r_safe);

    // costs 
    prior_and_obstacle_costs(0) = (0.5*x.transpose()*A*x + b.transpose()*x)(0,0);   
    prior_and_obstacle_costs(1) = distance_set.unaryExpr(std::ref(shp_fun)).sum(); // TODO - should apply smoothing function
    
    // cout<<"cost from vox:"<<endl;
    // cout<<prior_and_obstacle_costs(1)<<endl;
    // cout<<"cost from FDM"<<endl;
    // cout<<cost_obstacle(x)<<endl;

    
    // grad 
    grad.prior_gard = (A*x + b);    
    VectorXd dc_dd = distance_set.unaryExpr(std::ref(shp_fun_grad)); 
//    cout<<batch_grad_ref<<endl;
    Matrix3Xd batch_grad_shp = batch_grad_ref*dc_dd.asDiagonal();     //chain rule
//    cout<<batch_grad_shp<<endl; 
    Matrix2Xd grad_xy = batch_grad_shp.block(0,0,2,H);
    grad.nonlinear_grad = (Map<VectorXd>(grad_xy.data(), grad_xy.cols()*grad_xy.rows()));
    // cout<<"grad from vox:"<<endl;
    // cout<<grad.nonlinear_grad<<endl;
    // cout<<"grad from FDM"<<endl;
    // cout<<grad_cost_obstacle(x)<<endl;
    
    return distance_set;
}



