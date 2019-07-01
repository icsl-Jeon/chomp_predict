#include "chomp_predict/chomp_ros_wrapper.h"

using namespace CHOMP;

Wrapper::Wrapper(){    
    nh.param("ground_rejection_height",ground_rejection_height,0.4);
    nh.param("r_safe",r_safe,3.4);
    nh.param<string>("world_frame_id",world_frame_id,"/world");    
    pub_path_cur_solution = nh.advertise<nav_msgs::Path>("chomp_solution_path",1);
};

// load EDF map from octree. The octree is assumed to be provided from outside 
void Wrapper::load_map(octomap::OcTree* octree_ptr){    
    // EDT map scale = octomap  
    double x,y,z;
    octree_ptr->getMetricMin(x,y,z);
    octomap::point3d boundary_min(x,y,z); 
    boundary_min.z() =ground_rejection_height;
    octree_ptr->getMetricMax(x,y,z);
    octomap::point3d boundary_max(x,y,z); 
    dx = octree_ptr->getResolution();
    double edf_max_dist = r_safe;
    bool unknownAsOccupied = false;

    // EDF completed
    edf_ptr = new DynamicEDTOctomap(edf_max_dist,octree_ptr,
        boundary_min,
        boundary_max,unknownAsOccupied);
    edf_ptr->update();

    
    // flag 
    is_map_load = true;
};

/**
 * @brief build prior term from prior points (start_pnt) of length No+1(No observations and one final point) 
 * to make 1/2x'Ax+bx
 * @param A insert matrix to A 
 * @param b insert matrix to b 
 * @param prior_points points of length No<=N
 * @param gamma weight factor for prior points 
 * @param goal goal point 
 * @param N total time index
 */
void Wrapper::build_matrix(MatrixXd &A,VectorXd &b,nav_msgs::Path prior_points,double gamma,geometry_msgs::Point goal,int N){
        
    // init     
    int No = prior_points.poses.size();
    int m_row = (No + (N-1) + (N-2) + 1) * dim;  // prior points + velocity + acceleration + goal
    A = MatrixXd(m_row,dim*N);
    b = VectorXd(m_row);

    // init    
    MatrixXd A0 = MatrixXd::Zero(dim*(No+1),dim*N);
    VectorXd b0 = VectorXd::Zero(dim*(No+1));

    // 1. prior points + goal fitting 
    for(int n = 0;n<No;n++){
        double factor = n/N;
        geometry_msgs::Point p = prior_points.poses[n].pose.position;            
        A0.block(dim*n,dim*n,dim,dim) = (sqrt(gamma*exp(factor)) * VectorXd::Ones(dim)).asDiagonal();        
        b0.block(dim*n,0,dim,1) << sqrt(gamma*exp(factor))*p.x, 
                                   sqrt(gamma*exp(factor))*p.y;                                                    
    }
    
    A0.block(dim*No,dim*(N-1),dim,dim) =  (sqrt(gamma*exp(1)) * VectorXd::Ones(dim)).asDiagonal();     
    b0.block(dim*No,0,dim,1) << sqrt(gamma*exp(1))*goal.x, 
                                sqrt(gamma*exp(1))*goal.y;       

    // 2. 1st order derivatives  
    MatrixXd A1 = MatrixXd::Zero(dim*(N-1),dim*N);
    VectorXd b1= VectorXd::Zero(dim*(N-1));
    
    for(int n = 0;n<N-1;n++){
        A1.block((dim*n),(dim*n),dim,dim) = MatrixXd::Identity(dim,dim);        
        A1.block((dim*n),(dim*(n+1)),dim,dim) = -MatrixXd::Identity(dim,dim);            
    }

    // 3. 2nd order derivatives  
    MatrixXd A2= MatrixXd::Zero(dim*(N-2),dim*N);
    VectorXd b2= VectorXd::Zero(dim*(N-2));

    for(int n = 0;n<N-2;n++){
        A2.block((dim*n),(dim*n),dim,dim) = MatrixXd::Identity(dim,dim);        
        A2.block((dim*n),(dim*(n+1)),dim,dim) = -2*MatrixXd::Identity(dim,dim);            
        A2.block((dim*n),(dim*(n+2)),dim,dim) = MatrixXd::Identity(dim,dim);            
    }

    // matrix construct 
    A << A0,
         A1,
         A2;

    b << b0,
         b1,
         b2;
}


// Prepare chomp by setting cost matrix and fitting points. Also, it outputs initial guess    
VectorXd Wrapper::prepare_chomp(MatrixXd A,VectorXd b,nav_msgs::Path prior_path,geometry_msgs::Point goal){
    if (A.rows() == b.size()){
        
        int N = A.rows()/dim;
        int No = prior_path.poses.size();


        CostParam cost_param;
        cost_param.dx = dx;
        cost_param.ground_height = ground_rejection_height;
        cost_param.r_safe = r_safe;
        // complete problem with obstacle functions 
        solver.set_problem(A,b,this->edf_ptr,cost_param);

        // Intiial guess generation 
        VectorXd ts = VectorXd::LinSpaced(N,0,1);   
        VectorXd t_regress(No+1);
        t_regress << ts.block(0,0,No,1) , 1; // regression target : prior point and goal 

        vector<double> xs_regress,ys_regress,zs_regress;
        path2vec(prior_path,xs_regress,ys_regress,zs_regress);
        xs_regress.push_back(goal.x);
        ys_regress.push_back(goal.y);
        zs_regress.push_back(goal.z);
                
        LinearModel initial_guess_model_x =  linear_regression(t_regress,Map<VectorXd>(xs_regress.data(),N));                                        
        LinearModel initial_guess_model_y =  linear_regression(t_regress,Map<VectorXd>(ys_regress.data(),N));                                        
        LinearModel initial_guess_model_z =  linear_regression(t_regress,Map<VectorXd>(zs_regress.data(),N)); // will not be used in 2D case                                        

        VectorXd x0(dim*N);
        for (int n=0;n<N;n++){
            x0(dim*n) = model_eval(initial_guess_model_x,ts(n)); 
            x0(dim*n+1) = model_eval(initial_guess_model_y,ts(n));
        }

        return x0;
    }
    else{
        cerr<<"[CHOMP] dimension error for optimizatoin returning zero-filled initial guess."<<endl;
        return VectorXd::Zero(A.rows());
    }
}

// start chomp routine 
bool Wrapper::solve_chomp(VectorXd x0,OptimParam param){
    recent_optim_result = solver.solve(x0,param);    

    // if solved, 
    VectorXd x_sol = recent_optim_result.solution;
    nav_msgs::Path path;
    path.header.frame_id = world_frame_id;

    for (int h = 0; h<x_sol.size();h++){

        double x = x_sol(h*2);
        double y = x_sol(h*2+1);
        
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = world_frame_id;
        pose_stamped.pose.position.x = x;
        pose_stamped.pose.position.y = y; 
        path.poses.push_back(pose_stamped);
    }    

    std::cout<<"[CHOMP] path uploaded"<<std::endl;
}

void Wrapper::publish_routine(){
    pub_path_cur_solution.publish(current_path);
}
