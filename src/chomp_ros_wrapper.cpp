#include "chomp_predict/chomp_ros_wrapper.h"

using namespace CHOMP;

Wrapper::Wrapper(const ros::NodeHandle& nh_global):nh("~"){    
    // parameter parsing 
    nh.param("cost_param/r_safe",r_safe,3.4);
    nh.param("cost_param/ground_reject_height",ground_rejection_height,0.5);

    nh.param("optim_param/descending_rate",optim_param_default.descending_rate,0.1);
    nh.param("optim_param/descending_rate_min",optim_param_default.descending_rate_min,0.1);
    nh.param("optim_param/descending_rate_max",optim_param_default.descending_rate_max,0.1);

    nh.param("optim_param/max_iter",optim_param_default.max_iter,300);
    nh.param("optim_param/weight_prior",optim_param_default.weight_prior,1e-2);
    nh.param("optim_param/term_cond",optim_param_default.termination_cond,1e-2);
    nh.param("optim_param/gamma",optim_param_default.gamma,0.4);
    nh.param("optim_param/n_step",optim_param_default.n_step,10);


    nh.param<string>("world_frame_id",world_frame_id,"/world");    
    pub_path_cur_solution = nh.advertise<nav_msgs::Path>("predictor/chomp_solution_path",1);
    pub_vis_goal= nh.advertise<visualization_msgs::Marker>("predictor/chomp_goal",1);
    pub_vis_observations = nh.advertise<visualization_msgs::Marker>("predictor/chomp_obsrv",1);
    pub_marker_pnts_path = nh.advertise<visualization_msgs::Marker>("predictor/chomp_sol_pnts",1);

    nh.param("is_voxblox_pub",is_voxblox_pub,true);


    pnts_on_path_marker.action = 0;
    pnts_on_path_marker.header.frame_id = world_frame_id;
    pnts_on_path_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    double scale = 0.16;
    pnts_on_path_marker.scale.x = scale;
    pnts_on_path_marker.scale.y = scale;
    pnts_on_path_marker.scale.z = scale;
    pnts_on_path_marker.color.b = 0.8;
    pnts_on_path_marker.color.g = 1.;    
    pnts_on_path_marker.color.a = 0.8;
};


// update markers for prior_pnts 
void Wrapper::load_markers_prior_pnts(nav_msgs::Path prior_path,geometry_msgs::Point goal){
        
    obsrv_markers.action = 0;
    obsrv_markers.header.frame_id = world_frame_id;
    obsrv_markers.type = visualization_msgs::Marker::SPHERE_LIST;
    float scale = 0.16;
    obsrv_markers.scale.x = scale;
    obsrv_markers.scale.y = scale;
    obsrv_markers.scale.z = scale;
    obsrv_markers.color.a = 0.8;
    obsrv_markers.points.clear();
        for (int n =0;n<prior_path.poses.size();n++){
        prior_path.poses[n].pose.position.z = ground_rejection_height;
        obsrv_markers.points.push_back(prior_path.poses[n].pose.position);
    }
    goal_marker.action = 0;
    goal_marker.type = visualization_msgs::Marker::SPHERE;
    goal_marker.header.frame_id = world_frame_id;
    goal_marker.pose.position = goal;
    goal_marker.pose.position.z = ground_rejection_height; // hegith is arbitrarily
    goal_marker.scale.x = scale;
    goal_marker.scale.y = scale;
    goal_marker.scale.z = scale;
    goal_marker.color.r = 1; // current gaol 
    goal_marker.color.a = 0.8;
}

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

void Wrapper::load_map(string file_name){    
    // we will create voxblox server only if it is voxblox mode. It talks too much.
    ros::NodeHandle nh_global;
    voxblox_server = new voxblox::EsdfServer (nh_global,nh);    
    // EDT map scale = octomap  
    voxblox_server->loadMap(file_name);
    dx = voxblox_server->getEsdfMapPtr()->voxel_size(); // voxel_size 
    cout<<"resolution of ESDF : "<<dx<<"/ block size: "<<voxblox_server->getEsdfMapPtr()->block_size()<<endl;
    is_map_load = true;
};

/**
 * @brief build prior term from prior points (start_pnt) of length No+1(No observations and one final point) 
 * to make 1/2x'Mx+hx
 * @param M insert matrix to M 
 * @param h insert matrix to h 
 * @param prior_points points of length No<=N
 * @param gamma weight factor for prior points 
 * @param goal goal point 
 * @param N total time index
 */
void Wrapper::build_matrix(MatrixXd &M,VectorXd &h,nav_msgs::Path prior_points,geometry_msgs::Point goal,OptimParam* param){
        
    // init     
    // if external parameter is given 
    if(param == NULL)
        optim_param = optim_param_default;
    else
        optim_param = *param;

    // here, A,b is |Ax-b|^2
    int N = optim_param.n_step; 
    int No = prior_points.poses.size();
    int m_row = (No + (N-1) + (N-2) + 1) * dim;  // prior points + velocity + acceleration + goal

    MatrixXd A = MatrixXd(m_row,dim*N);
    VectorXd b = VectorXd(m_row);
    double gamma = optim_param.gamma;
    
    
    MatrixXd A0 = MatrixXd::Zero(dim*(No+1),dim*N);
    VectorXd b0 = VectorXd::Zero(dim*(No+1));

    // 1. prior points + goal fitting 
    for(int n = 0;n<No;n++){
        double factor = n/N;
        geometry_msgs::Point p = prior_points.poses[n].pose.position;            
        A0.block(dim*n,dim*n,dim,dim) = (sqrt(exp(gamma*factor)) * VectorXd::Ones(dim)).asDiagonal();        
        b0.block(dim*n,0,dim,1) << sqrt(exp(gamma*factor))*p.x, 
                                   sqrt(exp(gamma*factor))*p.y;                                                    
    }
    
    A0.block(dim*No,dim*(N-1),dim,dim) =  (sqrt(exp(gamma*1)) * VectorXd::Ones(dim)).asDiagonal();     
    b0.block(dim*No,0,dim,1) << sqrt(exp(gamma*1))*goal.x, 
                                sqrt(exp(gamma*1))*goal.y;       

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


    M = MatrixXd(dim*N,dim*N);
    h = VectorXd(dim*N);
    
    M = 2*A.transpose()*A;
    h = -2*A.transpose()*b;
}


// Prepare chomp by setting cost matrix and fitting points. Also, it outputs initial guess    
VectorXd Wrapper::prepare_chomp(MatrixXd M,VectorXd h,nav_msgs::Path prior_path,geometry_msgs::Point goal,OptimParam* param){

    // if external parameter is given 
    if(param == NULL)
        optim_param = optim_param_default;
    else
        optim_param = *param;
        
    
    if (M.rows() == h.size()){
        
        int N = M.rows()/dim;
        int No = prior_path.poses.size();

        CostParam cost_param;
        cost_param.dx = dx;
        cost_param.ground_height = ground_rejection_height;
        cost_param.r_safe = r_safe;

        // complete problem with obstacle functions 
        if (map_type == 0) // octomap 
            solver.set_problem(M,h,this->edf_ptr,cost_param);
        else
            solver.set_problem(M,h,(this->voxblox_server),cost_param);
        
        // Intiial guess generation 
        VectorXd ts = VectorXd::LinSpaced(N,0,1);   
        VectorXd t_regress(No+1);
        t_regress << ts.block(0,0,No,1) , 1; // regression target : prior point and goal 

        vector<double> xs_regress,ys_regress,zs_regress;
        path2vec(prior_path,xs_regress,ys_regress,zs_regress);
        xs_regress.push_back(goal.x);
        ys_regress.push_back(goal.y);
        zs_regress.push_back(goal.z);
                
        LinearModel initial_guess_model_x =  linear_regression(t_regress,Map<VectorXd>(xs_regress.data(),No+1));                                        
        LinearModel initial_guess_model_y =  linear_regression(t_regress,Map<VectorXd>(ys_regress.data(),No+1));                                        
        LinearModel initial_guess_model_z =  linear_regression(t_regress,Map<VectorXd>(zs_regress.data(),No+1)); // will not be used in 2D case                                        

        VectorXd x0(dim*N);
        for (int n=0;n<N;n++){
            x0(dim*n) = model_eval(initial_guess_model_x,ts(n)); 
            x0(dim*n+1) = model_eval(initial_guess_model_y,ts(n));
        }

        return x0;
    }
    else{
        cerr<<"[CHOMP] dimension error for optimizatoin returning zero-filled initial guess."<<endl;
        return VectorXd::Zero(M.rows());
    }
}

// start chomp routine and save the solution in path 
bool Wrapper::solve_chomp(VectorXd x0){

    if (x0.size())
        if (map_type == 0)
            recent_optim_result = solver.solve(x0,optim_param);    
        else 
            recent_optim_result = solver.solve2(x0,optim_param);
    else
        if (map_type == 0)
            recent_optim_result = solver.solve(recent_optim_result.solution,optim_param);    
        else
            recent_optim_result = solver.solve2(recent_optim_result.solution,optim_param);
    
    // if solved, 
    VectorXd x_sol = recent_optim_result.solution;
    nav_msgs::Path path;
    path.header.frame_id = world_frame_id;
    int H = x_sol.size()/2; // dim = 2 assumed
    // extract path from solution x_sol and let's make corresponding points along a path also
    pnts_on_path_marker.points.clear();
    for (int h = 0; h<H;h++){

        double x = x_sol(h*2);
        double y = x_sol(h*2+1);
        double z = ground_rejection_height; // the height  of the path  
        
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = world_frame_id;
        pose_stamped.pose.position.x = x;
        pose_stamped.pose.position.y = y; 
        pose_stamped.pose.position.z = z; 
        
        path.poses.push_back(pose_stamped);
        pnts_on_path_marker.points.push_back(pose_stamped.pose.position);
    }    
    current_path = path;
    std::cout<<"[CHOMP] path uploaded"<<std::endl;
}

void Wrapper::publish_routine(){
    // solution path publish 
    pub_path_cur_solution.publish(current_path);

    // marker publish 
    pub_vis_goal.publish(goal_marker);
    pub_vis_observations.publish(obsrv_markers);
    pub_marker_pnts_path.publish(pnts_on_path_marker);

    // esdf publish 
    if(this->map_type == 1 and is_voxblox_pub){
        this->voxblox_server->setSliceLevel(ground_rejection_height);
        this->voxblox_server->publishSlices();
        this->voxblox_server->publishPointclouds();
        this->voxblox_server->publishTsdfSurfacePoints();
    }
}

// return path. the returned path will be endowed with times. This function returns the solution path in the form of matrix  
MatrixXd Wrapper::get_current_prediction_path(){

    MatrixXd prediction_path; // N x 3 matrix 
    if(current_path.poses.size()){

        int N = current_path.poses.size(); // number of path  
        
        prediction_path  = MatrixXd(N,3);
        for (int n = 0; n<current_path.poses.size(); n++){
            prediction_path(n,0)  = current_path.poses[n].pose.position.x; 
            prediction_path(n,1)  = current_path.poses[n].pose.position.y; 
            prediction_path(n,2)  = current_path.poses[n].pose.position.z;         
        }

    }else    
        cerr<<"No prediction path loaded"<<endl;
        
    return prediction_path;     
}

// return default optim_param 
OptimParam Wrapper::get_default_optim_param(){

    return optim_param_default;
}
