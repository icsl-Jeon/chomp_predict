#include <chomp_predict/chomp_predict.h>

using namespace CHOMP;


/**
 * @brief Construct a new Prediction Trajectory object. Also complete a prediction traj directly from observation traj and future path predicton
 * 
 * @param obsrv_time_seq : No x 1 (No < N) 
 * @param path : N x 3 where (0:No-1) x 3 corresponds to observation  
 */
PredictionTrajectory::PredictionTrajectory(VectorXd obsrv_time_seq,MatrixXd path){
    // cout<<"[CHOMP] loaded path: "<<endl;
    // cout<<path<<endl;
    int No = obsrv_time_seq.size();
    int N = path.rows();
    
    time_seq = VectorXd(N);
    pred_path = path; 

    // first, we should obtain average speed in observation             
    // std::cout<<"[CHOMP] The number of observation: "<<No<<endl;
    
    double avg_speed;
    double tot_trans = 0;
    // observation
    int n = 0; 
    time_seq(n) = obsrv_time_seq(n); n++;
    
    for( ; n<No;n++){
        tot_trans += (path.block(n-1,0,1,3) - path.block(n,0,1,3)).norm(); 
        time_seq(n) = obsrv_time_seq(n); // plain insertion 
    }
                
    avg_speed = tot_trans/(obsrv_time_seq(No-1) - obsrv_time_seq(0));               
    

    // allocate time for each point
    if (avg_speed == 0) // stop is observed for a time 
        // prediction 
        for(; n<N;n++)
            time_seq(n) = time_seq(n-1) + BIG_T;                 
    else
        for(; n<N;n++)
            time_seq(n) = time_seq(n-1) + (path.block(n-1,0,1,3) - path.block(n,0,1,3)).norm()/avg_speed; 

    // print the result 
    MatrixXd result(N,4);
    result << time_seq , path;
    // cout <<"preidction traj: t/x/y/z"<<endl;
    // cout<< result <<endl;  

} 

/**
 * @brief evalute the trajectory at time 
 * 
 * @param t : ros::Time::now - init_time
 * @return Vector3d 
 */
Vector3d PredictionTrajectory::eval_at_time(double t){
    Vector3d output;
    VectorXd x_path = pred_path.block(0,0,pred_path.rows(),1);
    VectorXd y_path = pred_path.block(0,1,pred_path.rows(),1);
    VectorXd z_path = pred_path.block(0,2,pred_path.rows(),1);



    output(0) = interpolate(time_seq,x_path,t,false);
    output(1) = interpolate(time_seq,y_path,t,false);
    output(2) = interpolate(time_seq,z_path,t,false);            
    return output;
}

ChompForecaster::ChompForecaster():nh("~"),chomp_wrapper(nh),is_predicted(false),accum_error(0){
    
    vector<double> target_waypoint_x;
    vector<double> target_waypoint_y;    

    this->is_state_received = false;

    // parameter parsing 
    nh.param("pred_param/observation_size",pred_param.No,5);
    nh.param("pred_param/prediction_size_max",pred_param.Np_max,12);
    nh.param("pred_param/prediction_size_min",pred_param.Np_min,5);
    nh.param("pred_param/prediction_horizon",pred_param.prediction_horizon,4.0);
    nh.param("pred_param/observation_horizon",pred_param.observation_horizon,2.0);
    nh.param("pred_param/trigger_tol_accum_error",pred_param.trigger_tol_accum_error,2.0);    
    nh.param<string>("world_frame_id",world_frame_id,"/world");

    nh.param<string>("cal_time_log_dir",cal_time_write_dir,"/home/jbs/");
    nh.param("is_log",is_log,false);

    nh.getParam("target_waypoint/x",target_waypoint_x);
    nh.getParam("target_waypoint/y",target_waypoint_y);

    nh.param("is_pose",is_pose,true);

    // if no param available, this target waypoints are for map3.vxblx 
    if (not nh.hasParam("target_waypoint/x")){
        
        target_waypoint_x.push_back(3.4578);
        target_waypoint_y.push_back(4.1620);

        target_waypoint_x.push_back(6.7478);
        target_waypoint_y.push_back(8.0200);

        target_waypoint_x.push_back(8.3223);
        target_waypoint_y.push_back(3.9896);

        target_waypoint_x.push_back(11.4981);
        target_waypoint_y.push_back(8.0515);

    }
    
    //load target waypoints(via-points) 
    printf("[CHOMP] upload target waypoints");
    if(target_waypoint_x.size() == 0){
        cerr<<"[CHOMP] exit program. No target waypoint is found"<<endl;    
        exit(-1);
    }
    for (int n = 0; n<target_waypoint_x.size();n++){
        printf("[CHOMP] %dth target waypoints: [%f, %f]\n",n+1,target_waypoint_x[n],target_waypoint_y[n]);
        geometry_msgs::Point point;
        point.x = target_waypoint_x[n];
        point.y = target_waypoint_y[n];
        target_waypoints.push(point);                
    }


    // register advertise and subscriber 
    pub_path_prediction_traj = nh.advertise<nav_msgs::Path>("prediction_traj",1);
    pub_marker_waypoints = nh.advertise<visualization_msgs::Marker>("predictor/target_waypoints",1); 
    pub_pose_cur_pred = nh.advertise<geometry_msgs::PoseStamped>("predictor/pred_point_at_cur_time",1); 

    if (is_pose)
        sub_pose_target = nh.subscribe("/target_pose",2,&ChompForecaster::callback_target_state,this);
    else
        sub_point_target = nh.subscribe("/target_point",2,&ChompForecaster::callback_target_point,this);  
    
    // map mode selection 
    string file_name;
    nh.param<string>("map_file_name",file_name,"/home/jbs/catkin_ws/chomp_predict/worlds/map3.vxblx");

    if(file_name.substr(file_name.find_last_of(".")+1)=="bt"){
        std::cout << "Provided octomap file: "<<file_name<< std::endl;
        octomap::OcTree* tree = new octomap::OcTree(file_name);
        this->chomp_wrapper.load_map(tree);  
        this->chomp_wrapper.map_type = 0;   
    }
    else if(file_name.substr(file_name.find_last_of(".")+1)=="vxblx"){
        std::cout << "Provided voxblox file: "<<file_name<< std::endl;
        this->chomp_wrapper.load_map(file_name); // voxblox mode 
        this->chomp_wrapper.map_type = 1;
    } // vxblx 
    else{
        std::cerr << "Usage: " << "NODE_NAME" << " OCTOMAP_FILE_NAME(.bt) or VOXBLOX_FILE_NAME(.vxblx)" << std::endl;
        exit(0);
    }        


    
    /**
     * @todo currently, the following line will replace the mode selection phase
     */

    // // map load 
    // if (nh.hasParam("map_file_name")){ // offline map load mode         
    //     string file_name;
    //     nh.param<string>("map_file_name",file_name,"/home/jbs/catkin_ws/chomp_predict/worlds/map3.vxblx");
    //     this->chomp_wrapper.load_map(file_name); // voxblox mode 
    //     this->chomp_wrapper.map_type = 1; // vxblx 
    // }
    // else{
    //     // future 
    //     cerr<<"No map recieved. Currently, only offline map load mode is supported."<<endl;
    // }


    // time initialization 
    last_callback_time = ros::Time::now(); // set timer 
    init_time = ros::Time::now(); // initial time 
    cout<<"[CHOMP] time is initialized with: "<<init_time.toSec()<<endl;
    
    // marker init 
    marker_target_waypoints.header.frame_id = world_frame_id;
    float scale = 0.3;
    marker_target_waypoints.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_target_waypoints.scale.x = scale;
    marker_target_waypoints.scale.y = scale;
    marker_target_waypoints.scale.z = scale;
    marker_target_waypoints.color.g = 1;
    marker_target_waypoints.color.a = 1.0;
    
    for (int n =0; n<target_waypoint_x.size();n++){
        geometry_msgs::Point position;
        position.x = target_waypoint_x[n];
        position.y = target_waypoint_y[n];
        position.z = 1.2; // hover in the air 
        marker_target_waypoints.points.push_back(position);
    }

};



// extract only times from observations
VectorXd ChompForecaster::get_observation_time_stamps(){    
    if (this->is_state_received){
        int n_step = observation_queue.size();
        Eigen::VectorXd time_stamps(n_step);
        int n = 0;
        for(auto it = observation_queue.begin(); it != observation_queue.end(); it++,n++)
            time_stamps[n] = (it->header.stamp-init_time).toSec();
        return time_stamps;
    }
    else{
        cerr<<"[CHOMP] No observation. but time extraction was requested."<<endl;
         return VectorXd(); 
    }   
}

void ChompForecaster::to_next_target_waypoint(){
    if (~target_waypoints.empty())
        target_waypoints.pop();
    else
        cout<<"[CHOMP] all waypoints have been passed. Target will not move anymore"<<endl;   
}

double ChompForecaster::last_obsrv_to_goal(){
    // Check target goal reach
    geometry_msgs::Point most_recent_target_position; 
    most_recent_target_position = observation_queue.back().pose.position;  
    return  sqrt(pow(most_recent_target_position.x - target_waypoints.front().x,2) + 
                              pow(most_recent_target_position.y - target_waypoints.front().y,2));
}

nav_msgs::Path ChompForecaster::windowed_prediction_traj_eval(ros::Time time_now){

    double eval_t0 = (time_now - init_time).toSec(); 
    double eval_tf = eval_t0 + pred_param.prediction_horizon; 
    int N_step = 20;
    VectorXd t_eval_set = VectorXd::LinSpaced(N_step,eval_t0,eval_tf);

    nav_msgs::Path evaluated_prediction_traj;
    evaluated_prediction_traj.header.frame_id = world_frame_id;
    for (int n=0;n<N_step;n++){
        double t_cur = t_eval_set(n);
        geometry_msgs::PoseStamped pose_stamped;
        Vector3d pnt_eval = prediction_traj.eval_at_time(t_cur);        
        pose_stamped.pose.position.x = pnt_eval(0);
        pose_stamped.pose.position.y = pnt_eval(1);
        pose_stamped.pose.position.z = chomp_wrapper.get_ground_height();
        evaluated_prediction_traj.poses.push_back(pose_stamped);
    }
    return evaluated_prediction_traj;
}


// update observation at every specified duration 
void ChompForecaster::callback_target_state(geometry_msgs::PoseStampedConstPtr pose_stamped_ptr){
        
    if(((ros::Time::now() - last_callback_time).toSec())
           > pred_param.observation_callback_duration()){  
          
        if( observation_queue.size() < pred_param.No) {// not full
            geometry_msgs::PoseStamped pose_stamped = *pose_stamped_ptr;
            pose_stamped.header.stamp = ros::Time::now(); 
            observation_queue.push_back(pose_stamped);
            is_state_received = true;
        }
        else{
            //full
            observation_queue.pop_front();
            geometry_msgs::PoseStamped pose_stamped = *pose_stamped_ptr;
            pose_stamped.header.stamp = ros::Time::now(); 
            observation_queue.push_back(pose_stamped);
        }     
        last_callback_time = ros::Time::now();
    }
    // else, do not update
}

// update observation at every specified duration 
void ChompForecaster::callback_target_point(geometry_msgs::PointStampedConstPtr point_stamped_ptr){
        
    if(((ros::Time::now() - last_callback_time).toSec())
           > pred_param.observation_callback_duration()){  
          
        if( observation_queue.size() < pred_param.No) {// not full
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.pose.position = (point_stamped_ptr->point);
            pose_stamped.header.frame_id = world_frame_id;
            pose_stamped.header.stamp = ros::Time::now(); 
            observation_queue.push_back(pose_stamped);
            is_state_received = true;
        }
        else{
            //full
            observation_queue.pop_front();
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.pose.position = point_stamped_ptr->point;
            pose_stamped.header.frame_id = world_frame_id;
            pose_stamped.header.stamp = ros::Time::now(); 
            observation_queue.push_back(pose_stamped);
        }     
        last_callback_time = ros::Time::now();
    }
    // else, do not update
}


// predict routine 
void ChompForecaster::predict_with_obsrv_queue(){

    if (is_state_received){
        // load prior path(observations to be fitted) and sub goal of target (waypoints)
        nav_msgs::Path prior_path;         
        for(auto it = this->observation_queue.begin();it != this->observation_queue.end();it++)
            prior_path.poses.push_back(*it);

        geometry_msgs::Point g = this->target_waypoints.front();
        
        // update markers
        chomp_wrapper.load_markers_prior_pnts(prior_path,g);

        // 2. Solve optimization 
        MatrixXd M; VectorXd h;
        OptimParam optim_param = chomp_wrapper.get_default_optim_param();   
        // decide N              
        
        chomp_wrapper.build_matrix(M,h,prior_path,g,&optim_param); // A,b matrix 
        VectorXd x0 = chomp_wrapper.prepare_chomp(M,h,prior_path,g); // initial guess 
        ros::Time tic = ros::Time::now();
        if (~is_predicted or is_goal_moved or chomp_wrapper.recent_optim_result.distance_min < 0){
            cout<<"[CHOMP] use interp for start of optim"<<endl;
            chomp_wrapper.solve_chomp(x0); // optimization from the initial guess 
        }
        else{
            cout<<"[CHOMP] reuse previous sol for start of optim"<<endl;            
            chomp_wrapper.solve_chomp(VectorXd(0));
        }
        cout<<"optimizaton solved at "<<(ros::Time::now() - tic).toSec()<<" [sec]"<<endl;
        
        // file write
        if (is_log){
            std::ofstream wnpt_file;
            wnpt_file.open((cal_time_write_dir+"/chomp_compute_time.txt").c_str(),ios_base::app);

            if(wnpt_file.is_open()){
                wnpt_file<<(ros::Time::now() - tic).toNSec()<<"\n";
                wnpt_file.close();    
            }else
                cout<<"logging file for compute time is not opend"<<endl;
        }
        // 3. TIme allocation and finishing  
        MatrixXd current_prediction = chomp_wrapper.get_current_prediction_path(); 
        VectorXd observation_time_stamp = get_observation_time_stamps();                                                     
        prediction_traj=PredictionTrajectory(observation_time_stamp,current_prediction); // the final result         
        is_predicted = true;
    
    }
    else{
        cout<<"still no observation"<<endl;
        is_predicted = false;
    }

}
/**
 * @brief publish routine in while loop  
 * @details / 2. observation queue / 3.prediction path during prediction horizon (not entire path)
 */
void ChompForecaster::publish_routine(){

    
    pub_marker_waypoints.publish(marker_target_waypoints);    
    if(is_predicted){
        nav_msgs::Path pred_traj = windowed_prediction_traj_eval(ros::Time::now());
        pub_path_prediction_traj.publish(pred_traj);
        geometry_msgs::PoseStamped cur_pred_point = *pred_traj.poses.begin();
        cur_pred_point.header.frame_id = world_frame_id;
        cur_pred_point.header.stamp = ros::Time::now();
        pub_pose_cur_pred.publish(cur_pred_point);
    }   
}

/**
 * @brief code to be run in the main loop 
 * 
 */
void ChompForecaster::run(){

    int loop_fps = 20;
    ros::Rate rate(loop_fps);
    // in intial phase, prediction will be triggered only if there is a observation 

    bool trigger_prediction = true; 

    double accum_error = 0;
    double duration_from_last_trigger;
    ROS_INFO_STREAM("[CHOMP] waiting target observation. We will start if target history is enough(>= No)");
    while(ros::ok()){        
        if(is_state_received and observation_queue.size() >= pred_param.No){
            ROS_INFO_ONCE("[CHOMP] enough target observation received. Start loop");
                    
            // prediction tirgger 
            if(trigger_prediction){            
                predict_with_obsrv_queue();
                // reset 
                last_prediction_time = ros::Time::now();
                accum_error = 0;
            } 

            // check whether target reaches the imminent waypoint. If reached, move to the next one. 
            if (last_obsrv_to_goal() < REACH_TOL and (target_waypoints.size()>1))
                {to_next_target_waypoint(); is_goal_moved = true;}
            else 
                is_goal_moved = false;
            
            
            // Check the trigger condition for the next prediction 
            duration_from_last_trigger = (ros::Time::now() - last_prediction_time).toSec(); 
            accum_error += 1.0/loop_fps * (pow(eval_prediction(ros::Time::now()).x - observation_queue.back().pose.position.x,2) 
                                        + pow(eval_prediction(ros::Time::now()).y - observation_queue.back().pose.position.y,2));  // the MSE integration
            //cout << "accumulated error: " <<accum_error << endl;
            trigger_prediction = (duration_from_last_trigger > pred_param.prediction_horizon - 0.01 ) or 
                                (accum_error > pred_param.trigger_tol_accum_error );

        }


        // regular routine 
        ros::spinOnce();
        rate.sleep();            
        // publish routine 
        this->chomp_wrapper.publish_routine(); //inner loop(chomp_ros_wrapper) publisher 
        this->publish_routine(); // outer loop(chomp_predictor) publisher 
    }
}

bool ChompForecaster::session(){

    bool trigger_condition;   
    double duration_from_last_trigger;
    double dt = (ros::Time::now() - last_session_time).toSec();

    if (get_predict_condition()){
        ROS_INFO_ONCE("[CHOMP] received enough observation. start prediction");
        // this is the first time to predict  
        if(not is_predicted)
            trigger_condition = true;
        // not first time
        else{
            // Check the trigger condition for the next prediction                     
            duration_from_last_trigger = (ros::Time::now() - last_prediction_time).toSec(); 
            accum_error += dt * (pow(eval_prediction(ros::Time::now()).x - observation_queue.back().pose.position.x,2) 
                                        + pow(eval_prediction(ros::Time::now()).y - observation_queue.back().pose.position.y,2));  // the MSE integration
			//cout << "[Predictor] accum error: "<<accum_error <<endl; 
            trigger_condition = (duration_from_last_trigger > pred_param.prediction_horizon - 0.01 ) or 
                                (accum_error > pred_param.trigger_tol_accum_error );
        }

    // if trigger condition, then predict 
    if(trigger_condition){
        predict_with_obsrv_queue();
        // reset 
        last_prediction_time = ros::Time::now();
        accum_error = 0;
    }

    // check whether target reaches the imminent waypoint. If reached, move to the next one. 
    if (last_obsrv_to_goal() < REACH_TOL and (target_waypoints.size()>1)){
        to_next_target_waypoint(); is_goal_moved = true;
    }
    else 
        is_goal_moved = false;
    
        
    // publish routine 
    this->chomp_wrapper.publish_routine(); //inner loop(chomp_ros_wrapper) publisher 
    this->publish_routine(); // outer loop(chomp_predictor) publisher 


    }
    else
        ROS_INFO_ONCE("[CHOMP] wating target state callback... ");

    last_session_time = ros::Time::now();
	return trigger_condition;
}
/**
 * 
 * @brief Once prediction model is acquired, then we can evaluate the prediction in time 
 * @param eval_time 
 * @return geometry_msgs::Point 
 */
geometry_msgs::Point ChompForecaster::eval_prediction(ros::Time eval_time){    
    // time 
    double t_eval_double = eval_time.toSec();
    Vector3d output = prediction_traj.eval_at_time(t_eval_double-init_time.toSec());
    geometry_msgs::Point p; 
    p.x = output(0);
    p.y = output(1); 
    p.z = output(2);
    return p;
}

bool ChompForecaster::get_predict_condition(){
    return (is_state_received and observation_queue.size() >= pred_param.No); 
}



