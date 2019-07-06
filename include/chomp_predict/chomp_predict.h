
#include <chomp_predict/chomp_ros_wrapper.h>
#include <queue>
#include <list>

const double V_MAX = 1.5; // the speed of the target is bounded 
const double BIG_T = 1e+5; // big big time 
const double REACH_TOL = 1e-1; // if the target is observed to be located within this range, it is seen to reach the goal


namespace CHOMP{

    struct PredictParam{
        int No; // observation queue size 
        int Np_max; // maximum number of prediction points 
        int Np_min; // minimum number of prediction points
        double observation_horizon; // time window for observation -> sampling dt = horizon/NO
        double prediction_horizon; // time window for prediction to be evaluted 
        double trigger_tol_accum_error; // accumulation error for prediction trigger 
        double observation_callback_duration(){
            return (observation_horizon/No);
        }    
    };


    // this is the final result of prediction 
    struct PredictionTrajectory{
        
        // member 
        VectorXd time_seq; 
        MatrixXd pred_path; 

        PredictionTrajectory() {};

        /**
         * @brief Construct a new Prediction Trajectory object. Also complete a prediction traj directly from observation traj and future path predicton
         * 
         * @param obsrv_time_seq : No x 1 (No < N) 
         * @param path : N x 3 where (0:No-1) x 3 corresponds to observation  
         */

        PredictionTrajectory(VectorXd obsrv_time_seq,MatrixXd path){
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
            cout <<"preidction traj: t/x/y/z"<<endl;
            cout<< result <<endl;  
        
        } 
        /**
         * @brief evalute the trajectory at time 
         * 
         * @param t : ros::Time::now - init_time
         * @return Vector3d 
         */

        Vector3d eval_at_time(double t){
            Vector3d output;
            VectorXd x_path = pred_path.block(0,0,pred_path.rows(),1);
            VectorXd y_path = pred_path.block(0,1,pred_path.rows(),1);
            VectorXd z_path = pred_path.block(0,2,pred_path.rows(),1);
            output(0) = interpolate(time_seq,x_path,t,true);
            output(1) = interpolate(time_seq,y_path,t,true);
            output(2) = interpolate(time_seq,z_path,t,true);            
            return output;
        }
        
    
    };

    class ChompForecaster{

        private:               
            bool map_mode; // 0 : offline , 1: online build 
            ros::NodeHandle nh;
            PredictParam pred_param; // prediction parameter 

            ros::Publisher pub_path_prediction; // pub for path msg (path = prediction during a horizon)
            ros::Publisher pub_marker_waypoints; // pub for marker  
            ros::Subscriber sub_pose_target; // sub for poseStamped msgs for target state

            // observation 
            list<geometry_msgs::PoseStamped>  observation_queue; // observation queue
            queue<geometry_msgs::Point> target_waypoints; // waypoints sequence. reached waypoints are deleted. 
            visualization_msgs::Marker marker_target_waypoints; // visualization marker 

            // routine 
            ros::Time last_callback_time;
            ros::Time last_prediction_time; // check point for prediction 
            ros::Time init_time; // initial time when program started 

            // sub solver  
            Wrapper chomp_wrapper;  
            PredictionTrajectory prediction_traj;                        

            // sub modules 
            VectorXd get_observation_time_stamps(); // observation time stamps 
            void to_next_target_waypoint(); // change the target waypoint to the next one 
            double last_obsrv_to_goal(); // distance to goal from the last observation 
            nav_msgs::Path windowed_prediction_traj(ros::Time); // from now to PredictParam::prediction_horizon

        public:
            ChompForecaster(); // constructor  
            void callback_target_state(geometry_msgs::PoseStampedConstPtr pose_stamped_ptr); // target callback from observer
            void predict_with_obsrv_queue(); // obtain with current observation queue set                                  
            geometry_msgs::Point eval_prediction(ros::Time eval_time); // evaluate preidction at eval_time 
            void publish_routine(); // publish routine  

            // flags 
            bool is_state_received;
            bool is_predicted; 
            void run(); 
    };
}