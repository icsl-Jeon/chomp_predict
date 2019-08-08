
#include <chomp_predict/chomp_ros_wrapper.h>
#include <queue>
#include <list>

const double V_MAX = 1.5; // the speed of the target is bounded 
const double BIG_T = 1e+5; // big big time 
const double REACH_TOL = 0.5; // if the target is observed to be located within this range, it is seen to reach the goal


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
        PredictionTrajectory(VectorXd obsrv_time_seq,MatrixXd path);
        Vector3d eval_at_time(double t);    
    };

    class ChompForecaster{

        private:               
            bool map_mode; // 0 : offline , 1: online build 
            double accum_error;
            ros::NodeHandle nh;
            PredictParam pred_param; // prediction parameter 
            string world_frame_id;
            
            ros::Publisher pub_path_prediction_traj; // pub for path msg (path = prediction during a horizon)
            ros::Publisher pub_marker_waypoints; // pub for marker  
            ros::Subscriber sub_pose_target; // sub for poseStamped msgs for target state

            // observation 
            list<geometry_msgs::PoseStamped>  observation_queue; // observation queue
            queue<geometry_msgs::Point> target_waypoints; // waypoints sequence. reached waypoints are deleted. 
            visualization_msgs::Marker marker_target_waypoints; // visualization marker 

            // routine 
            ros::Time last_callback_time;
            ros::Time last_session_time;
            ros::Time last_prediction_time; // check point for prediction 
            ros::Time init_time; // initial time when program started 

            // sub solver  
            Wrapper chomp_wrapper;  
            PredictionTrajectory prediction_traj;                        

            // sub modules 
            VectorXd get_observation_time_stamps(); // observation time stamps 
            void to_next_target_waypoint(); // change the target waypoint to the next one 
            double last_obsrv_to_goal(); // distance to goal from the last observation 
            nav_msgs::Path windowed_prediction_traj_eval(ros::Time); // from now to PredictParam::prediction_horizon
            
            bool is_goal_moved =false;

        public:
            ChompForecaster(); // constructor  
            void callback_target_state(geometry_msgs::PoseStampedConstPtr pose_stamped_ptr); // target callback from observer
            void predict_with_obsrv_queue(); // obtain with current observation queue set                                  
            geometry_msgs::Point eval_prediction(ros::Time eval_time); // evaluate preidction at eval_time 
            void publish_routine(); // publish routine  
            bool get_predict_condition(); // is it ready to predict? 
            // flags 
            bool is_state_received;
            bool is_predicted; 
            void run();
            void session(); // one session in a loop 
    };
}