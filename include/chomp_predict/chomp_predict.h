
#include <ros/ros.h>
#include <chomp_predict/chomp_subroutine.h>


namespace TargetPredict{
typedef nav_msgs::Path ObservationPath;

struct PredictParam{
    int No; // observation queue size 
    int Np_max; // maximum number of prediction points 
    int Np_min; // minimum number of prediction points
};

}
class chomp_predict{

    private:                
        ros::NodeHandle nh;
        ros::Publisher pub_path_prediction; // pub for path msg 
        ros::Subscriber sub_pose_target; // sub for poseStamped msgs for target state
        TargetPredict::ObservationPath observation_path; 
        DynamicEDTOctomap *edf_ptr; //  


    public: 
        void callback_target_state(geometry_msgs::PoseConstPtr pose_ptr);

        // flags 
        bool is_state_recieved;


};
