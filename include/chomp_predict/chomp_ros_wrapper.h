#include <ros/ros.h>
#include "chomp_predict/chomp_subroutine.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>


namespace CHOMP{


    class Wrapper{
        private: 
            // ros
            ros::NodeHandle nh;
            nav_msgs::Path current_path;
            string world_frame_id;

            ros::Publisher pub_path_cur_solution; // publisher for path 
            ros::Publisher pub_vis_observations; // observations 
            ros::Publisher pub_vis_goal; // goal point 
            ros::Publisher pub_marker_pnts_path; // pnts on the path 
            
            visualization_msgs::Marker obsrv_markers;
            visualization_msgs::Marker goal_marker;
            visualization_msgs::Marker pnts_on_path_marker;
            
            // map 
            DynamicEDTOctomap *edf_ptr; // Euclidean Distance Field 
            voxblox::EsdfServer voxblox_server;  // voxblox server 
            
            // related parameters 
            double ground_rejection_height;            
            double dx; // obtain from octomap 
            double r_safe; // safe clearance (outside of r_safe, cost = 0) 
            OptimParam optim_param; // optimization parameters       
            OptimParam optim_param_default; // optimization parameters           
    
            int dim = 2;
            // flags 
            bool is_map_load = false; // is map loaded             
            // chomp solver 
            Solver solver;


        public:
            OptimResult recent_optim_result; // recent optimization result 

            int map_type; // 0 = octomap, 1 = voxblox 
            Wrapper(const ros::NodeHandle & );
            //ros
            void load_markers_prior_pnts(nav_msgs::Path prior_path,geometry_msgs::Point goal);
            void publish_routine();
            // map and edf 
            void load_map(octomap::OcTree* octree_ptr);
            void load_map(string file_name);            
            // cost evaluation
            
            // optimization routine 
            void build_matrix(MatrixXd &A,VectorXd &b,nav_msgs::Path prior_points,geometry_msgs::Point goal,OptimParam* param = NULL);
            VectorXd prepare_chomp(MatrixXd A,VectorXd b,nav_msgs::Path prior_path,geometry_msgs::Point goal,OptimParam* param = NULL );
            bool solve_chomp(VectorXd x0);

            // result retreive
            MatrixXd get_current_prediction_path();
            OptimParam get_default_optim_param();
            double get_ground_height() {return ground_rejection_height;};


    };
}
