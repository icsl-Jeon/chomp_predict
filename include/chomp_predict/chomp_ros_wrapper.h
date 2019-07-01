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
            
            // map 
            DynamicEDTOctomap *edf_ptr; // Euclidean Distance Field 
            // related parameters 
            double ground_rejection_height;            
            double dx; // obtain from octomap 
            double r_safe; // safe clearance (outside of r_safe, cost = 0)            
            int dim = 2;
            // flags 
            bool is_map_load = false; // is map loaded             
            // chomp solver 
            Solver solver;
            OptimResult recent_optim_result; // recent optimization result 
            ros::Publisher pub_path_cur_solution; // publisher for path 

        public:
            Wrapper();
            //ros
            void publish_routine();
            // map and edf 
            void load_map(octomap::OcTree* octree_ptr);
            // cost evaluation
            double cost_at_point(geometry_msgs::Point p); // cost of point p 
            double cost_obstacle(VectorXd x);
            VectorXd grad_cost_obstacle(VectorXd x);
            
            // optimization routine 
            void build_matrix(MatrixXd &A,VectorXd &b,nav_msgs::Path prior_points,double gamma,geometry_msgs::Point goal,int N);
            VectorXd prepare_chomp(MatrixXd A,VectorXd b,nav_msgs::Path prior_path,geometry_msgs::Point goal);
            bool solve_chomp(VectorXd x0,OptimParam optimization_param);            
    };
}
