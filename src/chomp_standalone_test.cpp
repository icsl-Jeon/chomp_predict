#include "chomp_predict/chomp_ros_wrapper.h"

using namespace octomap;
using namespace CHOMP;

int main(int argc,char *argv[]){
    
    // 1. Load octomap by argument parsing

    // Check the number of parameters
    if (argc < 2) {
        // Tell the user how to run the program
        std::cerr << "Usage: " << "NODE_NAME" << " OCTOMAP_FILE_NAME" << std::endl;
        return 1;
    }
    // Print the octomap file name:
    string file_name(argv[1]);
    std::cout << "Provided octomap file: "<<file_name<< std::endl;



    // octomap read
    OcTree* tree = new OcTree(file_name);
    if(tree){ 
        ros::init(argc,argv,"chomp_standalone_node");        
        std::cerr<<"octree of size " << tree->size() <<" is opened!"<<std::endl;
        std::cerr<<"Now, create edf from the octree.."<<std::endl;

        /**
         *  Main code starts here
         */
        
        // 0. Create wrapper object 
        Wrapper chomp_wrapper;
        chomp_wrapper.load_map(tree);
        
        // 1. Build matrix and vector for optimization  

        nav_msgs::Path prior_path;
        geometry_msgs::PoseStamped pose_stamp;
                 
        MatrixXd A; VectorXd b;
        
        // prior points 
        geometry_msgs::Point p1;          
        p1.x = 4; p1.y = -1; pose_stamp.pose.position = p1;
        prior_path.poses.push_back(pose_stamp);
        
        geometry_msgs::Point p2; 
        p2.x = 3; p2.y = 0; pose_stamp.pose.position = p2;
        prior_path.poses.push_back(pose_stamp);

        // goal 
        geometry_msgs::Point g;
        g.x = 3; g.y = 4;

        // parameters
        double gamma = 0.2; 
        int N_step = 5; 
        OptimParam optim_param; 
        optim_param.descending_rate = 0.1;
        optim_param.max_iter = 300;
        optim_param.weight_prior =  1e-3;
        optim_param.termination_cond = 1e-1;

        chomp_wrapper.build_matrix(A,b,prior_path,gamma,g,N_step);
        VectorXd x0 = chomp_wrapper.prepare_chomp(A,b,prior_path,g);
        chomp_wrapper.solve_chomp(x0,optim_param);
        
        


    // read error returns NULL
    }else{
        std::cerr<<"No octomap file found"<<std::endl;
        return 1;
    }

    
    return 0;
}


