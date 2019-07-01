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

        // update markers
        chomp_wrapper.load_markers_prior_pnts(prior_path,g);

        // 2. Solve optimization 
        chomp_wrapper.build_matrix(A,b,prior_path,g); // A,b matrix 
        VectorXd x0 = chomp_wrapper.prepare_chomp(A,b,prior_path,g); // initial guess 

        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        chomp_wrapper.solve_chomp(x0); // optimization from the initial guess 
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        double diff = std::chrono::duration_cast<chrono::nanoseconds>( end - start ).count()*1e-9;
        printf("[CHOMP] dynamic EDT computed in %f [sec]",diff);

        // 3. Publish routine 
        ros::Rate loop_rate(30);
        while(ros::ok()){            
            chomp_wrapper.publish_routine();
            loop_rate.sleep();
        }


    // read error returns NULL
    }else{
        std::cerr<<"No octomap file found"<<std::endl;
        return 1;
    }

    
    return 0;
}


