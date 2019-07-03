#include "chomp_predict/chomp_ros_wrapper.h"

using namespace octomap;
using namespace CHOMP;

int main(int argc,char *argv[]){
    
    int file_type; // 0 = octomap / 1 = voxblox

    // 1. Load octomap by argument parsing

    // Check the number of parameters
    if (argc < 2) {
        // Tell the user how to run the program
        std::cerr << "Usage: " << "NODE_NAME" << " OCTOMAP_FILE_NAME(.bt) or VOXBLOX_FILE_NAME(.vxblx)" << std::endl;
        return 1;
    }
    // Print the octomap file name:
    string file_name(argv[1]);
    if(file_name.substr(file_name.find_last_of(".")+1)=="bt"){
        std::cout << "Provided octomap file: "<<file_name<< std::endl;
        file_type = 1;
        
        }
    else if(file_name.substr(file_name.find_last_of(".")+1)=="vxblx"){
        std::cout << "Provided voxblox file: "<<file_name<< std::endl;
        file_type = 0;}
    else{
        std::cerr << "Usage: " << "NODE_NAME" << " OCTOMAP_FILE_NAME(.bt) or VOXBLOX_FILE_NAME(.vxblx)" << std::endl;
        return 1;
    }


    // init ros 
    ros::init(argc,argv,"chomp_standalone_node");        
    ros::NodeHandle nh("/chomp_standalone_global");
    // 0. Create wrapper object 
    Wrapper chomp_wrapper(nh);

    // 1. Create wrapper object 

        if (file_type){ 
            // octomap read
            OcTree* tree = new OcTree(file_name);
            std::cerr<<"octree of size " << tree->size() <<" is opened!"<<std::endl;
            std::cerr<<"Now, create edf from the octree.."<<std::endl;
            chomp_wrapper.load_map(tree);
            chomp_wrapper.map_type = 0; // set octomap representation 
        }else{ 
            // voxblox
            chomp_wrapper.load_map(file_name);
            chomp_wrapper.map_type = 1; // set voblox representation         
        }
            // 1. Build matrix and vector for optimization  

        nav_msgs::Path prior_path;
        geometry_msgs::PoseStamped pose_stamp;
                 
        MatrixXd A; VectorXd b;
        
        // prior points 
        geometry_msgs::Point p1;          
        p1.x = 5; p1.y = 10; pose_stamp.pose.position = p1;
        prior_path.poses.push_back(pose_stamp);
        
        geometry_msgs::Point p2; 
        p2.x = 6; p2.y = 11; pose_stamp.pose.position = p2;
        prior_path.poses.push_back(pose_stamp);

        // goal 
        geometry_msgs::Point g;
        g.x = 10; g.y = 10;

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



    
    return 0;
}


