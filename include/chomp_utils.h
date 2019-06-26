#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <vector>

/** Linear model evaluation **/
struct LinearModel{
    // x=beta0+beta1*t
    double beta0;
    double beta1;
};

LinearModel linear_regression(const Eigen::VectorXd& ts,const Eigen::VectorXd& xs);

double model_eval(const LinearModel& model,double t){
    return model.beta0+model.beta1*t;
}
// convert nav_msgs::Path to vector set x/y/z
void path2vec(const nav_msgs::Path& path,std::vector<double> &xs,std::vector<double> &ys,std::vector<double> &zs){

    unsigned long path_len=path.poses.size();

    xs.resize(path_len);
    ys.resize(path_len);
    zs.resize(path_len);

    for (int i=0;i<path_len;i++){
        xs[i]=path.poses[i].pose.position.x;
        ys[i]=path.poses[i].pose.position.y;
        zs[i]=path.poses[i].pose.position.z;
    }
};

// convert nav_msgs::Path to vector set x/y/z
void vec2path(std::vector<double> &xs,std::vector<double> &ys,std::vector<double> &zs,nav_msgs::Path& path){

    // Of course, we assume length of xs,ys and zs are same
    unsigned long path_len=xs.size();
    for (int i=0;i<path_len;i++){
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = xs[i];
        pose_stamped.pose.position.y = ys[i];
        pose_stamped.pose.position.z = zs[i];        
        path.poses.push_back(pose_stamped);        
    }

};




