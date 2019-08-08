#include <eigen3/Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/octomap_types.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <cmath>
#include <vector>
#include <functional>
#include <string>
#include <chrono>


// linear regression 
struct LinearModel{
    // x=beta0+beta1*t
    double beta0;
    double beta1;
};
LinearModel linear_regression(const Eigen::VectorXd& ts,const Eigen::VectorXd& xs);
double model_eval(const LinearModel& model,double t);

// conversion between nav_msgs::Path to vector set x/y/z
void path2vec(const nav_msgs::Path& path,std::vector<double> &xs,std::vector<double> &ys,std::vector<double> &zs);
void vec2path(std::vector<double> &xs,std::vector<double> &ys,std::vector<double> &zs,nav_msgs::Path& path);
Eigen::VectorXd get_time_stamps_from_nav_path(const nav_msgs::Path& path);
double interpolate( Eigen::VectorXd &xData, Eigen::VectorXd &yData, double x, bool extrapolate );
