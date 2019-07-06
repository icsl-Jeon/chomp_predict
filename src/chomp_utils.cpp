#include "chomp_utils.h"

LinearModel linear_regression(const Eigen::VectorXd& ts,const Eigen::VectorXd& xs){
    // 1st order regression on two set of vectors
    double SS_xy=xs.dot(ts)-xs.sum()*ts.sum()/ts.size();
    double SS_xx=ts.dot(ts)-pow(ts.sum(),2)/ts.size();

    double beta1=SS_xy/SS_xx;
    double beta0;

    beta1=SS_xy/SS_xx;
    beta0=xs.mean()-beta1*ts.mean();
    LinearModel model;
    model.beta0=beta0;
    model.beta1=beta1;
    return model;
}


double model_eval(const LinearModel& model,double t){
    return model.beta0+model.beta1*t;
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

Eigen::VectorXd get_time_stamps_from_nav_path(const nav_msgs::Path& path){

    int n_step = path.poses.size();
    Eigen::VectorXd time_stamps(n_step);
    for(int n =0; n<n_step;n++){
        time_stamps(n)  = (path.poses[n].header.stamp.toSec() + 
                                path.poses[n].header.stamp.toNSec()*(1e-9));
    }

    return time_stamps;
}

double interpolate( Eigen::VectorXd &xData, Eigen::VectorXd &yData, double x, bool extrapolate )
{
    int size = xData.size();

    int i = 0;                                                                  // find left end of interval for interpolation
    if ( x >= xData(size - 2) )                                                 // special case: beyond right end
    {
        i = size - 2;
    }
    else
    {
        while ( x > xData(i+1) ) i++;
    }
    double xL = xData(i), yL = yData(i), xR = xData(i+1), yR = yData(i+1);      // points on either side (unless beyond ends)
    if ( !extrapolate )                                                         // if beyond ends of array and not extrapolating
    {
        if ( x < xL ) yR = yL;
        if ( x > xR ) yL = yR;
    }

    double dydx = ( yR - yL ) / ( xR - xL );                                    // gradient
    return yL + dydx * ( x - xL );                                              // linear interpolation
}