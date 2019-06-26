#include "chomp_predict/chomp_ros_wrapper.h"

using namespace CHOMP;

Wrapper::Wrapper(){    
    nh.param("ground_rejection_height",ground_rejection_height,0.4);
};

// load EDF map from octree. The octree is assumed to be provided from outside 
void Wrapper::load_map(octomap::OcTree* octree_ptr){
    
    // EDT map scale = octomap  
    double x,y,z;
    octree_ptr->getMetricMin(x,y,z);
    octomap::point3d boundary_min(x,y,z); 
    boundary_min.z() =ground_rejection_height;
    octree_ptr->getMetricMax(x,y,z);
    octomap::point3d boundary_max(x,y,z); 
    dx = octree_ptr->getResolution();

    double edf_max_dist = 3.0;
    bool unknownAsOccupied = false;

    // EDF completed
    edf_ptr = new DynamicEDTOctomap(edf_max_dist,octree_ptr,
        boundary_min,
        boundary_max,unknownAsOccupied);
    edf_ptr->update();
    
    // flag 
    is_map_load = true;
};

// evaluate cost at one point 
double Wrapper::cost_at_point(geometry_msgs::Point p){    
    try{
        if (!is_map_load)
            throw 1;        
        double distance_raw = edf_ptr->getDistance(octomap::point3d(p.x,p.y,p.z));
        // compute real cost from distance value 
        if (distance_raw <=0 )
           return (-distance_raw + 0.5*r_safe); 
        else if((0<distance_raw) and (distance_raw < r_safe) ){
            return 1/(2*r_safe)*pow(distance_raw - r_safe,2);                
        }else        
            return 0;

    }catch(exception e){
        std::cout<<"error in evaulating EDT value. Is edf completely loaded?"<<std::endl;
    }
}
// TODO : calculating cost and its gradient can be placed together. It might improve the performance 
// evalute cost of a path 
double Wrapper::cost_obstacle(VectorXd x){
    // length(x) = dim x H
    double cost = 0;
    int H = x.size()/2;
    for(int h = 0;h<H;h++){
            geometry_msgs::Point p;
            p.x = x(2*h);
            p.y = x(2*h+1);      
            p.z = ground_rejection_height+1e-2;                          
            cost += cost_at_point(p);        
        }
}

// evaluate gradient of cost of a path 
VectorXd Wrapper::grad_cost_obstacle(VectorXd x){
    // length(x) = dim x H
    int H = x.size()/2;
    double cost0 = cost_obstacle(x); // original cost 
    VectorXd grad;    
    for(int h = 0;h<H;h++){
        VectorXd pert_x(x.size()); pert_x.setZero(); pert_x(2*h) = dx; grad(2*h) = (cost_obstacle(x+pert_x) - cost0)/dx;
        VectorXd pert_y(x.size()); pert_y.setZero(); pert_x(2*h+1) = dx; grad(2*h+1) = (cost_obstacle(x+pert_y) - cost0)/dx;
    }
    return grad;
}

void Wrapper::prepare_chomp(MatrixXd A,VectorXd b){

    if (A.rows() == b.size())
    // complete problem with obstacle functions 
        solver.set_problem(A,b,this->cost_obstacle,this->grad_cost_obstacle);
    else
        cerr<<"[CHOMP] dimension error for optimizatoin"<<endl;
}

bool Wrapper::solve_chomp(VectorXd x0,OptimParam param){
    recent_optim_result = solver.solve(x0,param);    
}
nav_msgs::Path Wrapper::get_solution_path(){


}
