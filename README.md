# chomp_predict

____

 This package presents a method for moving target movement prediction based on Covariant Hamiltonian Optimization in order to tackle obstacle environments. We consider scenarios such as chasing or autonomous video-graphy where an actor(target) is assigned a set of sparse via-points in a workplace. In this case, it is required to predict the
future movement of target reliably so that an autonomous shooter could plan a chasing path efficiently in real-time. In
this project, an optimization is formulated by leveraging observation history and prior knowledge of the waypoints of target along with assumptions on behavior. The optimization is solved on covariant framework to ensure the smooth path update during iterations. After obtaining the geometric path prediction from optimization, a time parametrization is preformed to complete the path into a trajectory for prediction.

In this package, we support two map representation. One is octomap (EDT) and the other is voxblox (SEDT). The user can compare the performance of the two. In our implementation, *octomap is more preferable.* The main difference of them is that voxblox can compute **signed** distance field which gives negative value inside of obstacle. In contrast, all the distance value inside of obstacle is zero, which might be disadvantageous to pull out a trajectory which crosses obstacles. In our implementation, however,  the difference was negligible.  

<p align = "center">
<img src= "https://github.com/icsl-Jeon/chomp_predict/blob/master/img/chomp_predict_intro.gif" width="600">
</p>




#### (1) Dependencies

#### 	a. [voxblox_ros](<https://voxblox.readthedocs.io/en/latest/pages/Installation.html>) 

#### 	b. [octomap](<https://github.com/OctoMap/octomap>)

#### 	c. [dyanmicEDT3D](<https://github.com/OctoMap/octomap>)



## 1. Usage 

### 1.1 run first program

0) Check doxygen in ./doc/index.html

1) simple test (for map3) 

Without loop, one shot simulation is performed. We can do tuning parameters in this node. In this node, we perform simulation on a specific map (map3.*) and with a specific prior points ([5,10] , [6,11]) and a goal([10,10]).

```
roscd chomp_predict
rosrun chomp_predict chomp_predict_basic_test_node ./worlds/map3.vxblx (or map3.bt) 
```

2) prediction simulation from rosbag 

This launch file can test two different scenarios (map3.(bt/vxblx) / 4floor0826.(bt/vxblx))

<p align = "center">
<img src= "https://github.com/icsl-Jeon/chomp_predict/blob/master/img/two_maps.png" width="400">
</p>

```
roslaunch chomp_predict chomp_predict_sim.launch 
```

### 1.2 node 

#### 0) parameters (see ./params/chomp_param_map3.yaml for example)

* cost_param/r_safe: the obstacle cost will be zero outside of this distance
* cost_param/ground_reject_height: the ground level(z) of target (which is assumed to move on 2D plane)

* optim_param/descending_rate: step size of update 

* optim_param/descending_min[|max]: do not change. They will not be used (TODO)

* optim_param/max_iter: maximum iteration 

* optim_param/weight_prior:  relative importance to fitting observation history, penalty for high-order derivatives and intermittent goal    

* optim_param/term_cond: termination condition (cost_prev - cost)

* optim_param/gamma: relative importance to more recent fitting points. exp(gamma * n/No) where n <= No 

  

* pred_param/prediction_horizon: the time window [sec] for evaluation of prediction. Total 20 points will be evaluated between  [t_cur,t_cur +prediction_horizon ]  

* pred_param/observation_size: size of observation (/target_pose) queue. They become points to be fitted in prior term

* pred_param/observation_horizon: time window [sec] to be looked back for the past   

  

* target_waypoint: {x: [3.4578, 6.7478, 8.3223, 11.4981], y:[4.1620, 8.0200, 3.9896, 8.0515]} set the waypoints to be passed by target (ensure that the waypoints are actually passed by target in your record.).   

  

#### 1) Topics

**Subscribed**  : 

* /target_pose [geometry_msgs/PoseStamped] (if param ```is_pose``` is true )
* /target_point [geometry_msgs/PointStamped] (if param ```is_pose``` is false )

**Published** : 

- /chomp_predict_sim/chomp_solution_path [nav_msgs/Path]

- /chomp_predict_sim/chomp_goal [visualization_msgs/Marker]

- /chomp_predict_sim/chomp_obsrv [visualization_msgs/Marker]

- /chomp_predict_sim/chomp_sol_pnts [visualization_msgs/Marker]

- /chomp_predict_sim/target_waypoints [visualization_msgs/Marker]



   *Belows are important only in voxblox mode*

 * /chomp_predict_sim/esdf_slice [sensor_msgs/PointCloud2]

 * /chomp_predict_sim/occupied_nodes [visualization_msgs/MarkerArray]

 * /chomp_predict_sim/prediction_traj [nav_msgs/Path]

 * /chomp_predict_sim/surface_pointcloud [sensor_msgs/PointCloud2]

 * /chomp_predict_sim/traversable [sensor_msgs/PointCloud2]

 * /chomp_predict_sim/tsdf_map_out [voxblox_msgs/Layer]

 * /chomp_predict_sim/tsdf_pointcloud [sensor_msgs/PointCloud2]

 * /chomp_predict_sim/tsdf_slice [sensor_msgs/PointCloud2]

   
