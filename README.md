# chomp_predict

____

 This package presents a method for moving target movement prediction based on Covariant Hamiltonian Optimization in order to tackle obstacle environments. We consider scenarios such as chasing or autonomous video-graphy where an actor(target) is assigned a set of sparse via-points in a workplace. In this case, it is required to predict the
future movement of target reliably so that an autonomous shooter could plan a chasing path efficiently in real-time. In
this project, an optimization is formulated by leveraging observation history and prior knowledge of the waypoints of target along with assumptions on behavior. The optimization is solved on covariant framework to ensure the smooth path update during iterations. After obtaining the geometric path prediction from optimization, a time parametrization is preformed to complete the path into a trajectory for prediction.

<p align = "center">
<img src="https://github.com/icsl-Jeon/hsv_target_localization/blob/master/img/hsv_tracker_intro.gif">
</p>



## 1. Usage

### 1) simple test 

Without loop, one shot simulation is performed. We can do tuning parameters from this node.

```d
roscd chomp_predict
rosrun chomp_predict chomp_predict_basic_test_node ./worlds/map3.vxblx 
```

### 2) prediction simulation from rosbag 

```d
roslaunch chomp_predict chomp_predict_sim 
# in other terminal 
roscd chomp_predict/data
rosbag play -r 0.3 map3_target_move_recode.bag
```

