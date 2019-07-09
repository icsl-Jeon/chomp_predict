# chomp_predict



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

