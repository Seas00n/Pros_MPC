# Pros_MPC

## Install from source code



```shell
##TODO
```



## Quick Start

1. Run the simulation

   ```shell
   source <directory_to_ws>/<catkin_ws_name>/devel/setup.bash
   roslaunch ocs2_pros world.launch
   ```

2. Start the controller in simulation





## Overview

1. Write your `.urdf` file. The first step is to collect data in swing phase, and then calculate the parameters using  `identification.py` . Use the result to modify the inertial parameters in `ocs2_pros/urdf/pros.urdf`. 

2.  除了`urdf`文件还需要实现`task.info`, `reference.info`, `gait.info`文件

   #### identification.py: Inertial Parameters Identification based on SQP

   #### task.info: Optimization task

   #### reference.info: Default parameters in each gait cycle

   #### gait.info: Mode Sequence and Switching Times 

3. `LeggedController`: 实现了底层的控制器
4. `legged_target_trajectories_publisher`: 运行时只需要发送`/cmd_vel`消息或者`/move_base_simple/goal`消息，`legged_target_trajectories_publisher`会生成类型为`TargetTrajectories`的预测轨迹





## More about OCS2

SystemObservation:  ocs2_mpc

TargetTrajectories: ocs2_core







## Reference

[1]**legged_control** from **qiayuanliao** : https://github.com/qiayuanliao/legged_control

[2]**OCS2** from **leggedrobotics** : https://github.com/leggedrobotics/ocs2

[3]**ros_control** :http://wiki.ros.org/ros_control

