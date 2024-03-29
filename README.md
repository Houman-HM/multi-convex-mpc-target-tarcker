# multi-convex-mpc-target-tarcker
This repository containes the impelmetation of MPC algorithm proposed in "Real-Time Multi-Convex Model Predictive Control for Occlusion-Free Target Tracking with Quadrotors" [paper](https://doi.org/10.1109/ACCESS.2022.3157977).

# Dependecies:

* [eigen_quad_prog](https://github.com/jrl-umi3218/eigen-quadprog)
* [bebop_simulator](https://github.com/gstavrinos/bebop_simulator)
* [AutoChaser](https://github.com/icsl-Jeon/traj_gen_vis)

## Installation procedure
After installing the dependencies, you can build our propsed MPC package as follows:
``` 
cd your_catkin_ws/src
git clone https://github.com/Houman-HM/multi-convex-mpc-target-tarcker.git
cd .. && catkin build
source your_catkin_ws/devel/setup.bash
```

There are several launch files for different Gazebo setups for different number of obstacles. You can launch any of Gazebo worlds and run the algorithm by running the corresponding MPC node.
For example in order to run the MPC for tracking a target in a world with 6 obstacles, follow the procedure below:

##### In the first terminal:
```
roslaunch target_tracker 6_cylinder_world_dynamic.launch
```
##### In the second terminal:
```
rosrun target_tracker mpc_tracker_6_obs_ros_node
```
Now, you can start teleoperating the target by publishing velocities on ``` /target/cmd_vel ``` topic. The drone should start following the target as you are teleoperating the target.
## Running Nageli implmentation using ACADO
#### Running generated ACADO code
* Edit global variables in ```test.c```: maker position, obstacles initial position and velocity, quadrotor's initial position, weights
* Run ```make clean all``` only for the first time otherwise just ```make``` followed by ```./test```.
* To visualize, run ```python quad_plot.py```

_Look into code_gen.cpp file for acado settings, cost terms, constraints, etc._

<!-- #### Generating ACADO code
* Install [ACADO Toolkit](https://acado.github.io/install_linux.html)
* Copy ```getting_started.cpp``` from ```Nageli_implementation``` folder and paste it to ```/ACADOtoolkit/examples/code_generation/mpc_mhe```.
* Edit ```getting_started.cpp``` for desired settings (steps, planning time).
* Navigate to ```ACADOtoolkit/build``` and run ```make code_generation_getting_started```.
* Finally run ```./code_generation_getting_started``` in directory ```/ACADOtoolkit/examples/code_generation/mpc_mhe```.  -->
