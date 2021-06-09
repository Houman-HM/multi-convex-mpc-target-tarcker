# multi-convex-mpc-target-tarcker
This repository containes the impelmetation of MPC algorithm proposed in Real-Time Multi-Convex Model Predictive Control for Occlusion-Free Target Tracking with Quadrotors paper.

## Installation procedure
``` 
cd your_catkin_ws/src
git clone https://github.com/Houman-HM/multi-convex-mpc-target-tarcker.git
cd .. && catkin build
```

## Running Nageli implmentation using ACADO
#### Running generated ACADO code
* Edit global variables in ```test.c```: maker position, obstacles initial position and velocity, quadrotor's initial position, weights
* Run ```make clean all``` only for the first time otherwise just ```make``` followed by ```./test```.
* To visualize, run ```python quad_plot.py```

_Look into code_gen.cpp file for acado settings, cost terms, constraints, etc._

#### Generating ACADO code
* Install [ACADO Toolkit](https://acado.github.io/install_linux.html)
* Copy ```getting_started.cpp``` from ```acadoOptim_alonso_TI_3obs``` folder and paste it to ```/ACADOtoolkit/examples/code_generation/mpc_mhe```.
* Edit ```getting_started.cpp``` for desired settings (steps, planning time).
* Navigate to ```ACADOtoolkit/build``` and run ```make code_generation_getting_started```.
* Finally run ```./code_generation_getting_started``` in directory ```/ACADOtoolkit/examples/code_generation/mpc_mhe```. 