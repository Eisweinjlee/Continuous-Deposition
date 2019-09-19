# Continuous Deposition

This repository is containing files of the research project of Yamakita Lab, Tokyo Institute of Technology, Japan.

## 1. The optimal planning for continuous deposition
 
`Copy_of_test_plan_learning.m` is the main function of excavator continuous deposition, which should be runned with
- `excavator_data.m`: the parameters of the bucket, vessel of truck.
- `initializaH_2d.m`: the initialization of vessel shape (93x99 matrix).
- `function_input_2d.m`: the dynamics of the deposition.
- `desired_2d.m`: the desired soil shape (93x99 matrix).
- `objfun_2d_fl_diffusion.m`: the object function for optimization.
- `objfun_2d_fl_diffusion_LI.m`: the modified ver. of object func.
- `good_u0.mat`: the good result of optimal parameters.

However, those programs are not able to be provided because of copyright.

## 2. The soil distribution retrieved via a Kinect camera.

To obtain the soil shape, we are using the depth sensor on Kinect camera to capture the data.
- `kinect_new.m` is a program to retreive the soil shape data from our hardware system on MATLAB.
- `mainMt.m` is used in `kinect_new.m`.
- `getDepth.m`, `DepthPlot.m` and `plotDepth.m` are used in `kinect_new.m`.

Yang

Sep 19th, 2019