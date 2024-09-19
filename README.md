# SLAM Extended Kalman Filter

This project aims to implement from scratch the EKF SLAM algorithm in a simple simulated environment in Gazebo and ROS2 (Galactic). The robot is the Turtlebot3, cylindrical landmarks are scattered around the environment. The center of the cylinders are inferred from the lidar scan and used as the landmark locations. In the demo below, the left panel plots the robot and landmark location while the right panels show the covariance matrix and the Gazebo environment.

## Demo
![EKF SLAM Demo](EKF%20SLAM%20DEMO.gif)

## Measurement model

According to *Probabilistic Robotics* (Thrun et al.), the standard formulation of the bearing measurement is the following:

**θ̅<sub>j</sub> = arctan2(ȳ<sub>j</sub> − ȳ<sub>r</sub>, x̅<sub>j</sub> − x̅<sub>r</sub>) − ϕ̅<sub>r</sub>**

where **[x̅<sub>r</sub>, ȳ<sub>r</sub>, ϕ̅<sub>r</sub>]<sup>T</sup>** represents the predicted robot pose, **[x̅<sub>j</sub>, ȳ<sub>j</sub>]** is the landmark location. Let **δx = x̅<sub>j</sub> − x̅<sub>r</sub>** and **δy = ȳ<sub>j</sub> − ȳ<sub>r</sub>**. Instead of dealing with wrapping around π and −π, it is easier and more consistent to express the bearing in the robot or sensor frame. Define the rotation matrix:

**R = [ c  -s ]  
     [ s   c ]**

where **c = cos(ϕ̅<sub>r</sub>)**, **s = sin(ϕ̅<sub>r</sub>)**. The bearing expressed in the robot frame is thus:

**θ̅<sub>j</sub><sup>(r)</sup> = arctan2(δy<sup>(r)</sup>, δx<sup>(r)</sup>)**

where **δx<sup>(r)</sup> = cδx + sδy**, and **δy<sup>(r)</sup> = −sδx + cδy**. The partial derivative of the bearing with respect to the robot landmark state is thus:

**∂θ̅<sub>j</sub><sup>(r)</sup> / ∂x̅<sub>r</sub> = −cρ<sub>x</sub> + sρ<sub>y</sub>**  
**∂θ̅<sub>j</sub><sup>(r)</sup> / ∂ȳ<sub>r</sub> = −sρ<sub>x</sub> − cρ<sub>y</sub>**  
**∂θ̅<sub>j</sub><sup>(r)</sup> / ∂ϕ̅<sub>r</sub> = δy<sup>(r)</sup> ρ<sub>x</sub> − δx<sup>(r)</sup> ρ<sub>y</sub>**  
**∂θ̅<sub>j</sub><sup>(r)</sup> / ∂x̅<sub>j</sub> = cρ<sub>x</sub> − sρ<sub>y</sub>**  
**∂θ̅<sub>j</sub><sup>(r)</sup> / ∂ȳ<sub>j</sub> = sρ<sub>x</sub> + cρ<sub>y</sub>**

where **ρ<sub>x</sub> = −δy<sup>(r)</sup> / ((δx<sup>(r)</sup>)² + (δy<sup>(r)</sup>)²)**, and **ρ<sub>y</sub> = δx<sup>(r)</sup> / ((δx<sup>(r)</sup>)² + (δy<sup>(r)</sup>)²)**. This formulation is used in the implementation of this project. A similar approach is used whenever the angle between two yaw angles needs to be calculated.
