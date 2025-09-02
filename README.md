# About 
This repo exists to showcase an Extented Kalman Filter I implemented during my co-op at Wato.  
The repo contains the ros2 package for the EKF and also contains a breif write explaning  
the process of making designing and making your own EKF. 

# A Novice's Guide to Designing An Asynchronous Extended Kalman Filter

## Background and Motivation

Before I talk about what an Extended Kalman Filter (EKF) is, it’s better to first ask: why do we even need one?

If you’ve ever used motor encoders to measure distance, you know a common problem is that the encoder measurements become inaccurate over time.  
This is an inherent issue when we try to deterministically estimate our state. The Extended Kalman Filter addresses this through a probabilistic approach.  
The basic idea of the EKF is that it uses a model to propagate the state forward in time, then incorporates sensor data and our understanding of uncertainty to  
produce a better state estimate through a weighted average.

## Introduction

The EKF is a sensor fusion algorithm based on the Bayes filter, a mathematical framework for estimating an unknown probability distribution using a process model  
and incoming measurements. The Kalman Filter is a special case of the Bayes filter and serves as an optimal estimator for systems that are linear and where all  
uncertainties are modeled as Gaussian (normally distributed). The Extended Kalman Filter takes this idea further by allowing us to apply the Kalman Filter to  
non-linear systems. This is achieved by linearizing the non-linear process and observation models around the current estimate. This allows it to apply  
Kalman Filter techniques in a locally linear approximation of the system.

![EKF1](images/EKF_1.png "Standard Extended Kalman Filter")

Lines 1 and 2 are the prediction step, while lines 3 to 5 are the correction step. In line 1, we use our prediction function g, which is our process model, to  
predict what we believe the next state will be, given the previous state and the current control inputs. Next, the predicted covariance $\bar{\Sigma}_t$ calculated  
using the previous covariance and G, which is the Jacobian of the process model. Then we add the process noise. In the correction step, line 3, we calculate the  
Kalman gain. The Kalman gain is essentially a weighting that biases the corrected state either toward the observation or the prediction. It’s calculated using the  
predicted covariance, the Jacobian of the observation model, and the sensor noise term Q. On line 4, the state at time t is calculated using the predicted  
state, and we add the difference between the predicted and observed state, scaled by the Kalman gain. Finally, on line 5 the new covariance at time t is calculated.

![EKF_VIS](images/kalman_filter_vis.png "EKF visualization")

To build better intuition for what's happening, it's better to look at the problem from a statistical standpoint. We start with a higher-dimensional Gaussian, pass  
it through a linearized function, and the covariance grows. Then we multiply the Gaussian we got from our function with the observation Gaussian, and the result is  
a new Gaussian distribution. The covariance shrinks (ideally), and the mean becomes a weighted average between the prediction and the observation.  
That new mean becomes our updated state.

## Implementation Details

### Defining The Objective Of Our EKF

Before we go and design an EKF, it is important to understand where in our state estimation stack the EKF is and what goals we have for it.  
For my project, I had to design an EKF for a car-like/Ackermann steering robot with motor encoders, an IMU, and LiDAR. However, if you are  
designing an EKF for a different type of robot with different sensors, the process of designing will be much the same, but your equations and motion  
models will look different.

My Robot's State Estimation Articture :  

![State Estimation Articture ](images/state_estimation_artecture.png "State Estimation Articture")

As you can see in the diagram, I have three parts in my state estimation: the Odometry, the EKF, and the Particle Filter. The job of the EKF in my architecture  
is to incorporate the IMU data into the odometry data. The odometry is fairly accurate at keeping track of velocity and position in a straight line, but it  
struggles to handle changes in direction or maintain the robot's orientation. This is where the IMU comes in handy. When calibrated properly, the IMU can track  
orientation very well. The goal of the EKF is to fuse the IMU data with the odometry to correct for orientation, as well as fuse the control input velocity with  
the odometry velocity to get a good estimate of the robot's actual velocity. We do not need to worry too much about the x and y accuracy from the EKF, since the  
Particle Filter will take care of most of the heavy lifting in that department.

### Defining the state vector ($\mu_t$) and motion model $g(\mu_{t-1}, u_t)$

The state vector will be as follow :  

![State Vector](images/state_vector.png "State Vector")

prediction function $g(\mu_{t-1}, u_t)$ and control input :  

![prediction function and control input](images/motion_model.png "prediction function g and control input")

The state vector consists of the following quantities : $x$, $y$, ${\theta}$, $v$, $\dot{\theta}$, $a_x$, $a_y$. x, y and $\theta$ make up the robots pose;  
they are inputs that need to be fed into the particle filter. Additonly, $v$ is need by the cars safety system to calculate accurate collision times  
to prevent unnessary collison. We keep track of the rest of the quantities as they help enhance our other predictions. 

The control inputs for the robot are velocity and steering angle, so they are used to drive the state forward in time. The motion model or prediction function $g$  
is an implementation of the Ackermann kinematics using the tricycle model. This model is fairly simple but captures the motion of the vehicle well enough  
for our use case. Additionally, the main assumption made in the model is that the vehicle moves with constant acceleration along arcs.

### Defining the Jacobian $G$

![Jacobian G](images/Jacobian_G.png "Jacobian G")

$G$ is is the Jacobain / slope of the motion model $g$. It is a nesscary part of the Extended Kalman Filter, as it helps us linearize our non linear function  
so that we can appply the Kalman Filter framework to them. I recomend using Software such as matlab or octave to calculate this for you, as getting it wrong can   
seriously affect your EKF

### Multiple Sensor Models and Asynchrony

This is where things diverge a little from other guides. Generally, most other sources I found online talk about the single-sensor case for the EKF, but gloss over  
how to implement a multi-sensor version. Thus, I will try my best to explain the process. To start out, we must first define the motivation for asynchronous   
multi-sensor EKF. Given we have two sensors, a common strategy that is often employed is to bundle all the sensor information into one correction step. However,  
this can be rather inefficient, as you will either end up wasting sensor information (in the case of having a high-frequency sensor and a low-frequency one) or  
using stale sensor information (incorporating outdated data, leading to poor correction). To remedy this, we will use one sensor model per sensor. This way we can  
have a higher update rate for our EKF and improve performance by only using the most current data. The basic idea will be to predict when we have new control  
inputs and accept the predicted state and covariance as the estimated state, while also storing the control inputs. Next, when new sensor information arrives, we  
predict again from the time we last updated to the arrival of the sensor data using the stored control inputs, then perform the correction step with the correct   
sensor model.

### IMU Sensor Model

Imu Sensor Model : 

![Imu sensor model](images/Imu_sensor_model.png "Imu Sensor model")

Imu Jacobian H : 

![Imu Jacobian](images/imu_jacobian.png "Imu Jacobian")

Imu sensor noise matrix : 

The Imu Sensor models has the 

### Odometry Sensor Model

Odom Sensor Model : 

![Imu Jacobian](images/odom_sensor_model.png "Odom Sensor model")

Odom Jacobian H : 

![Imu Jacobian](images/odom_jacobian.png "Odom Jacobian")

Odom sensor noise matrix : 

### Process noise and Inital Covariance

### Improving Numerical Stability

## Testing and Results

### Testing Method 
### Results

## Discussion And Reflection

Overall, the filter's performance is within expectations. It doesn't drastically improve the accuracy of the x and y coordinates; however, the theta and velocity  
estimation is much improved. This achieves the initial goals set when building this filter. However, there are still areas where the filter could be much improved.  
When looking at the motion model, it is rather simplistic and can capture the motion of the robot. However, when slip is introduced into the equation or fast  
heading changes occur, the model can't fully capture the robot's motion, as it has no idea of the full system dynamics. This could be improved with a   
better model, but that would also require more compute. Additionally, my current understanding of vehicle dynamics is also limited,  
thus further limiting this area. Another area where the filter is lacking is not being truly async. The current design uses the ROS2 sequential executor,  
so the callback functions happen in sequence, rather than truly concurrently. This could be remedied by switching to the concurrent  
executor, but given the time frame in which this filter was developed and my understanding of concurrency, this was not feasible.  
Overall I am happy with the outcome, and have learned a lot. However, there is still more to learn, which excites me for the future.