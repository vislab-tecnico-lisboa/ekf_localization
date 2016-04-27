# ekf_localization
A ROS package for mobile robot localization using an extended Kalman Filter

## Description
This repository contains a ROS package for solving the mobile robot localization problem with an extended Kalman Filter. 

In this methodology, the Iterative Closest Point (ICP) algorithm is employed for matching laser scans to a grid-based map. 
The obtained alignment transformation is directly employed to obtain the residual measurement and covariance matrices.

This implementation employs a landmark-free EKF localization algorithm which relies on the transformation obtained by an ICP scan-matcher (between a known map and the laser measurements) as the residual to perform correction after the prediction step.  
Furthermore, the method uses the well-studied odometry motion model detailed in [Thrun et al. 2005].
