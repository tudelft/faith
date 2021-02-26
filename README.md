# FAITH: Fast iterative half-plane focus of expansion estimation using event-based optic flow

This repository contains the FAITH algorithm implementation on ROS. 

Supporting paper: https://arxiv.org/abs/2102.12823

## Matlab implementation

We provide the Matlab implementation for our FAITH method, as well as the 3 other methods that we used for comparison in our paper. Namely: 
  - The NESW method: Huang, R., & Ericson, S. (2018, June). An Efficient Way to Estimate the Focus of Expansion. In 2018 IEEE 3rd International Conference on Image, Vision and Computing (ICIVC) (pp. 691-695). IEEE.  
  - The Vec. Intersections method: Buczko, M., & Willert, V. (2017, June). Monocular outlier detection for visual odometry. In 2017 IEEE Intelligent Vehicles Symposium (IV) (pp. 739-745). IEEE.
  - The Half-planes method: Clady, X., Clercq, C., Ieng, S. H., Houseini, F., Randazzo, M., Natale, L., ... & Benosman, R. B. (2014). Asynchronous visual event-based time-to-contact. Frontiers in neuroscience, 8, 9.

Each function takes as input the (x,y) coordinates of the PoIs that provided the flow vectors (u,v). The output is the FoE candidate (x_foe,y_foe) coordinates. 

## ROS implementation

These ROS packages are meant for use with ROS 1 (Kinetic, Melodic). They require you to use the DVS240 event-based camera, along with the ROS drivers (). 
  - The `dvs_of` package is meant to output derotated flow from the DVS.
  - The `foe_estimator` package uses the optic flow provided by the `dvs_of` package to estimate the FOE position (FAITH).
  - The `object_detection` package uses the FOE estimation to make a drone equipped with the DVS camera avoid obstacles. 

For further information, please contact Julien Dupeyroux: j.j.g.dupeyroux@tudelft.nl 
