# Covariance Repo

Repository for writing code to turn the 
Workflow
1. [x] Python writing function
2. [ ] Eigen Writing function
    * [ ] Compilation with `g++ -o cov cov.cpp -I/usr/include/eigen3`
3. [ ] Pybind11 - for Eigen to python
3. [ ] Alternative - using GTEST to test the covariance transformation

## TODO:
* Find functioning implementation to test this
    * [ ] Test with rosbag? 

## Basic Resource

see [this link](https://stats.stackexchange.com/questions/119780/what-does-the-covariance-of-a-quaternion-mean)
going into [this file](https://github.com/PX4/PX4-Autopilot/blob/master/src/modules/ekf2/EKF2.cpp#L808-L809)
