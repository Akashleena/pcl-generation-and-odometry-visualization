# Point Cloud Registration and Odometry Visualization

_Assignment for Robotics Software Engineer HoloSuit Pte.Ltd_

## Dependencies

* G++
* OpenCV(tested with OpenCV 2.4 )
* PCL (tested with PCL 1.2) 

For visualization used PCLVisualizer

## System
Tested in Ubuntu 18.04

## Compile and Build Commands


Navigate to the workspace
```
$cd pcl-generation-and-odometry-visualization
```
To compile using the CMakefile using the following commands 
```
$cd build
$cmake ..
$make -j4
```
To run the code
```
$cd ../bin
$./generate_pointcloud
```
