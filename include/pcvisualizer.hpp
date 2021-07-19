#include <iostream>
#include <thread>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

//To initialise and open a 3D viewer and update it with a pointcloud

pcl::visualization::PCLVisualizer::Ptr createViewer (pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud) 
{ 
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer")); 
    //std::string viewerName = "3D Viewer";
    viewer->setBackgroundColor (0, 0, 0); 
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGBA> (cloud,rgb,"sample cloud"); 
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud"); 
    viewer->addCoordinateSystem (1.0); 
    viewer->initCameraParameters (); 

    return (viewer); 
} 


