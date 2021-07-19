#include <iostream>
#include <thread>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

//using namespace std::chrono_literals;
/*
//To initialise and open a 3D viewer 

boost::shared_ptr createViewer()
{
boost::shared_ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
viewer->setBackgroundColor (0, 0, 0);
viewer->addCoordinateSystem (1.0);
viewer->initCameraParameters ();
return (viewer);
}

//To update the viewer with a point cloud

void updateViewer (boost::shared_ptr viewer, pcl::PointCloud::ConstPtr cloud)
{
pcl::visualization::PointCloudColorHandlerRGBField rgb(cloud);
viewer->removePointCloud();
viewer->addPointCloud (cloud, rgb);
viewer->spinOnce();
}*/
//To initialise and open a 3D viewer 
//boost::shared_ptr<pcl::visualization::PCLVisualizer> createViewer (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, std::string& viewerName="3D Viewer") 
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
/*
//To update the viewer with a point cloud
void visualizePointCloud (boost::shared_ptr viewer,pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) 
{ 
    std::string viewerName = "3D Viewer"; 

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer; 
    viewer = createViewer(cloud, viewerName); 

    while (!viewer->wasStopped()) 
    { 
        viewer->spinOnce(100); 
        boost::this_thread::sleep (boost::posix_time::microseconds (100000)); 
    } 
	viewer->close();
}*/


