#include <iostream>
#include <string>
 // opencv library
#include <opencv2/opencv.hpp>
 // PCL library
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
using namespace std;
 
 // Define the point cloud type
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud; 
 
 // camera internal reference
const double camera_factor=5000;
const double camera_cx=325.5;
const double camera_cy=253.5;
const double camera_fx=518.0;
const double camera_fy=519.0;
 
 // main function
int main(int argc,char** argv)
{
	 // Read rgb image and depth image, and converted to point cloud
	 // Image matrix
	cv::Mat rgb, depth;
	 // Use cv::imread () to read the image
	 //rgb image is a color image of 8UC3
	rgb = cv::imread("/home/akashleena/pointcloud/data/rgbd_dataset_freiburg1_room/rgb/1305031910.797230.png");
	 //depth is a single-channel image of 16UC1. Note that flags are set to -1, indicating that the original data is read without modification.
	depth = cv::imread("/home/akashleena/pointcloud/data/rgbd_dataset_freiburg1_room/depth/1305031910.803249.png", -1);
 
	 //Point cloud variable
	 //Create a null cloud using smart pointers. This kind of pointer will be automatically released when it is used up.
	PointCloud::Ptr cloud(new PointCloud);
	
	 // traverse the depth map
	for(int m = 0; m<depth.rows; m++) {
		for(int n = 0; n<depth.cols; n++) {
		 // Get the value at (m, n) in the depth map
		ushort d = depth.ptr<ushort>(m)[n];
		 //d may have no value, if so, skip this point
		if(d == 0)
			continue;
		 //d has a value, then add a point to the point cloud
		PointT p;
		 // Calculate the space coordinates of this point
		p.z = double(d)/camera_factor;
		p.x = (n-camera_cx)*p.z/camera_fx;
		p.y = (m-camera_cy)*p.z/camera_fy;
		 // Get its color from the rgb image
		 //rgb is a three-channel BGR format, so get the colors in the following order.
		p.b = rgb.ptr<uchar>(m)[n*3];
		p.g = rgb.ptr<uchar>(m)[n*3+1];
		p.r = rgb.ptr<uchar>(m)[n*3+2];
		 //Add p to the point cloud
		cloud->points.push_back(p);
		}
	}
	 // Set and save point cloud
	cloud->height = 1;
	cloud->width = cloud->points.size();
	cout<< "point cloud size=" << cloud->points.size() << endl;
	cloud->is_dense = false;
 	pcl::io::savePLYFileASCII ("output.ply", *cloud);
	pcl::io::savePCDFile("./pointcloud.pcd", *cloud);

	
/*struct dataType { Point3d point; int red; int green; int blue; };
typedef dataType SpacePoint;
vector<SpacePoint> pointCloud;

ofstream output("pointcloud.ply");
output << "ply\n" << "format ascii 1.0\n" << "comment VTK generated PLY File\n";
output << "obj_info vtkPolyData points and polygons : vtk4.0\n" << "element vertex " << cloud->points.size(); << "\n";
output << "property float x\n" << "property float y\n" << "property float z\n" << "element face 0\n";
output << "property list uchar int vertex_indices\n" << "end_header\n";
for (int i = 0; i < pointCloud.size(); i++)
{
    Point3d point = pointCloud.at(i).point;
    output << point.x << " ";
    output << point.y << " ";
    output << point.z << " ";
    output << "\n";
}
outfile.close();*/
	 // Clear the data and save
	cloud->points.clear();
	cout<< "Point cloud saved." << endl;
	return 0;
}
