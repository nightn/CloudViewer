#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>

using namespace std;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class MyCloud
{
public:
	MyCloud();
	~MyCloud();

	PointCloudT::Ptr cloud;  //点云指针
	string filename;  //点云的全路径文件名
	string subname;  //点云的文件名
	string dirname = "E:\\Date\\PointCloud\\";
	bool visible = true;  //点云在 viewer 中是否可见


};

