#pragma once

#include <pcl/PolygonMesh.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <cmath>

double calculatePCLPolygonMeshArea(const pcl::PolygonMesh&);
