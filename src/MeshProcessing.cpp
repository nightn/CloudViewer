#include "MeshProcessing.h"
#include <pcl/Vertices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>
#include <pcl/PolygonMesh.h>
#include <vector>

// 计算三角形面积，接收三个角度的 xyz 坐标，返回面积
double calculateTriangleMeshArea(
  double x1, double y1, double z1,
  double x2, double y2, double z2,
  double x3, double y3, double z3) {
  // Heron's formula
  double a, b, c, q;
  double area;
  a = sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2) + pow((z1 - z2), 2));
  b = sqrt(pow((x1 - x3), 2) + pow((y1 - y3), 2) + pow((z1 - z3), 2));
  c = sqrt(pow((x3 - x2), 2) + pow((y3 - y2), 2) + pow((z3 - z2), 2));
  q = (a + b + c) / 2;
  area = sqrt(q * (q - a) * (q - b) * (q - c));
  return area;
}

// 接收 pcl::PolygonMesh 常量引用，计算网格总面积（注意：网格不一定是三角形）
// v0.2 这个版本只是将多边形按给定点三角剖分，但有两个局限：
//   (1) 多边形顶点必须是有序的；（2）只适用于凸多边形，不适用于凹多边形
// FIXME:
double calculatePCLPolygonMeshArea(const pcl::PolygonMesh& mesh) {
  size_t index1, index2, index3;
  double x1, y1, z1, x2, y2, z2, x3, y3, z3;
  double area = 0;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, cloud);
  // for each polygon
  for (size_t i = 0; i != mesh.polygons.size(); ++i) {

    std::vector<uint32_t> vertexIndexes = mesh.polygons[i].vertices;
    int n = vertexIndexes.size();  // n 边形

    // 将一个 n 边形分解为 (n - 2) 个三角形
    double polygonArea = 0;
    for (int k = 1; k != n - 1; ++k) {
      index1 = vertexIndexes[0];
      index2 = vertexIndexes[k];
      index3 = vertexIndexes[k + 1];

      x1 = cloud[index1].x;
      y1 = cloud[index1].y;
      z1 = cloud[index1].z;

      x2 = cloud[index2].x;
      y2 = cloud[index2].y;
      z2 = cloud[index2].z;

      x3 = cloud[index3].x;
      y3 = cloud[index3].y;
      z3 = cloud[index3].z;

      polygonArea += calculateTriangleMeshArea(x1, y1, z1, x2, y2, z2, x3, y3, z3);
    }

    area += polygonArea;
  }
  return area;
}

// Greedy Projection triangulation
pcl::PolygonMesh triangulationGreedyProjection(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud) {
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(xyzCloud);
  normalEstimation.setInputCloud(xyzCloud);
  normalEstimation.setSearchMethod(tree);
  normalEstimation.setKSearch(20);
  normalEstimation.compute(*normals);

  pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointNormal>);
  // 将已获得的点数据和法向数据拼接
  pcl::concatenateFields(*xyzCloud, *normals, *cloudWithNormals);

  // another kd-tree for reconstruction
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud(cloudWithNormals);

  // reconstruction
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh mesh;
  // options
  gp3.setSearchRadius(25);
  gp3.setMu(2.5);
  gp3.setMaximumNearestNeighbors(100);
  gp3.setMaximumSurfaceAngle(M_PI / 2);
  gp3.setMinimumAngle(M_PI / 18);
  gp3.setMaximumAngle(2 * M_PI / 3);
  gp3.setNormalConsistency(false);
  gp3.setInputCloud(cloudWithNormals);
  gp3.setSearchMethod(tree2);
  gp3.reconstruct(mesh);

  return mesh;
}

