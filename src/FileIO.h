#ifndef __FILE_IO_H__
#define __FILE_IO_H__

#include <QFileInfo>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>  // loadPolygonFileOBJ
#include <vector>
#include <string>
#include <map>
#include "MyCloud.h"

using std::string;
using std::map;

class FileIO {
public:

  MyCloud load(const QFileInfo& fileInfo);
  MyCloud loadPLY(const QFileInfo& fileInfo);
  MyCloud loadPCD(const QFileInfo& fileInfo);
  MyCloud loadOBJ(const QFileInfo& fileInfo);
  MyCloud loadSTL(const QFileInfo& fileInfo);
  MyCloud loadVTK(const QFileInfo& fileInfo);

  bool save(const MyCloud& myCloud, const QFileInfo& fileInfo, bool isBinaryFormat);
  bool savePLY(const MyCloud& myCloud, const QFileInfo& fileInfo, bool isBinaryFormat);
  bool savePCD(const MyCloud& myCloud, const QFileInfo& fileInfo, bool isBinaryFormat);
  bool saveOBJ(const MyCloud& myCloud, const QFileInfo& fileInfo);
  bool saveSTL(const MyCloud& myCloud, const QFileInfo& fileInfo, bool isBinaryFormat);
  bool saveVTK(const MyCloud& myCloud, const QFileInfo& fileInfo, bool isBinaryFormat);

  string getInputFormatsStr() const;
  string getOutputFormatsStr() const;

  map<string, string> inputFiltersMap = {
    {"ply", "Stanford Polygon File Format (*.ply)"},
    {"pcd", "PCL Point Cloud Data (*.pcd)"},
    {"obj", "Alias Wavefront Object (*.obj)"},
    {"stl", "STL File Format (*.stl)"},
    {"vtk", "Visualization Tookit Format (*.vtk)"},
    {"*", "All Files (*.*)"}
  };

  map<string, string> outputFiltersMap = {
    {"ply", "Stanford Polygon File Format (*.ply)"},
    {"pcd", "PCL Point Cloud Data (*.pcd)"},
    {"obj", "Alias Wavefront Object (*.obj)"},
    {"stl", "STL File Format (*.stl)"},
    {"vtk", "Visualization Tookit Format (*.vtk)"},
    {"*", "All Files (*.*)"}
  };
};
#endif

