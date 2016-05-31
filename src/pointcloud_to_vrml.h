#ifndef POINTCLOUD_TO_VRML_H
#define POINTCLOUD_TO_VRML_H
#include <omp.h>

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>


#include <vtkIVWriter.h>



class PointCloudToVRML
{
private:
    std::string AbsolutPathToPCLFile;
    std::string PathToSaveInvetorFile;

public:
    PointCloudToVRML();
    void setAbsolutPathToPCLFile(std::string AbsolutPathToPCLFile);
    void setPathToSaveInvetorFile(std::string PathToSaveInvetorFile);
    void getAbsolutPathToPCLFile(std::string &AbsolutPathToPCLFile);
    void getPathToSaveInvetorFile(std::string &PathToSaveInvetorFile);

    std::string getAbsolutPathToPCLFile();
    std::string getPathToSaveInvetorFile();

    int WriteToVRML();

};

#endif // POINTCLOUD_TO_VRML_H
