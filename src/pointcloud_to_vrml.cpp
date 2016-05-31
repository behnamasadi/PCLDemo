#include "pointcloud_to_vrml.h"

void PointCloudToVRML::setAbsolutPathToPCLFile(std::string AbsolutPathToPCLFile)
{
    this->AbsolutPathToPCLFile=AbsolutPathToPCLFile;
}

void PointCloudToVRML::setPathToSaveInvetorFile(std::string PathToSaveInvetorFile)
{
    this->PathToSaveInvetorFile=PathToSaveInvetorFile;
}

void PointCloudToVRML::getAbsolutPathToPCLFile(std::string &AbsolutPathToPCLFile)
{
    AbsolutPathToPCLFile=this->AbsolutPathToPCLFile;
}

void PointCloudToVRML::getPathToSaveInvetorFile(std::string &PathToSaveInvetorFile)
{
    PathToSaveInvetorFile=this->PathToSaveInvetorFile;
}

std::string PointCloudToVRML::getAbsolutPathToPCLFile()
{
    return this->AbsolutPathToPCLFile;
}

std::string PointCloudToVRML::getPathToSaveInvetorFile()
{
    return this->PathToSaveInvetorFile;
}

PointCloudToVRML::PointCloudToVRML()
{
}

int PointCloudToVRML::WriteToVRML()
{


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // Read a PCD file from disk.
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(this->getAbsolutPathToPCLFile(), *cloud) != 0)
    {
        std::cout<<"not a valid point cloud" <<std::endl;
        return -1;
    }

    //if you like to see the pointcloud uncomment this:
    /*
    pcl::visualization::CloudViewer viewerPlane("PointCloud");
    viewerPlane.showCloud(cloud);
    while (!viewerPlane.wasStopped())
    {
    }*/
    //you can create either a concave or convex hull, in the case of concav you must set alpha parameter
    //    pcl::ConcaveHull<pcl::PointXYZ> hull;
    //    hull.setAlpha(0.1);
    double start_link_ini = omp_get_wtime();
    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(cloud);
    hull.setDimension(3);
    pcl::PolygonMesh pcl_polygonmesh;
    hull.reconstruct(pcl_polygonmesh);

    /////////////////////////////////////////////////////conversion from pcl::PolygonMesh to vtk//////////////////////////////////////////////////////////

    vtkSmartPointer<vtkPolyData> vtk_triangles_out;
    pcl::VTKUtils::mesh2vtk (pcl_polygonmesh, vtk_triangles_out);

    /////////////////////////////////////////////////////conversion from vtkPolydata to inventor//////////////////////////////////////////////////////////
    double end_link_ini = omp_get_wtime();


    vtkIVWriter *writer=vtkIVWriter::New();
    writer->SetInput(vtk_triangles_out);
    writer->SetFileName(this->getPathToSaveInvetorFile().c_str());
    writer->Write();
    writer->Delete();
    std::cout<<"link_ini took  "<< end_link_ini-start_link_ini<<std::endl;

}
