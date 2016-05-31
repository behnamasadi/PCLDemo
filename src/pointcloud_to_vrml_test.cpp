#include "pointcloud_to_vrml.h"

int test_pointcloud_to_inventor(int argc, char** argv)
{

    std::string AbsolutPathToPCLFile="../pcd_files/robot1.pcd";
    std::string PathToSaveInvetorFile="robot.iv";

    PointCloudToVRML* point_cloud_to_VRML_object=new PointCloudToVRML();
    point_cloud_to_VRML_object->setAbsolutPathToPCLFile(AbsolutPathToPCLFile);
    point_cloud_to_VRML_object->setPathToSaveInvetorFile(PathToSaveInvetorFile);
    point_cloud_to_VRML_object->WriteToVRML();

}









int main(int argc , char ** argv)
{
    test_pointcloud_to_inventor(argc,argv);
}
