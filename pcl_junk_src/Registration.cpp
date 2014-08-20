// http://www.pcl-users.org/Very-poor-registration-results-td3569265.html
//Here is the basic outline of my code:
//-get 2 point clouds (loaded from pcd files)
//-filter out NaN values
//-downsample them (voxel grid)
//-compute normals
//-compute FPFH features
//-get an initial transformation using SAC
//-merge the full clouds using the transformation computed on the downsampled clouds

#include <iostream>
#include <time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include "pcl/point_cloud.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/features/fpfh.h"
//------------------------------------------------

using namespace pcl;
using namespace std;


const double FILTER_LIMIT = 1000.0;
const int MAX_SACIA_ITERATIONS = 500;

//units are meters:
//const float VOXEL_GRID_SIZE = 0.03;
//const double NORMALS_RADIUS = 0.04;
//const double FEATURES_RADIUS = 0.04;
//const double SAC_MAX_CORRESPONDENCE_DIST = 0.001;
//


const float VOXEL_GRID_SIZE = 0.01;
const double NORMALS_RADIUS = 0.03;
const double FEATURES_RADIUS = 0.08;
const double SAC_MAX_CORRESPONDENCE_DIST = 1;
const double SAC_MIN_SAMPLE_DIST = 0.01;

void filterCloud( PointCloud<PointXYZRGB>::Ptr );
PointCloud<Normal>::Ptr getNormals( PointCloud<PointXYZRGB>::Ptr incloud );
PointCloud<FPFHSignature33>::Ptr getFeatures( PointCloud<PointXYZRGB>::Ptr incloud, PointCloud<Normal>::Ptr normals );
void view( PointCloud<pcl::PointXYZRGB> & cloud );
SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33>
        align( PointCloud<PointXYZRGB>::Ptr c1, PointCloud<PointXYZRGB>::Ptr c2,
                   PointCloud<FPFHSignature33>::Ptr features1, PointCloud<FPFHSignature33>::Ptr features2 );

int main(int argc, char** argv)
{
	time_t starttime = time(NULL);

	cout << "Loading clouds...";
	cout.flush();

	//open the clouds
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);

//	pcl::io::loadPCDFile("cornercloud1", *cloud1);
//	pcl::io::loadPCDFile("cornercloud2", *cloud2);

	pcl::io::loadPCDFile(argv[1], *cloud1);
	pcl::io::loadPCDFile(argv[2], *cloud2);

	cout << "done. Time elapsed: " << time(NULL) - starttime << " seconds\nFiltering input clouds...";
	cout.flush();

	//pass both through filters first
	filterCloud( cloud1 );
	filterCloud( cloud2 );

	cout << "done. Time elapsed: " << time(NULL) - starttime << " seconds\nDownsampling the clouds...";
	cout.flush();

	//downsample the clouds, but store the downsampled clouds seperately
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1ds (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2ds (new pcl::PointCloud<pcl::PointXYZRGB>);
	VoxelGrid<PointXYZRGB> vox_grid;
	vox_grid.setLeafSize( VOXEL_GRID_SIZE, VOXEL_GRID_SIZE, VOXEL_GRID_SIZE );
	vox_grid.setInputCloud( cloud1 );
	vox_grid.filter( *cloud1ds );

	vox_grid.setInputCloud( cloud2 );
	vox_grid.filter( *cloud2ds );

	cout << "done. Time elapsed: " << time(NULL) - starttime << " seconds\nCalculating normals...";
	cout.flush();

	//compute normals
	pcl::PointCloud<pcl::Normal>::Ptr normals1 = getNormals( cloud1ds );
	pcl::PointCloud<pcl::Normal>::Ptr normals2 = getNormals( cloud2ds );

	cout << "done. Time elapsed: " << time(NULL) - starttime << " seconds\nComputing local features...";
	cout.flush();

	//compute local features
	PointCloud<FPFHSignature33>::Ptr features1 = getFeatures( cloud1ds, normals1 );
	PointCloud<FPFHSignature33>::Ptr features2 = getFeatures( cloud2ds, normals2 );

	cout << "done. Time elapsed: " << time(NULL) - starttime << " seconds\nComputing initial alignment using SAC...";
	cout.flush();

	//Get an initial estimate for the transformation using SAC
	//returns the transformation for cloud2 so that it is aligned with cloud1
	SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33> sac_ia = align( cloud1ds, cloud2ds, features1, features2 );
	Eigen::Matrix4f	init_transform = sac_ia.getFinalTransformation();
	transformPointCloud( *cloud2, *cloud2, init_transform );
	pcl::PointCloud<pcl::PointXYZRGB> final = *cloud1;
	final += *cloud2;

	cout << "done. Time elapsed: " << time(NULL) - starttime << " seconds\n";
	cout << "Opening aligned cloud; will return when viewer window is closed.";
	cout.flush();

	view(final);

	return 1;
}

//computes the transformation for cloud2 so that it is transformed so that it is aligned with cloud1
SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33>
         align( PointCloud<PointXYZRGB>::Ptr cloud1, PointCloud<PointXYZRGB>::Ptr cloud2,
                        PointCloud<FPFHSignature33>::Ptr features1, PointCloud<FPFHSignature33>::Ptr features2 ) {

         SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33> sac_ia;
         Eigen::Matrix4f final_transformation;
         sac_ia.setInputCloud( cloud2 );
         sac_ia.setSourceFeatures( features2 );
         sac_ia.setInputTarget( cloud1 );
         sac_ia.setTargetFeatures( features1 );
         sac_ia.setMaximumIterations( MAX_SACIA_ITERATIONS );
         PointCloud<PointXYZRGB> finalcloud;
         sac_ia.align( finalcloud );
         return sac_ia;
}

PointCloud<FPFHSignature33>::Ptr getFeatures( PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr normals ) {

        PointCloud<FPFHSignature33>::Ptr features = PointCloud<FPFHSignature33>::Ptr (new PointCloud<FPFHSignature33>);
        search::KdTree<PointXYZRGB>::Ptr search_method_ptr = search::KdTree<PointXYZRGB>::Ptr (new search::KdTree<PointXYZRGB>);
        FPFHEstimation<PointXYZRGB, Normal, FPFHSignature33> fpfh_est;
        fpfh_est.setInputCloud( cloud );
        fpfh_est.setInputNormals( normals );
        fpfh_est.setSearchMethod( search_method_ptr );
        fpfh_est.setRadiusSearch( FEATURES_RADIUS );
        fpfh_est.compute( *features );
        return features;
}

PointCloud<Normal>::Ptr getNormals( PointCloud<PointXYZRGB>::Ptr incloud ) {

	PointCloud<Normal>::Ptr normalsPtr = PointCloud<Normal>::Ptr (new PointCloud<Normal>);
	NormalEstimation<PointXYZRGB, Normal> norm_est;
	norm_est.setInputCloud( incloud );
	norm_est.setRadiusSearch( NORMALS_RADIUS );
	norm_est.compute( *normalsPtr );
	return normalsPtr;
}

void filterCloud( PointCloud<PointXYZRGB>::Ptr pc ) {

	pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(pc);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0, FILTER_LIMIT);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(0, FILTER_LIMIT);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, FILTER_LIMIT);
    pass.filter(*pc);

}

void view( PointCloud<pcl::PointXYZRGB> & cloud )
{
	pcl::visualization::CloudViewer viewer1("Cloud Viewer");
	viewer1.showCloud( cloud.makeShared() );
	while( !viewer1.wasStopped() );

	return;
}


