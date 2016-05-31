#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/file_io.h>
#include <Eigen/Geometry>
#include <pcl/registration/icp.h>
#include <boost/thread/thread.hpp>

#include <pcl/features/normal_3d.h>

#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>

#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>

#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/impl/sift_keypoint.hpp>

#include <pcl/features/integral_image_normal.h>


#include <pcl/features/pfh.h>
//#include "pcl/features/fpfh.h"

#include <pcl/visualization/histogram_visualizer.h>


#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_estimation.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/features/narf_descriptor.h>

//#include <pcl/keypoints/harris_keypoint3D.h>

#include <pcl/common/eigen.h>

#include <pcl/filters/filter.h>

#include <pcl/features/fpfh_omp.h>

#include <pcl/io/ply_io.h>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

using namespace Eigen;
using namespace std;
using namespace pcl;


//convert to poitcloud:
//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr
// *cloud_ptr;
//convert to pointcloudpointer:
//pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud( &mycloud);

template<typename to, typename from>
to lexical_cast(from const &x)
{
	std::stringstream os;
	to ret;
	os << x;
	os >> ret;
	return ret;
}

int RandomColorGenerator()
{
	int color;

	/* initialize random seed: */
//	srand ( time(NULL) );

	//rand()  return An integer value between 0 and RAND_MAX.

	/* generate secret number: */
	color = rand() % 255;
	return color;
}

template<typename PointT>
void SimpleVisualizer(std::vector< boost::shared_ptr<pcl::PointCloud<PointT> >  > PointCloudPtrVector)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	int vp_1,vp_2;
//	viewer->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
//	viewer->createViewPort (0.5, 0, 1.0, 1.0, vp_2);
//	cout<<vp_1 <<" " <<vp_2 <<endl;
//	cout<<"PointCloudPtrVector.size():" <<PointCloudPtrVector.size() <<endl;
	int Red,Blue,Green;
	for(size_t i=0;i<PointCloudPtrVector.size();i++)
	{
		Red=RandomColorGenerator();
		Green=RandomColorGenerator();
		Blue=RandomColorGenerator();
//		std::cout<< "Red: " << Red <<std::endl;
//		std::cout<<"Green: " <<Green <<std::endl;
//		std::cout<<"Blue: " <<Blue <<std::endl;
		pcl::visualization::PointCloudColorHandlerCustom<PointT >  tgt_h ( PointCloudPtrVector.at(i),Red,Green,Blue   );
		viewer->addPointCloud<PointT> (PointCloudPtrVector.at(i) ,tgt_h ,lexical_cast<std::string>(i), 0);
	}
//	viewer->removePointCloud();
	while (!viewer->wasStopped ())
	{
	  viewer->spinOnce (100);
	  boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}

template<typename PointT>
void SimpleCloudViewer(boost::shared_ptr<pcl::PointCloud<PointT> > mycloud_ptr)
{
	pcl::visualization::CloudViewer myviewer("title:");
    myviewer.showCloud( mycloud_ptr);
	while(!myviewer.wasStopped() )
	{
	}
}

void SimpleCloudViewer_Test(const char ** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile(argv[1],*cloudrgb_ptr);
    std::cout<<"Size of cloud is: " <<cloudrgb_ptr->size() <<std::endl;
	SimpleCloudViewer<pcl::PointXYZRGB>(cloudrgb_ptr);
}

template<typename PointT>
void CreatePointCloud(boost::shared_ptr<pcl::PointCloud<PointT> >& cloud_ptr)
{
    for(int i=0;i<100;i++)
	{
		for(int j=0;j<100;j++)
		{
			pcl::PointXYZ pt;
			pt.x=i;
			pt.y=0;
			pt.z=j;
			cloud_ptr->push_back(pt);
		}
	}
}

void CreatePointCloud_Test()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr mycloud(new pcl::PointCloud<pcl::PointXYZ>);
	CreatePointCloud<pcl::PointXYZ>(mycloud);
	SimpleCloudViewer  (  mycloud);
}

template<typename PointT>
void CreatePointCloudPointer( boost::shared_ptr<pcl::PointCloud<PointT> >& cloud_ptr)
{
    for(int i=0;i<100;i++)
    {
    	for(int j=0;j<100;j++)
    	{
    		pcl::PointXYZ pt;
    		pt.x=i;
    		pt.y=0;
        	pt.z=j;
//        	pt.r=200;
//        	pt.g=50;
//        	pt.b=100;
        	cloud_ptr->push_back(pt);
    	}
    }
}

void CreatePointCloudPointer_Test()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr mycloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	CreatePointCloudPointer<pcl::PointXYZ>(mycloud_ptr);
	SimpleCloudViewer<pcl::PointXYZ>(  mycloud_ptr);
	pcl::io::savePCDFileASCII( "plan.pcd",  *mycloud_ptr);
}

template<typename PointT>
void DownSamplingPointCloudUsingVoxelGridFilter(boost::shared_ptr<pcl::PointCloud<PointT> > cloud_in,float LeafSize,boost::shared_ptr<pcl::PointCloud<PointT> >& cloud_out)
{
	pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud (cloud_in);
	sor.setLeafSize (LeafSize, LeafSize, LeafSize);
	sor.filter (*cloud_out);
}

void DownSamplingPointCloudUsingVoxelGridFilter_Test(const char ** argv)
{
//	for example ./sandbox table_scene_mug_stereo_textured.pcd 0.01
	std::string strLeafSize=argv[2];
	float LeafSize;
	LeafSize=lexical_cast<float>(strLeafSize);
	pcl::PointCloud<pcl::PointXYZ>::Ptr down_sampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(argv[1],*cloud_ptr);
	DownSamplingPointCloudUsingVoxelGridFilter(cloud_ptr,LeafSize,down_sampled_cloud);
	std::cout<< cloud_ptr->size() <<endl;
	std::cout<< down_sampled_cloud->size() <<endl;
	SimpleCloudViewer<pcl::PointXYZ>(down_sampled_cloud);
}

void MakePartialViewSnapShot()
{
	vtkSmartPointer<vtkPolyData> model;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	CreatePointCloudPointer<pcl::PointXYZ>(cloud);

	pcl::visualization::PCLVisualizer viz;
	viz.initCameraParameters();
	viz.addModelFromPolyData(model);
	viz.setRepresentationToSurfaceForAllActors();
	viz.renderView(640, 480, cloud);
}

template<typename PointT>
void FindTransformationMatrix(boost::shared_ptr<pcl::PointCloud<PointT> >
 cloud_in, boost::shared_ptr<pcl::PointCloud<PointT> > cloud_out,double MaxCorrespondenceDistance,int MaximumIterations,Eigen::Matrix4f& EstimatedTransformationMatrix )
{
	//The transformation is estimated based on Singular Value Decomposition (SVD).
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputCloud( cloud_in);
	icp.setInputTarget( cloud_out);
	boost::shared_ptr<pcl::PointCloud<PointT> > Final(new  pcl::PointCloud<PointT>);
//	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.setMaxCorrespondenceDistance( MaxCorrespondenceDistance);
	icp.setMaximumIterations(MaximumIterations);
	icp.align( *Final);
	EstimatedTransformationMatrix=icp.getFinalTransformation();
	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
}

void FindTransformationMatrix_Test(const char ** argv)
{
/*//	example of run ./sandbox bunny.pcd
	Vector3f offset;
	offset(0)=0.0;
	offset(1)=0.0;
	offset(2)=0.0;
	float alpha, beta, gamma;
//	alpha=M_PI/6;
//	beta=M_PI/3;
//	gamma=M_PI/4;
//

	alpha=pcl::deg2rad(0.0f);
	beta=pcl::deg2rad(90.0f);
	gamma=pcl::deg2rad(0.0f);

	cout<<alpha <<endl;
	cout<<beta <<endl;
	cout<<gamma <<endl;

	float X,Y,Z;
	X=0;
	Y=0;
	Z=0;
	Eigen::Affine3f newt;

	pcl::getTransformation(X,Y,Z, alpha,beta,gamma,newt);


	Matrix3f m;
	m = AngleAxisf(gamma, Vector3f::UnitZ()) * AngleAxisf(beta,  Vector3f::UnitY())*  AngleAxisf(alpha,Vector3f::UnitX()) ;
	Matrix4f  	transform;
	transform(0,0)=m(0,0);
	transform(0,1)=m(0,1);
	transform(0,2)=m(0,2);
	transform(1,0)=m(1,0);
	transform(1,1)=m(1,1);
	transform(1,2)=m(1,2);
	transform(2,0)=m(2,0);
	transform(2,1)=m(2,1);
	transform(2,2)=m(2,2);


	transform(0,3)=offset(0);
	transform(1,3)=offset(1);
	transform(2,3)=offset(2);

	transform(3,0)=0.0;
	transform(3,1)=0.0;
	transform(3,2)=0.0;
	transform(3,3)=1.0;


	std::cout <<"Desired Transform Matrix is: "  << std::endl;
	std::cout << Eigen::Matrix4f(newt ) << std::endl;

	std::string PathToPCDFile=argv[1];
	std::string PathToPCDOutputFile =PathToPCDFile.substr(0,PathToPCDFile.find('.'));

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(PathToPCDFile, *cloud_in);
//	pcl::io::savePCDFileASCII( "pcd_files/cloud_in.pcd",  *cloud_in);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::transformPointCloud 	( 	*cloud_in, *cloud_out,transform);
	pcl::transformPointCloud 	( 	*cloud_in, *cloud_out, newt);
	pcl::io::savePCDFileASCII(PathToPCDOutputFile+ "_transformed.pcd",  *cloud_out);
	double MaxCorrespondenceDistance=10000000.0;
	int MaximumIterations=50;
	Eigen::Matrix4f EstimatedTransformationMatrix;
	FindTransformationMatrix( cloud_in,  cloud_out,MaxCorrespondenceDistance ,MaximumIterations ,EstimatedTransformationMatrix);
	std::cout <<"Estimated Transformation Matrix is: "<<endl;
	std::cout<<EstimatedTransformationMatrix<< std::endl;




	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_transformed(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud 	( 	*cloud_in, *cloud_in_transformed,EstimatedTransformationMatrix);


	std::vector< pcl::PointCloud<pcl::PointXYZ >::Ptr > PointCloudPtrVector;
	PointCloudPtrVector.push_back(cloud_in);
	PointCloudPtrVector.push_back(cloud_in_transformed);
//	PointCloudPtrVector.push_back(cloud_out);
	SimpleVisualizer<pcl::PointXYZ>(PointCloudPtrVector);


	Eigen::Affine3f f(newt);
	float   x,y,z,roll,pitch,yaw;




	pcl::getTranslationAndEulerAngles( f,x,y,z,roll,pitch,yaw);


	cout<<"x " <<x<<endl;
	cout<<"y " <<y <<endl;
	cout<<"z " <<z<<endl;

	cout<<"roll " <<roll<<endl;
	cout<<"pitch " <<pitch<<endl;
	cout<<"yaw " <<yaw<<endl;*/


	float roll, pitch,yaw;
	float x,y,z;

	x=1.0;
	y=0.3;
	z=0.4;


	roll=pcl::deg2rad(45.0f);
	pitch=pcl::deg2rad(90.0f);
	yaw=pcl::deg2rad(30.0f);


	Eigen::Affine3f transformation;
	pcl::getTransformation(x,y,z,roll, pitch,yaw,transformation);
	std::cout <<"Desired Transform Matrix is: "  << std::endl;
	std::cout << transformation.matrix()  << std::endl;
	std::string PathToPCDFile=argv[1];

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(PathToPCDFile, *cloud_in);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_transformed(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud 	( 	*cloud_in, *cloud_in_transformed,transformation);




	double MaxCorrespondenceDistance=10000000.0;
	int MaximumIterations=50;
	Eigen::Matrix4f EstimatedTransformationMatrix;
	FindTransformationMatrix( cloud_in,  cloud_in_transformed,MaxCorrespondenceDistance ,MaximumIterations ,EstimatedTransformationMatrix);
	std::cout <<"Estimated Transformation Matrix is: "<<endl;
	std::cout<<EstimatedTransformationMatrix<< std::endl;



	std::vector< pcl::PointCloud<pcl::PointXYZ >::Ptr > PointCloudPtrVector;
	PointCloudPtrVector.push_back(cloud_in);
	PointCloudPtrVector.push_back(cloud_in_transformed);
	SimpleVisualizer<pcl::PointXYZ>(PointCloudPtrVector);

	Eigen::Affine3f f(EstimatedTransformationMatrix);



	float ROLL,PITCH,YAW;
	float X,Y,Z;



	pcl::getTranslationAndEulerAngles( transformation,X,Y,Z,ROLL,PITCH,YAW);


	cout<<"X " <<X <<endl;
	cout<<"Y " <<Y <<endl;
	cout<<"Z " <<Z<<endl;

	cout<<"ROLL " <<pcl::rad2deg( ROLL)<<endl;
	cout<<"PITCH " <<pcl::rad2deg(PITCH)<<endl;
	cout<<"YAW " <<pcl::rad2deg(YAW)<<endl;


}

void SimpleVisualizer_Test(const char ** argv)
{
//	pcl::visualization::PCLVisualizer::updatePointCloud

	std::string str_cloud_in =argv[1];
	std::string str_cloud_out =argv[2];
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(str_cloud_in,*cloud_in);


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(str_cloud_out, *cloud_out);


	std::vector< pcl::PointCloud<pcl::PointXYZ >::Ptr > PointCloudPtrVector;
	PointCloudPtrVector.push_back(cloud_in);
	PointCloudPtrVector.push_back(cloud_out);
	SimpleVisualizer<pcl::PointXYZ>(PointCloudPtrVector);
}

template<typename PointT>
void CreateRangeImage(boost::shared_ptr<pcl::PointCloud<PointT> > & pointCloud,pcl::RangeImage &rangeImage,Eigen::Affine3f scene_sensor_pose)
{
//http://pointclouds.org/documentation/tutorials/range_image_creation.php#range-image-creation
	  // We now want to create a range image from the above point cloud, with a 1deg angular resolution
	  //The angular resolution is supposed to be 1 degree, meaning the beams represented by neighboring pixels differ by one degree.
//	  float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
	  float angular_resolution = 0.1f;

	  angular_resolution = pcl::deg2rad (angular_resolution);

	  //maxAngleWidth=360 and maxAngleHeight=180 mean that the range sensor we are simulating has a complete 360 degree view of the surrounding.
	  //You can always use this setting, since the range image will be cropped to only the areas where something was observed automatically.
	  //Yet you can save some computation by reducing the values.
	  //E.g. for a laser scanner with a 180 degree view facing forward, where no points behind the sensor can be observed,
	  //maxAngleWidth=180 is enough.




	  float maxAngleWidth     =  pcl::deg2rad (360.0f);  // 360.0 degree in radians
	  float maxAngleHeight    = pcl::deg2rad (180.0f);  // 180.0 degree in radians



//	  coordinate_frame=CAMERA_FRAME tells the system that x is facing right, y downwards and the z axis is forward.
//	  An alternative would be LASER_FRAME, with x facing forward, y to the left and z upwards.
//	  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	  pcl::RangeImage::CoordinateFrame coordinate_frame =pcl::RangeImage::LASER_FRAME;

//	  For noiseLevel=0 the range image is created using a normal z-buffer.
//	  Yet if you want to average over points falling in the same cell you can use a higher value. 0.05 would mean,
//	  that all point with a maximum distance of 5cm to the closest point are used to calculate the range.

//	  If minRange is greater 0 all points that are closer will be ignored.
//	  borderSize greater 0 will leave a border of unobserved points around the image when cropping it.
	  float noiseLevel=0.00;
	  float minRange = 0.0f;
	  int borderSize = 1;
	  rangeImage.createFromPointCloud( *pointCloud, angular_resolution, maxAngleWidth, maxAngleHeight,scene_sensor_pose, coordinate_frame, noiseLevel, minRange, borderSize);
}


template<typename PointT>
void VisualizeRangeImage(pcl::RangeImage range_image, boost::shared_ptr<  pcl::PointCloud<PointT> > & point_cloud)
{
	pcl::visualization::PCLVisualizer viz;
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//	viewer->setBackgroundColor (0, 0, 0);
	viz.setBackgroundColor (0, 0, 0);
//	viewer->addPointCloud<PointT> (point_cloud);
	viz.addPointCloud<PointT> (point_cloud);
	pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
	range_image_widget.showRangeImage (range_image);
	Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
	bool live_update=true;
	//--------------------
	// -----Main loop-----
	//--------------------
	while (!viz.wasStopped ())
	{
		range_image_widget.spinOnce ();
		viz.spinOnce ();
		pcl_sleep (0.01);
		if (live_update)
		{
			scene_sensor_pose = viz.getViewerPose();
			CreateRangeImage(point_cloud,range_image,scene_sensor_pose);
			range_image_widget.showRangeImage (range_image);
		}
	}
}

void VisualizeRangeImage_Test(const char ** argv)
{
	pcl::RangeImage range_image;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile(argv[1], *cloud_ptr);
	Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
	CreateRangeImage<pcl::PointXYZRGB>(cloud_ptr,range_image,scene_sensor_pose);
	VisualizeRangeImage<pcl::PointXYZRGB>(range_image,cloud_ptr);
}

template<typename PointT>
void ExtractNARFkeypoints(pcl::RangeImage range_image, float support_size,pcl::PointCloud<int>& keypoint_indices_out)
{
	// --------------------------------
	// -----Extract NARF keypoints-----
	// --------------------------------

	pcl::NarfKeypoint narf_keypoint_detector;
	pcl::RangeImageBorderExtractor range_image_border_extractor;
	narf_keypoint_detector.setRangeImageBorderExtractor(&range_image_border_extractor);
	narf_keypoint_detector.setRangeImage (&range_image);
	narf_keypoint_detector.getParameters ().support_size = support_size;
	//narf_keypoint_detector.getParameters ().add_points_on_straight_edges = true;
	//narf_keypoint_detector.getParameters ().distance_for_additional_points = 0.5;

	narf_keypoint_detector.compute (keypoint_indices_out);
	std::cout << "Found "<<keypoint_indices_out.points.size ()<<" key points.\n";

	// -------------------------------------
	// -----Show keypoints in 3D viewer-----
	// -------------------------------------
	boost::shared_ptr< pcl::PointCloud<PointT> >  keypoints_ptr (new pcl::PointCloud<PointT>);
//	boost::shared_ptr< pcl::PointCloud<PointT> > keypoints (keypoints_ptr);
	keypoints_ptr->points.resize (keypoint_indices_out.points.size ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints1_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
	keypoints1_xyzrgb->resize(keypoint_indices_out.points.size());
	for (size_t i=0; i<keypoint_indices_out.points.size (); ++i)
	{
	  keypoints_ptr->points[i].getVector3fMap () = range_image.points[keypoint_indices_out.points[i]].getVector3fMap ();
	  keypoints1_xyzrgb->points[i].getVector3fMap () =range_image.points[keypoint_indices_out.points[i]].getVector3fMap();
	  keypoints1_xyzrgb->points.at(i).r=255;
	  keypoints1_xyzrgb->points.at(i).g=0;
	  keypoints1_xyzrgb->points.at(i).b=0;
	}


//	pcl::visualization::PCLVisualizer viewer ("3D Viewer");
//	viewer.setBackgroundColor (1, 1, 1);
//	pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZ> keypoints_color_handler ( keypoints_ptr, 0, 255, 0);
//	viewer.addPointCloud<pcl::PointXYZ> (keypoints_ptr, keypoints_color_handler, "keypoints");
//	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");


//	SimpleCloudViewer <pcl::PointXYZRGB> (keypoints1_xyzrgb);
}

void ExtractNARFFeatures(pcl::RangeImage range_image,float support_size, pcl::PointCloud<int>& keypoint_indices,
		pcl::PointCloud<pcl::Narf36>::Ptr & result)
{
	bool rotation_invariant = true;
	std::vector<int> keypoint_indices2;
	keypoint_indices2.resize (keypoint_indices.points.size ());
	for (unsigned int i=0; i<keypoint_indices.size (); i++)
	{
		keypoint_indices2[i]=keypoint_indices.points[i];
	}
	pcl::NarfDescriptor narf_descriptor;
	narf_descriptor.setRangeImage(&range_image, &keypoint_indices2);
	narf_descriptor.getParameters ().support_size = support_size;
	narf_descriptor.getParameters ().rotation_invariant = rotation_invariant;
//	pcl::PointCloud<pcl::Narf36> narf_descriptors;
	narf_descriptor.compute (*result);
//	cout << "Extracted "<<narf_descriptors.size ()<<" descriptors for " <<keypoint_indices.points.size ()<< " keypoints.\n";
	cout<<result->size()  <<endl;
}

void ExtractNARFkeypoints_Test(const char ** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile(argv[1], *cloud_ptr);
	pcl::RangeImage range_image;
	Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
	CreateRangeImage(cloud_ptr,range_image,scene_sensor_pose);
	float 	support_size = 0.001f;
	pcl::PointCloud<int> keypoint_indices;
	ExtractNARFkeypoints<pcl::PointXYZRGB>( range_image,support_size,keypoint_indices );
}

void ExtractNARFFeatures_Test(const char ** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile(argv[1], *cloud_ptr);
	pcl::RangeImage range_image;
	Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
	CreateRangeImage(cloud_ptr,range_image,scene_sensor_pose);
	float 	support_size = 0.001f;
	pcl::PointCloud<int> keypoint_indices;
	ExtractNARFkeypoints<pcl::PointXYZRGB>( range_image,support_size,keypoint_indices );
//	pcl::NarfDescriptor narf_descriptor;
	pcl::PointCloud<pcl::Narf36>::Ptr narf_descriptors1 (new pcl::PointCloud<pcl::Narf36>);

	ExtractNARFFeatures(range_image,support_size,keypoint_indices ,narf_descriptors1 );

}

template <typename PointT>
//PointT will be something like  pcl::PFHSignature125
void FindFeatureCorrespondences (boost::shared_ptr< pcl::PointCloud<PointT> >  &source_descriptors,
									boost::shared_ptr< pcl::PointCloud< PointT > > &target_descriptors,
                              std::vector<int> &correspondences_out, std::vector<float> &correspondence_scores_out)
{
	// Resize the output vector
	correspondences_out.resize (source_descriptors->size ());
	correspondence_scores_out.resize (source_descriptors->size ());

	// Use a KdTree to search for the nearest matches in feature space
	//pcl::search::KdTree<pcl::PFHSignature125> descriptor_kdtree;
	boost::shared_ptr< pcl::search::KdTree<PointT> >  descriptor_kdtree(new pcl::search::KdTree<PointT>);
	descriptor_kdtree->setInputCloud (target_descriptors);

	// Find the index of the best match for each keypoint, and store it in "correspondences_out"
	const int k = 1;
	std::vector<int> k_indices (k);
	std::vector<float> k_squared_distances (k);
	for (size_t i = 0; i < source_descriptors->size (); ++i)
	{
//		Search for k-nearest neighbors for the given query point.
		descriptor_kdtree->nearestKSearch (*source_descriptors, i, k, k_indices, k_squared_distances);
		correspondences_out[i] = k_indices[0];
		correspondence_scores_out[i] = k_squared_distances[0];
	}
}

void CreateCorrespondences(boost::shared_ptr< pcl::Correspondences > &correspondences,
		std::vector<int> &correspondences_descriptors,
		std::vector<float> &correspondences_scores)
{
	for(size_t i=0;i<correspondences_descriptors.size();i++ )
	{
		pcl::Correspondence correspondence_entry;
		correspondence_entry.distance=correspondences_scores.at(i);
//		index_query : Index of the query (source) point.
		correspondence_entry.index_query=i;
//		index_match : Index of the matching (target) point.
		correspondence_entry.index_match=correspondences_descriptors.at(i);
		correspondences->push_back(correspondence_entry);
	}
}

void MedianDistance_Correspondence_Rejector(double MedianFactor,
		boost::shared_ptr<pcl::Correspondences> &correspondences,
		boost::shared_ptr<pcl::Correspondences> &correspondences_after_median)
{

	pcl::registration::CorrespondenceRejectorMedianDistance MedianDistanceCorrespondenceRejector;
	//MedianFactor: Points with distance greater than median times factor   will be rejected
	MedianDistanceCorrespondenceRejector.setMedianFactor (MedianFactor);
	MedianDistanceCorrespondenceRejector.setInputCorrespondences (correspondences);
	MedianDistanceCorrespondenceRejector.getCorrespondences ( *correspondences_after_median);
	return;
}

template <typename PointT>
void VisualizeCorrespondencesNARF
(
				const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> > points1,
		        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints1,
//				const boost::shared_ptr< pcl::PointCloud<PointT> > keypoints1,
		        const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> > points2,
		        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints2,
//		        const boost::shared_ptr< pcl::PointCloud<PointT> > keypoints2,
		        boost::shared_ptr<pcl::Correspondences> &correspondences)
{

	//	 We want to visualize two clouds side-by-side, so do to this, we'll make copies of the clouds and transform them
	//	 by shifting one to the left and the other to the right.  Then we'll draw lines between the corresponding points
	//
	//	 Create some new point clouds to hold our transformed data
		boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> > points_left (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_left (new pcl::PointCloud<pcl::PointXYZRGB>);
		boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> > points_right(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_right (new pcl::PointCloud<pcl::PointXYZRGB>);

	//	 Shift the first clouds' points to the left
	//	const Eigen::Vector3f translate (0.0, 0.0, 0.3);
		const Eigen::Vector3f translate (0.4, 0.0, 0.0);
		const Eigen::Quaternionf no_rotation (0, 0, 0, 0);
		pcl::transformPointCloud (*points1, *points_left, -translate, no_rotation);
		pcl::transformPointCloud (*keypoints1, *keypoints_left, -translate, no_rotation);

		// Shift the second clouds' points to the right
		pcl::transformPointCloud (*points2, *points_right, translate, no_rotation);
		pcl::transformPointCloud (*keypoints2, *keypoints_right, translate, no_rotation);

		// Add the clouds to the vizualizer
		pcl::visualization::PCLVisualizer viz;
		viz.addPointCloud (points_left, "points_left");
		viz.addPointCloud (points_right, "points_right");


		for(size_t i=0;i<correspondences->size();i++)
		{
			/*const pcl::PointWithScale & p_left = keypoints_left->points[correspondences->at(i).index_query  ];
			const pcl::PointWithScale & p_right = keypoints_right->points[ correspondences->at(i).index_match  ];
			*/
			const pcl::PointXYZRGB & p_left = keypoints_left->points[correspondences->at(i).index_query  ];
			const pcl::PointXYZRGB & p_right = keypoints_right->points[ correspondences->at(i).index_match  ];

			// Generate a random (bright) color
			double r = (rand() % 100);
			double g = (rand() % 100);
			double b = (rand() % 100);
			double max_channel = std::max (r, std::max (g, b));
			r /= max_channel;
			g /= max_channel;
			b /= max_channel;

			// Generate a unique string for each line
			std::stringstream ss ("line");
			ss << i;

			// Draw the line
			viz.addLine (p_left, p_right, r, g, b, ss.str ());

		}

		viz.spin ();
		return;
}

template <typename PointT>
void RANSAC_Correspondence_Rejector(boost::shared_ptr<pcl::PointCloud<PointT> >input_cloud,
		boost::shared_ptr<pcl::PointCloud<PointT> >target_cloud,
		boost::shared_ptr<pcl::Correspondences> &correspondences,
		double threshold,int MaxIterations,
		boost::shared_ptr<pcl::Correspondences> &correspondences_after_ransac,
		Eigen::Matrix4f  &transformation)
{
	pcl::registration::CorrespondenceRejectorSampleConsensus< PointT > correspondence_rejector;
	correspondence_rejector.setInlierThreshold(threshold);
	correspondence_rejector.setInputCloud(input_cloud);
	correspondence_rejector.setTargetCloud(target_cloud);
	correspondence_rejector.setMaxIterations(MaxIterations);
	correspondence_rejector.setInputCorrespondences( correspondences);
    correspondence_rejector.getCorrespondences(*correspondences_after_ransac);
    transformation = correspondence_rejector.getBestTransformation();
	return;
}

void NARFRegistration(std::string FullPathToFirstPCDFile, std::string FullPathToSecondPCDFile)
{
//	Create some new point clouds to hold our data
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points1 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled1 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Narf36>::Ptr narf_descriptors1 (new pcl::PointCloud<pcl::Narf36>);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Narf36>::Ptr narf_descriptors2 (new pcl::PointCloud<pcl::Narf36>);

	// Load the pair of point clouds

	pcl::io::loadPCDFile (FullPathToFirstPCDFile.c_str(), *points1);
	pcl::io::loadPCDFile (FullPathToSecondPCDFile.c_str(), *points2);


	std::cout<<"Size of first cloud: "<< points1->size() <<std::endl;
	std::cout<<"Size of second cloud: "<< points2->size() <<std::endl;

//	Downsample the cloud
	const float voxel_grid_leaf_size = 0.001;

	DownSamplingPointCloudUsingVoxelGridFilter<pcl::PointXYZRGB>(points1, voxel_grid_leaf_size, downsampled1);
	DownSamplingPointCloudUsingVoxelGridFilter<pcl::PointXYZRGB>(points2, voxel_grid_leaf_size, downsampled2);


	std::cout<<"Size of first cloud after downsampling: "<< downsampled1->size() <<std::endl;
	std::cout<<"Size of second cloud after downsampling: "<< downsampled2->size() <<std::endl;

	pcl::RangeImage range_image1,range_image2;
	Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
	CreateRangeImage(downsampled1,range_image1,scene_sensor_pose);
	CreateRangeImage(downsampled2,range_image2,scene_sensor_pose);

	cout<<"Size of first range image: "  <<range_image1.size()  <<endl;
	cout<<"Size of second range image: "  <<range_image2.size()  <<endl;


	float 	support_size = 0.001f;

	pcl::PointCloud<int> keypoint_indices1,keypoint_indices2;
	ExtractNARFkeypoints<pcl::PointXYZRGB>( range_image1,support_size,keypoint_indices1 );
	ExtractNARFkeypoints<pcl::PointXYZRGB>( range_image2,support_size,keypoint_indices2 );
	ExtractNARFFeatures(range_image1,support_size,keypoint_indices1 ,narf_descriptors1 );
	ExtractNARFFeatures(range_image2,support_size,keypoint_indices2 ,narf_descriptors2 );

	std::vector<int> correspondences_descriptors1_to_descriptors2;
	std::vector<float> correspondences_descriptors1_to_descriptors2_scores;
	FindFeatureCorrespondences<pcl::Narf36> (narf_descriptors1, narf_descriptors2, correspondences_descriptors1_to_descriptors2, correspondences_descriptors1_to_descriptors2_scores);



	boost::shared_ptr<pcl::Correspondences > all_correspondences(new pcl::Correspondences );
	CreateCorrespondences(all_correspondences,correspondences_descriptors1_to_descriptors2,correspondences_descriptors1_to_descriptors2_scores);
	cout<<"all_correspondences->size(): " << all_correspondences->size()  <<endl;
	double MedianFactor=1.0;

	boost::shared_ptr<pcl::Correspondences> correspondences_after_median(new pcl::Correspondences);
	MedianDistance_Correspondence_Rejector(MedianFactor,all_correspondences, correspondences_after_median );
	cout<<"correspondences_after_median->size(): " << correspondences_after_median->size() <<endl;
	double RANSAC_threshold=0.03;
	int RANSAC_MaxIterations=5000;
	boost::shared_ptr<pcl::Correspondences> correspondences_after_ransac(new pcl::Correspondences);
	Eigen::Matrix4f  transformation;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints1_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
	keypoints1_xyzrgb->resize(keypoint_indices1.points.size());
	for (size_t i=0; i<keypoint_indices1.points.size (); ++i)
	{

	  keypoints1_xyzrgb->points[i].getVector3fMap () =range_image1.points[keypoint_indices1.points[i]].getVector3fMap();
	  keypoints1_xyzrgb->points.at(i).r=255;
	  keypoints1_xyzrgb->points.at(i).g=0;
	  keypoints1_xyzrgb->points.at(i).b=0;
	}


//	SimpleCloudViewer <pcl::PointXYZRGB> (keypoints1_xyzrgb);



	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints2_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
	keypoints2_xyzrgb->resize(keypoint_indices2.points.size());
	for (size_t i=0; i<keypoint_indices2.points.size (); ++i)
	{

	  keypoints2_xyzrgb->points[i].getVector3fMap () =range_image2.points[keypoint_indices2.points[i]].getVector3fMap();
	  keypoints2_xyzrgb->points.at(i).r=255;
	  keypoints2_xyzrgb->points.at(i).g=0;
	  keypoints2_xyzrgb->points.at(i).b=0;
	}


	RANSAC_Correspondence_Rejector<pcl::PointXYZRGB>(keypoints1_xyzrgb, keypoints2_xyzrgb,correspondences_after_median,RANSAC_threshold, RANSAC_MaxIterations,  correspondences_after_ransac,transformation );
//	RANSAC_Correspondence_Rejector<pcl::PointXYZRGB>(keypoints1_xyzrgb, keypoints2_xyzrgb,all_correspondences,RANSAC_threshold, RANSAC_MaxIterations,  correspondences_after_ransac,transformation );
	cout<< "correspondences_after_ransac->size() "<< correspondences_after_ransac->size()<<endl;
	VisualizeCorrespondencesNARF <pcl::PointXYZRGB>(points1, keypoints1_xyzrgb, points2, keypoints1_xyzrgb,correspondences_after_ransac );

	cout<<transformation<<endl;

/*
	pcl::visualization::PCLVisualizer viz;
	viz.addPointCloud (points1, "points1");
//	viz.addPointCloud (points2, "points2");

	boost::shared_ptr< PointCloud<pcl::PointXYZRGB> > points2_transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud (*points2, *points2_transformed, transformation);

	viz.addPointCloud (points2_transformed, "points1_transformed");
	viz.spin ();*/

	return;
}

void NARFRegistration_Test(const char **argv)
{
//	./BasicOperation pcd_files/reg/robot1.pcd pcd_files/reg/robot2.pcd

	NARFRegistration(argv[1], argv[2]);
	return;
}

template< typename PointT>
void ExtractSIFTkeypoints(boost::shared_ptr< pcl::PointCloud<PointT> > cloud, float min_scale ,int n_octaves,
		int n_scales_per_octave ,float min_contrast,pcl::PointCloud<pcl::PointWithScale>::Ptr & result)
{
//	 Estimate the sift interest points using Intensity values from RGB values
//	pcl::SIFTKeypoint< pcl::PointXYZRGB, pcl::PointWithScale> sift;
	pcl::SIFTKeypoint< PointT, pcl::PointWithScale> sift;
	boost::shared_ptr< pcl::search::KdTree<PointT> >   tree(new pcl::search::KdTree<PointT> ());
	sift.setSearchMethod (tree);
	sift.setScales(min_scale, n_octaves, n_scales_per_octave);
	sift.setMinimumContrast(min_contrast);
	sift.setInputCloud( cloud);
	sift.compute(*result);
}

void ExtractSIFTkeypoints_Test(const char ** argv)
{
	std::string filename = argv[1];
	std::cout << "Reading " << filename << std::endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	if(pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename, *cloud) == -1) // load the file
	{
		PCL_ERROR ("Couldn't read file");
		return ;
	}
//	Parameters for sift computation
	const float min_scale = 0.1f;
	const int n_octaves = 6;
	const int n_scales_per_octave = 10;
	const float min_contrast = 0.5f;
	pcl::PointCloud<pcl::PointWithScale>::Ptr result(new pcl::PointCloud<pcl::PointWithScale>);
	ExtractSIFTkeypoints<pcl::PointXYZRGB>(cloud,min_scale, n_octaves, n_scales_per_octave , min_contrast,result);

//	Copying the pointwithscale to pointxyz so as visualize the cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZRGB>);
	copyPointCloud(*result, *cloud_temp);


	// Visualization of keypoints along with the original cloud
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> keypoints_color_handler (cloud_temp, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_color_handler (cloud, 255, 255, 0);
	viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
	viewer.addPointCloud(cloud, "cloud");
	viewer.addPointCloud(cloud_temp, keypoints_color_handler, "keypoints");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
	while(!viewer.wasStopped ())
	{
		viewer.spinOnce ();
	}
}

template<typename PointT>
void VisualizeNormals (const boost::shared_ptr< pcl::PointCloud<PointT> >   points,
                       const pcl::PointCloud<pcl::Normal>::Ptr normals)
{
  // Add the points and normals to the vizualizer
  pcl::visualization::PCLVisualizer viz;
  viz.addPointCloud (points, "points");
  viz.addPointCloudNormals<PointT, pcl::Normal> (points, normals, 1, 0.01, "normals");
  // Give control over to the visualizer
  viz.spin ();
  cout<<"visualizing normals done ..." <<endl;
}

/*
you can only use one of these two:
	NumberOfNeighborsUseToEstimateNormal
	RadiusSearch
one of them should  always has negative value
 */
template <typename PointT>
void NormalEstimat (boost::shared_ptr< pcl::PointCloud<PointT> >  cloud_in, int NumberOfNeighborsUseToEstimateNormal,float RadiusSearch,pcl::PointCloud<pcl::Normal>::Ptr& normals)
{
//	http://www.pointclouds.org/documentation/tutorials/normal_estimation.php
	pcl::NormalEstimation< PointT , pcl::Normal> norm_est;
	boost::shared_ptr< pcl::search::KdTree<PointT> >   tree (new pcl::search::KdTree<PointT> ());
	norm_est.setSearchMethod (tree);

	if(NumberOfNeighborsUseToEstimateNormal<0)
	{
		norm_est.setRadiusSearch(RadiusSearch);
	}
	if(RadiusSearch<0)
	{
		norm_est.setKSearch (NumberOfNeighborsUseToEstimateNormal);
	}
	norm_est.setInputCloud (cloud_in);
	norm_est.compute ( *normals);
	cout<<"estimating normals done ..." <<endl;

}

void NormalEstimation_Test(const char ** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile(argv[1], *cloud_in);

	pcl::PointCloud<pcl::Normal>::Ptr normals(new  pcl::PointCloud<pcl::Normal>);
	int NumberOfNeighborsUseToEstimateNormal=10;

/*
you can only use one of these two:
	NumberOfNeighborsUseToEstimateNormal
	RadiusSearch
one of them should  always has negative value
*/
	float RadiusSearch=-1.0;
	NormalEstimat(cloud_in,NumberOfNeighborsUseToEstimateNormal, RadiusSearch,normals);
	//	visualize normals
	VisualizeNormals < pcl::PointXYZRGB >(cloud_in,normals);

}

template <typename PointT>
void NormalEstimationUsingIntegralImages(boost::shared_ptr< pcl::PointCloud<PointT> >  cloud_in,
		float MaxDepthChangeFactor, float NormalSmoothingSize, pcl::PointCloud<pcl::Normal>::Ptr & normals)
{

//	http://www.pointclouds.org/documentation/tutorials/normal_estimation_using_integral_images.php
    // estimate normals


    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
    // instead of ne.AVERAGE_3D_GRADIENT these could be used as well:
    //    enum NormalEstimationMethod
    //    {
    //      COVARIANCE_MATRIX,
    //      AVERAGE_3D_GRADIENT,
    //      AVERAGE_DEPTH_CHANGE,
    //      SIMPLE_3D_GRADIENT
    //    };
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor( MaxDepthChangeFactor);
    ne.setNormalSmoothingSize(NormalSmoothingSize);
    ne.setInputCloud(cloud_in);
    ne.compute(*normals);

}

void NormalEstimationUsingIntegralImages_Test(const char **argv)
{
	//example of pcd file table_scene_mug_stereo_textured.pcd
	float MaxDepthChangeFactor= 0.02f;
	float NormalSmoothingSize=10.0f;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(argv[1], *cloud_ptr);
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	NormalEstimationUsingIntegralImages<pcl::PointXYZ>(cloud_ptr, MaxDepthChangeFactor, NormalSmoothingSize,normals);
    // visualize normals
    VisualizeNormals <pcl::PointXYZ>(cloud_ptr,normals);
}

template< typename PointT>
void PFH(boost::shared_ptr< pcl::PointCloud<PointT> >   Cloud,
		pcl::PointCloud<pcl::Normal>::Ptr Cloud_Normals	,
		double FeaturesRadius,
		pcl::PointCloud<pcl::PFHSignature125>::Ptr& PFHSignature,
		boost::shared_ptr< pcl::PointCloud<PointT> >  Keypoints)
{
/*
	for each point p in cloud P
	  1. get the nearest neighbors of p (those who euclidean  distance is less than given Radius)
	  2. for each pair of neighbors, compute the three angular values
	  3. bin all the results in an output histogram
*/
//	units are meters:
//	const double NORMALS_RADIUS = 0.03;
//	const double FEATURES_RADIUS = 0.05;
	pcl::PFHEstimation< PointT,pcl::Normal,pcl::PFHSignature125> PFH_Estimator;
	boost::shared_ptr<pcl::search::KdTree< PointT> > search_method_ptr(new pcl::search::KdTree<PointT>());
	PFH_Estimator.setSearchMethod( search_method_ptr);
//	This is optional, if this is not set, it will only use the data in the input cloud to estimate the features.
//	This is useful when you only need to compute the features for a downsampled cloud.
	PFH_Estimator.setSearchSurface (Cloud);
	PFH_Estimator.setRadiusSearch(FeaturesRadius);
	PFH_Estimator.setInputNormals(Cloud_Normals);
	PFH_Estimator.setInputCloud(Keypoints);
	PFH_Estimator.compute(*PFHSignature);
	cout<< "point feature histogram done ... "<<endl;

}

template <typename PointT>
void PFH_Test(const char ** argv)
{
	int NumberOfNeighborsUseToEstimateNormal=-1;
	float RadiusSearch=0.03f;
	// load point cloud
	boost::shared_ptr< pcl::PointCloud<PointT> > cloud (new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile <PointT> (argv[1], *cloud);
	cout<<"loading done..." <<endl;
	cout<<"size of point cloud is:" <<endl;
	cout<<cloud->points.size () <<endl;


	// estimate normals
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	NormalEstimat<PointT>( cloud, NumberOfNeighborsUseToEstimateNormal,RadiusSearch, normals);


	double FeaturesRadius=0.05f;
	boost::shared_ptr<pcl::PointCloud<pcl::PFHSignature125> >  PFHSignature(new pcl::PointCloud<pcl::PFHSignature125>());
	VisualizeNormals<PointT>(cloud,normals);

//	boost::shared_ptr< pcl::PointCloud<PointT> > Keypoints (new pcl::PointCloud<PointT>);
	PFH<PointT>(cloud,normals,  FeaturesRadius,PFHSignature,cloud);

//	PFHSignature->points.size () should have the same size as the input cloud->points.size ()*
	cout<<"PFHSignature->points.size()" <<endl;
	std::cout<< PFHSignature->points.size() <<std::endl;


	// Display and retrieve the shape context descriptor vector for the 0th point.
	pcl::PFHSignature125 descriptor = PFHSignature->points[0];
	cout<<"shape context descriptor vector for the 0th point, 125 number is being printed:" <<endl;
	std::cout << descriptor << std::endl;

	pcl::visualization::PCLHistogramVisualizer pclHistogramVisualizer;
	int hsize=200;
	const std::string &id="cloud";
	int win_width=640;
	int win_height=200;
	pclHistogramVisualizer.addFeatureHistogram<pcl::PFHSignature125>(*PFHSignature,hsize,"cloud",win_width,win_height);
	pclHistogramVisualizer.spin();


	//alternatively
	for(int i=0;i<125;i++)
	{
		std::cout<<PFHSignature->points[0].histogram[i]  <<std::endl;
	}


	return;
}

template <typename PointT>
void VisualizeNormals_Test(const char ** argv)
{
	int NumberOfNeighborsUseToEstimateNormal=100;
	float RadiusSearch=-1.0f;

	// load point cloud
	boost::shared_ptr< pcl::PointCloud<PointT> > cloud (new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile <PointT> (argv[1], *cloud);

	// estimate normals
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	NormalEstimat<PointT>( cloud, NumberOfNeighborsUseToEstimateNormal,RadiusSearch, normals);
	VisualizeNormals<PointT>(cloud,normals);
	return;
}

template<typename PointT>
void FPFH(boost::shared_ptr< pcl::PointCloud<PointT> >  Cloud, pcl::PointCloud<pcl::Normal>::Ptr Cloud_Normals ,double FeaturesRadius, pcl::PointCloud<pcl::FPFHSignature33>::Ptr& FPFHSignature)
{
/*
//	//units are meters:
//	const double NORMALS_RADIUS = 0.03;
//	const double FEATURES_RADIUS = 0.05;
//
	pcl::search::KdTree< PointT > search_method_ptr(new pcl::search::KdTree<PointT>);
	pcl::FPFHEstimation<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
	fpfh_est.setInputCloud(  Cloud );
	fpfh_est.setInputNormals( Cloud_Normals  );
	fpfh_est.setSearchMethod( search_method_ptr );
	fpfh_est.setRadiusSearch( FeaturesRadius );
	fpfh_est.compute( *FPFHSignature  );

	pcl::FPFHEstimationOMP< PointT, pcl::Normal, pcl::FPFHSignature33 > fpfh_OMP_est;
	fpfh_OMP_est.setInputCloud(  Cloud );
	fpfh_OMP_est.setInputNormals( Cloud_Normals  );
	fpfh_OMP_est.setSearchMethod( search_method_ptr );
	fpfh_OMP_est.setRadiusSearch( FeaturesRadius );
	fpfh_OMP_est.compute( *FPFHSignature  );
*/

//	units are meters:
	pcl::FPFHEstimation< PointT,pcl::Normal,pcl::FPFHSignature33> FPFH_Estimator;
	boost::shared_ptr<pcl::search::KdTree< PointT> > search_method_ptr(new pcl::search::KdTree<PointT>());
	FPFH_Estimator.setSearchMethod( search_method_ptr);
//	This is optional, if this is not set, it will only use the data in the input cloud to estimate the features.
//	This is useful when you only need to compute the features for a downsampled cloud.
	FPFH_Estimator.setRadiusSearch(FeaturesRadius);
	FPFH_Estimator.setInputNormals(Cloud_Normals);
	FPFH_Estimator.setInputCloud(Cloud);
	FPFH_Estimator.compute(*FPFHSignature);
	//cout<<FPFHSignature->size()<<endl;
	//cout<<FPFHSignature->points.at(0)<<endl;
	
	cout<< "fast point feature histogram done ... "<<endl;
    
}

void FPFH_Test(const char ** argv )
{

}

void VFH()
{}

void VFH_Test()
{}

void ConcatenateClouds()
{
	pcl::PointCloud<pcl::PointXYZ> cloud_a, cloud_b, cloud_c;
	pcl::PointCloud<pcl::Normal> n_cloud_b;
	pcl::PointCloud<pcl::PointNormal> p_n_cloud_c;

//	  If we are trying to concatenate points then the code below:
	cloud_c  = cloud_a;
	cloud_c += cloud_b;

//	Otherwise if we are attempting to concatenate fields then the code below:
	pcl::concatenateFields (cloud_a, n_cloud_b, p_n_cloud_c);
}

template <typename PointT>
void VisualizeCorrespondences (const boost::shared_ptr< PointCloud<PointT> > points1,
        const pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints1,
        const boost::shared_ptr< PointCloud<PointT> > points2,
        const pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints2,
        const std::vector<int> &correspondences,
        const std::vector<float> &correspondence_scores)

{
//	 We want to visualize two clouds side-by-side, so do to this, we'll make copies of the clouds and transform them
//	 by shifting one to the left and the other to the right.  Then we'll draw lines between the corresponding points
//
//	 Create some new point clouds to hold our transformed data
	boost::shared_ptr< PointCloud<PointT> > points_left (new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_left (new pcl::PointCloud<pcl::PointWithScale>);
	boost::shared_ptr< PointCloud<PointT> > points_right(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_right (new pcl::PointCloud<pcl::PointWithScale>);

//	 Shift the first clouds' points to the left
//	const Eigen::Vector3f translate (0.0, 0.0, 0.3);
	const Eigen::Vector3f translate (0.4, 0.0, 0.0);
	const Eigen::Quaternionf no_rotation (0, 0, 0, 0);
	pcl::transformPointCloud (*points1, *points_left, -translate, no_rotation);
	pcl::transformPointCloud (*keypoints1, *keypoints_left, -translate, no_rotation);

	// Shift the second clouds' points to the right
	pcl::transformPointCloud (*points2, *points_right, translate, no_rotation);
	pcl::transformPointCloud (*keypoints2, *keypoints_right, translate, no_rotation);

	// Add the clouds to the vizualizer
	pcl::visualization::PCLVisualizer viz;
	viz.addPointCloud (points_left, "points_left");
	viz.addPointCloud (points_right, "points_right");

	// Compute the median correspondence score
	std::vector<float> temp (correspondence_scores);
	std::sort (temp.begin (), temp.end ());
	float median_score = temp[temp.size ()/2];

	// Draw lines between the best corresponding points
	for (size_t i = 0; i < keypoints_left->size (); ++i)
//	for (size_t i = 0; i < correspondences.size (); ++i)

	{
		if (correspondence_scores[i] > median_score)
		{
			continue; // Don't draw weak correspondences
		}

/*		std::cout<<"i is  "<<i <<std::endl;
		std::cout<<"correspondences is  " <<correspondences[i] <<std::endl;
		std::cout<<"correspondence scores is " <<correspondence_scores[i] <<std::endl;
		*/
		// Get the pair of points
		const pcl::PointWithScale & p_left = keypoints_left->points[i];
		const pcl::PointWithScale & p_right = keypoints_right->points[correspondences[i]];

		// Generate a random (bright) color
		double r = (rand() % 100);
		double g = (rand() % 100);
		double b = (rand() % 100);
		double max_channel = std::max (r, std::max (g, b));
		r /= max_channel;
		g /= max_channel;
		b /= max_channel;

		// Generate a unique string for each line
		std::stringstream ss ("line");
		ss << i;

		// Draw the line
		viz.addLine (p_left, p_right, r, g, b, ss.str ());

		viz.addText3D( lexical_cast<std::string>(i),p_left, 0.01,r,g,b);
		viz.addText3D( lexical_cast<std::string>(correspondences[i]),p_right,0.01,r,g,b);
	}

	// Give control over to the visualizer
	viz.spin ();
}

template <typename PointT>
//PointT will be something like  pcl::PFHSignature125
void VisualizeHistogram(boost::shared_ptr< pcl::PointCloud<PointT> >  PFHcloud)
{
	// Create the PCLVisualizer object
	pcl::visualization::PCLHistogramVisualizer PFHVisualizer;
	PFHVisualizer.addFeatureHistogram ( *PFHcloud,  600);
	PFHVisualizer.spin();
}

void VisualizeHistogram_Test(const char** argv)
{
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::io::loadPCDFile(argv[1],*cloud);
//	int NumberOfNeighborsUseToEstimateNormal=200;
//	pcl::PointCloud<pcl::Normal>::Ptr normals(new  pcl::PointCloud<pcl::Normal>);
//	/*
//	you can only use one of these two:
//		NumberOfNeighborsUseToEstimateNormal
//		RadiusSearch
//	one of them should  always has negative value
//	 */
//	float RadiusSearch=-1.0;
//	NormalEstimat (cloud,NumberOfNeighborsUseToEstimateNormal,RadiusSearch,normals);
//
//	pcl::PointCloud<pcl::PFHSignature125>::Ptr PFHcloud(new pcl::PointCloud<pcl::PFHSignature125>);
//	double FeaturesRadius=2.0;
//	PFH(cloud,normals,FeaturesRadius,PFHcloud,cloud);
//	VisualizeHistogram<pcl::PFHSignature125>( PFHcloud);


//	example run: ./sandbox vfh_trainingset/000.580.67/1258250240333_cluster_0_nxyz_vfh.pcd
	pcl::PointCloud<pcl::VFHSignature308 >::Ptr cloud(new pcl::PointCloud<pcl::VFHSignature308>);
	pcl::io::loadPCDFile(argv[1],*cloud);
	VisualizeHistogram<pcl::VFHSignature308 >( cloud);


}

template <typename PointT>
void MedianDistance_Correspondence_Rejector(
		boost::shared_ptr<pcl::PointCloud<PointT> >input_cloud,
		boost::shared_ptr<pcl::PointCloud<PointT> >target_cloud,
		double MedianFactor,
		boost::shared_ptr<pcl::Correspondences> &correspondences,
		boost::shared_ptr<pcl::Correspondences> &correspondences_after_median)
{

	pcl::registration::CorrespondenceRejectorMedianDistance MedianDistanceCorrespondenceRejector;
	//MedianFactor: Points with distance greater than median times factor   will be rejected
	MedianDistanceCorrespondenceRejector.setMedianFactor (MedianFactor);
	MedianDistanceCorrespondenceRejector.setInputCorrespondences (correspondences);
	MedianDistanceCorrespondenceRejector.getCorrespondences ( *correspondences_after_median);
	return;
}

template <typename PointT>
void VisualizeCorrespondences(

		const boost::shared_ptr< PointCloud<PointT> > points1,
		        const pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints1,
		        const boost::shared_ptr< PointCloud<PointT> > points2,
		        const pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints2,
		        boost::shared_ptr<pcl::Correspondences> &correspondences)
{

	//	 We want to visualize two clouds side-by-side, so do to this, we'll make copies of the clouds and transform them
	//	 by shifting one to the left and the other to the right.  Then we'll draw lines between the corresponding points
	//
	//	 Create some new point clouds to hold our transformed data
		boost::shared_ptr< PointCloud<PointT> > points_left (new pcl::PointCloud<PointT>);
		pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_left (new pcl::PointCloud<pcl::PointWithScale>);
		boost::shared_ptr< PointCloud<PointT> > points_right(new pcl::PointCloud<PointT>);
		pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_right (new pcl::PointCloud<pcl::PointWithScale>);

	//	 Shift the first clouds' points to the left
	//	const Eigen::Vector3f translate (0.0, 0.0, 0.3);
		const Eigen::Vector3f translate (0.4, 0.0, 0.0);
		const Eigen::Quaternionf no_rotation (0, 0, 0, 0);
		pcl::transformPointCloud (*points1, *points_left, -translate, no_rotation);
		pcl::transformPointCloud (*keypoints1, *keypoints_left, -translate, no_rotation);

		// Shift the second clouds' points to the right
		pcl::transformPointCloud (*points2, *points_right, translate, no_rotation);
		pcl::transformPointCloud (*keypoints2, *keypoints_right, translate, no_rotation);

		// Add the clouds to the vizualizer
		pcl::visualization::PCLVisualizer viz;
		viz.addPointCloud (points_left, "points_left");
		viz.addPointCloud (points_right, "points_right");

		for(size_t i=0;i<correspondences->size();i++)
		{


			const pcl::PointWithScale & p_left = keypoints_left->points[correspondences->at(i).index_query  ];
			const pcl::PointWithScale & p_right = keypoints_right->points[ correspondences->at(i).index_match  ];

			// Generate a random (bright) color
			double r = (rand() % 100);
			double g = (rand() % 100);
			double b = (rand() % 100);
			double max_channel = std::max (r, std::max (g, b));
			r /= max_channel;
			g /= max_channel;
			b /= max_channel;

			// Generate a unique string for each line
			std::stringstream ss ("line");
			ss << i;

			// Draw the line
			viz.addLine (p_left, p_right, r, g, b, ss.str ());

//			viz.addText3D( lexical_cast<std::string>(i),p_left, 0.01,r,g,b);
//			viz.addText3D( lexical_cast<std::string>(correspondences[i]),p_right,0.01,r,g,b);
		}
		// Give control over to the visualizer
		viz.spin ();
		return;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> Visualizer;
template <typename PointT>
void UpdateVisulalizer_Test(boost::shared_ptr<pcl::PointCloud<PointT> > cloud_ptr)
{
	if(Visualizer==boost::shared_ptr<pcl::visualization::PCLVisualizer>())
	{
		Visualizer=boost::shared_ptr<pcl::visualization::PCLVisualizer>( new pcl::visualization::PCLVisualizer("3D Viewer"));
		Visualizer->setBackgroundColor (0, 0, 0);
		Visualizer->addPointCloud(cloud_ptr ,"cloud_ptr");
	}
	else
	{
		cout<<"here" <<endl;
		Visualizer->removeAllPointClouds();
		Visualizer->addPointCloud(cloud_ptr ,"cloud_ptr");
	}
	Visualizer->spin();
	return;
}

void Temp(const char ** argv)
{
// read and visualize point cloud without down sampeling


//	std::string strLeafSize=argv[2];
//	float LeafSize;
//	LeafSize=lexical_cast<float>(strLeafSize);
	pcl::PointCloud<pcl::PointXYZ>::Ptr down_sampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(argv[1],*cloud_ptr);

//	std::cout<< cloud_ptr->size() <<endl;
//	std::cout<< down_sampled_cloud->size() <<endl;

	UpdateVisulalizer_Test(cloud_ptr);

	string input_str;
	float leaf_size;
	char tekrar='y';
	while (tekrar!='n')
	{
	  getline(cin, input_str);
	  leaf_size=lexical_cast<float>(input_str);
	  cout<<leaf_size<<endl;
	  DownSamplingPointCloudUsingVoxelGridFilter(cloud_ptr,leaf_size,down_sampled_cloud);
	  UpdateVisulalizer_Test(down_sampled_cloud);
	  cout<<"Again? (y/n)";
	  cin >> tekrar;
	  cin.get();
	}

}



template <typename PointT>
void VisualizeCorrespondencesHarris(

		const boost::shared_ptr< PointCloud<PointT> > points1,
		        const pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints1,
		        const boost::shared_ptr< PointCloud<PointT> > points2,
		        const pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints2,
		        boost::shared_ptr<pcl::Correspondences> &correspondences)
{

	//	 We want to visualize two clouds side-by-side, so do to this, we'll make copies of the clouds and transform them
	//	 by shifting one to the left and the other to the right.  Then we'll draw lines between the corresponding points
	//
	//	 Create some new point clouds to hold our transformed data
		boost::shared_ptr< PointCloud<PointT> > points_left (new pcl::PointCloud<PointT>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_left (new pcl::PointCloud<pcl::PointXYZI>);
		boost::shared_ptr< PointCloud<PointT> > points_right(new pcl::PointCloud<PointT>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_right (new pcl::PointCloud<pcl::PointXYZI>);

	//	 Shift the first clouds' points to the left
	//	const Eigen::Vector3f translate (0.0, 0.0, 0.3);
		const Eigen::Vector3f translate (0.4, 0.0, 0.0);
		const Eigen::Quaternionf no_rotation (0, 0, 0, 0);
		pcl::transformPointCloud (*points1, *points_left, -translate, no_rotation);
		pcl::transformPointCloud (*keypoints1, *keypoints_left, -translate, no_rotation);

		// Shift the second clouds' points to the right
		pcl::transformPointCloud (*points2, *points_right, translate, no_rotation);
		pcl::transformPointCloud (*keypoints2, *keypoints_right, translate, no_rotation);

		// Add the clouds to the vizualizer
		pcl::visualization::PCLVisualizer viz;
		viz.addPointCloud (points_left, "points_left");
		viz.addPointCloud (points_right, "points_right");

		for(size_t i=0;i<correspondences->size();i++)
		{


			const pcl::PointXYZI & p_left = keypoints_left->points[correspondences->at(i).index_query  ];
			const pcl::PointXYZI & p_right = keypoints_right->points[ correspondences->at(i).index_match  ];

			// Generate a random (bright) color
			double r = (rand() % 100);
			double g = (rand() % 100);
			double b = (rand() % 100);
			double max_channel = std::max (r, std::max (g, b));
			r /= max_channel;
			g /= max_channel;
			b /= max_channel;

			// Generate a unique string for each line
			std::stringstream ss ("line");
			ss << i;

			// Draw the line
			viz.addLine (p_left, p_right, r, g, b, ss.str ());

//			viz.addText3D( lexical_cast<std::string>(i),p_left, 0.01,r,g,b);
//			viz.addText3D( lexical_cast<std::string>(correspondences[i]),p_right,0.01,r,g,b);
		}
		// Give control over to the visualizer
		viz.spin ();
		return;
}

void RemoveNaNFromPFHHistogram(boost::shared_ptr<pcl::PointCloud<pcl::PFHSignature125>  > & cloud_histogram)
{
	pcl::PFHSignature125 descriptor;
	std::vector<int> IndexOfPointsThatContainNan;

	for(size_t i=0;i<cloud_histogram->points.size();i++)
	{
		descriptor = cloud_histogram->points[i];
		for(int j=0;j<125;j++)
		{
			if ( isnan(descriptor.histogram[j] ))
			{
				IndexOfPointsThatContainNan.push_back(i);
				break;
			}
		}
	}

	for(int i=IndexOfPointsThatContainNan.size()-1;i>-1;i--)
	{
		cloud_histogram->points.erase(cloud_histogram->points.begin()+ IndexOfPointsThatContainNan.at(i));
		std::cout<< "Point at index " <<IndexOfPointsThatContainNan.at(i) <<" Has been removed."  <<std::endl;
	}

	return;
}

void RemoveNaNFromFPFHHistogram(boost::shared_ptr<pcl::PointCloud<pcl::FPFHSignature33>  > & cloud_histogram)
{
	pcl::FPFHSignature33 descriptor;
	std::vector<int> IndexOfPointsThatContainNan;

	for(size_t i=0;i<cloud_histogram->points.size();i++)
	{
		descriptor = cloud_histogram->points[i];
		for(int j=0;j<33;j++)
		{
			if ( isnan(descriptor.histogram[j] ))
			{
				IndexOfPointsThatContainNan.push_back(i);
				break;
			}
		}
	}

	for(int i=IndexOfPointsThatContainNan.size()-1;i>-1;i--)
	{
		cloud_histogram->points.erase(cloud_histogram->points.begin()+ IndexOfPointsThatContainNan.at(i));
		std::cout<< "Point at index " <<IndexOfPointsThatContainNan.at(i) <<" Has been removed."  <<std::endl;
	}

	return;
}

void HarrisCorrespondencesDemo(const char **argv)
{
	//	Create some new point clouds to hold our data
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points1 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled1 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr normals1 (new pcl::PointCloud<pcl::Normal>);
//	pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints1 (new pcl::PointCloud<pcl::PointWithScale>);
	pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors1 (new pcl::PointCloud<pcl::PFHSignature125>);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr FPFHdescriptors1 (new pcl::PointCloud<pcl::FPFHSignature33>);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr normals2 (new pcl::PointCloud<pcl::Normal>);
//	pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints2 (new pcl::PointCloud<pcl::PointWithScale>);
	pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors2 (new pcl::PointCloud<pcl::PFHSignature125>);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr FPFHdescriptors2 (new pcl::PointCloud<pcl::FPFHSignature33>);


	// Load the pair of point clouds
	std::stringstream ss1, ss2;
	ss1 << argv[1] << "1.pcd";
	pcl::io::loadPCDFile (ss1.str (), *points1);
	ss2 << argv[1] << "2.pcd";
	pcl::io::loadPCDFile (ss2.str (), *points2);


	std::cout<<"Size of first cloud: "<< points1->size() <<std::endl;
	std::cout<<"Size of second cloud: "<< points2->size() <<std::endl;

//	Downsample the cloud
	const float voxel_grid_leaf_size = 0.01;


	DownSamplingPointCloudUsingVoxelGridFilter<pcl::PointXYZRGB>(points1, voxel_grid_leaf_size, downsampled1);
	DownSamplingPointCloudUsingVoxelGridFilter<pcl::PointXYZRGB>(points2, voxel_grid_leaf_size, downsampled2);


	std::cout<<"Size of first cloud after downsampling: "<< downsampled1->size() <<std::endl;
	std::cout<<"Size of second cloud after downsampling: "<< downsampled2->size() <<std::endl;

//	Compute surface normals
	const float normal_radius = 0.03;
	int NumberOfNeighborsUseToEstimateNormal=-1;
	NormalEstimat<pcl::PointXYZRGB>(downsampled1,NumberOfNeighborsUseToEstimateNormal,normal_radius,normals1);
	NormalEstimat<pcl::PointXYZRGB>(downsampled2,NumberOfNeighborsUseToEstimateNormal,normal_radius,normals2);

//	Compute keypoints
	float radius_for_normal_estimation=0.01;
	float radius_for_determining_nearest_neighbors_used_for_key_point_detection=0.01;
	boost::shared_ptr< pcl::PointCloud<pcl::PointXYZI> > keypoints1(new  pcl::PointCloud<pcl::PointXYZI>);
	boost::shared_ptr< pcl::PointCloud<pcl::PointXYZI> > keypoints2(new  pcl::PointCloud<pcl::PointXYZI>);
//	ExtractHarrisKeypoint(points1,radius_for_normal_estimation,radius_for_determining_nearest_neighbors_used_for_key_point_detection,keypoints1);
//	ExtractHarrisKeypoint(points2,radius_for_normal_estimation,radius_for_determining_nearest_neighbors_used_for_key_point_detection,keypoints2);


//	Compute PFH features
	const float feature_radius = 0.08;
	/* This is a little bit messy: since our keypoint detection returns PointWithScale points, but we want to
	* use them as an input to our PFH estimation, which expects clouds of PointXYZRGB points.  To get around this,
	* we'll use copyPointCloud to convert "keypoints" (a cloud of type PointCloud<PointWithScale>) to
	* "keypoints_xyzrgb" (a cloud of type PointCloud<PointXYZRGB>).  Note that the original cloud doesn't have any RGB
	* values, so when we copy from PointWithScale to PointXYZRGB, the new r,g,b fields will all be zero.
	*/
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints1_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud (*keypoints1, *keypoints1_xyzrgb);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints2_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud (*keypoints2, *keypoints2_xyzrgb);

/*
	PFH<pcl::PointXYZRGB>(downsampled1, normals1,  feature_radius, descriptors1,keypoints1_xyzrgb);
	PFH<pcl::PointXYZRGB>(downsampled2, normals2,  feature_radius, descriptors2,keypoints2_xyzrgb);

*/
	FPFH<pcl::PointXYZRGB> (keypoints1_xyzrgb,normals1,feature_radius,FPFHdescriptors1);
	FPFH<pcl::PointXYZRGB> (keypoints2_xyzrgb,normals2,feature_radius,FPFHdescriptors2);


/*
	RemoveNaNFromPFHHistogram(descriptors1);
	RemoveNaNFromPFHHistogram(descriptors2);
*/
	
	RemoveNaNFromFPFHHistogram(FPFHdescriptors1);
	RemoveNaNFromFPFHHistogram(FPFHdescriptors2);


//	 Find feature correspondences
	std::vector<int> correspondences_descriptors1_to_descriptors2;
	std::vector<float> correspondences_descriptors1_to_descriptors2_scores;

	FindFeatureCorrespondences<pcl::FPFHSignature33 > (FPFHdescriptors1,FPFHdescriptors2, correspondences_descriptors1_to_descriptors2, correspondences_descriptors1_to_descriptors2_scores);
	boost::shared_ptr<pcl::Correspondences > all_correspondences(new pcl::Correspondences );
	CreateCorrespondences(all_correspondences,correspondences_descriptors1_to_descriptors2,correspondences_descriptors1_to_descriptors2_scores);
	cout<<"all_correspondences->size(): " << all_correspondences->size()  <<endl;
	double MedianFactor=0.6f;
	boost::shared_ptr<pcl::Correspondences> correspondences_after_median(new pcl::Correspondences);
	MedianDistance_Correspondence_Rejector(keypoints1_xyzrgb, keypoints2_xyzrgb,MedianFactor,all_correspondences, correspondences_after_median );
	cout<<"correspondences_after_median->size(): " << correspondences_after_median->size() <<endl;
	double RANSAC_threshold=0.03;
	int RANSAC_MaxIterations=5000;
	boost::shared_ptr<pcl::Correspondences> correspondences_after_ransac(new pcl::Correspondences);
	Eigen::Matrix4f  transformation;
	RANSAC_Correspondence_Rejector<pcl::PointXYZRGB>(keypoints1_xyzrgb, keypoints2_xyzrgb,correspondences_after_median,RANSAC_threshold, RANSAC_MaxIterations,  correspondences_after_ransac,transformation );
	cout<< "correspondences_after_ransac->size() "<< correspondences_after_ransac->size()<<endl;
	VisualizeCorrespondencesHarris <pcl::PointXYZRGB>(points1, keypoints1, points2, keypoints2,correspondences_after_ransac );






/*
//	 Find feature correspondences
	std::vector<int> correspondences_descriptors1_to_descriptors2;
	std::vector<float> correspondences_descriptors1_to_descriptors2_scores;

	FindFeatureCorrespondences<pcl::PFHSignature125> (descriptors1, descriptors2, correspondences_descriptors1_to_descriptors2, correspondences_descriptors1_to_descriptors2_scores);
	boost::shared_ptr<pcl::Correspondences > all_correspondences(new pcl::Correspondences );
	CreateCorrespondences(all_correspondences,correspondences_descriptors1_to_descriptors2,correspondences_descriptors1_to_descriptors2_scores);
	cout<<"all_correspondences->size(): " << all_correspondences->size()  <<endl;
	double MedianFactor=0.6f;
	boost::shared_ptr<pcl::Correspondences> correspondences_after_median(new pcl::Correspondences);
	MedianDistance_Correspondence_Rejector(keypoints1_xyzrgb, keypoints2_xyzrgb,MedianFactor,all_correspondences, correspondences_after_median );
	cout<<"correspondences_after_median->size(): " << correspondences_after_median->size() <<endl;
	double RANSAC_threshold=0.03;
	int RANSAC_MaxIterations=5000;
	boost::shared_ptr<pcl::Correspondences> correspondences_after_ransac(new pcl::Correspondences);
	Eigen::Matrix4f  transformation;
	RANSAC_Correspondence_Rejector<pcl::PointXYZRGB>(keypoints1_xyzrgb, keypoints2_xyzrgb,correspondences_after_median,RANSAC_threshold, RANSAC_MaxIterations,  correspondences_after_ransac,transformation );
	cout<< "correspondences_after_ransac->size() "<< correspondences_after_ransac->size()<<endl;
	VisualizeCorrespondencesHarris <pcl::PointXYZRGB>(points1, keypoints1, points2, keypoints2,correspondences_after_ransac );
    */
	return;

}

void CorrespondencesDemo (const char ** argv)
{
//	Create some new point clouds to hold our data
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points1 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled1 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr normals1 (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints1 (new pcl::PointCloud<pcl::PointWithScale>);
	pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors1 (new pcl::PointCloud<pcl::PFHSignature125>);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr normals2 (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints2 (new pcl::PointCloud<pcl::PointWithScale>);
	pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors2 (new pcl::PointCloud<pcl::PFHSignature125>);

	// Load the pair of point clouds
	std::stringstream ss1, ss2;
	ss1 << argv[1] << "1.pcd";
	pcl::io::loadPCDFile (ss1.str (), *points1);
	ss2 << argv[1] << "2.pcd";
	pcl::io::loadPCDFile (ss2.str (), *points2);


	std::cout<<"Size of first cloud: "<< points1->size() <<std::endl;
	std::cout<<"Size of second cloud: "<< points2->size() <<std::endl;

//	Downsample the cloud
	const float voxel_grid_leaf_size = 0.01;


	DownSamplingPointCloudUsingVoxelGridFilter<pcl::PointXYZRGB>(points1, voxel_grid_leaf_size, downsampled1);
	DownSamplingPointCloudUsingVoxelGridFilter<pcl::PointXYZRGB>(points2, voxel_grid_leaf_size, downsampled2);


	std::cout<<"Size of first cloud after downsampling: "<< downsampled1->size() <<std::endl;
	std::cout<<"Size of second cloud after downsampling: "<< downsampled2->size() <<std::endl;

//	Compute surface normals
	const float normal_radius = 0.03;
	int NumberOfNeighborsUseToEstimateNormal=-1;
	NormalEstimat<pcl::PointXYZRGB>(downsampled1,NumberOfNeighborsUseToEstimateNormal,normal_radius,normals1);
	NormalEstimat<pcl::PointXYZRGB>(downsampled2,NumberOfNeighborsUseToEstimateNormal,normal_radius,normals2);

//	Compute keypoints
	const float min_scale = 0.001;
	const int nr_octaves = 5;
	const int nr_octaves_per_scale = 3;
	const float min_contrast = 10.0;
	ExtractSIFTkeypoints<pcl::PointXYZRGB>(points1,min_scale, nr_octaves, nr_octaves_per_scale, min_contrast,keypoints1);
	ExtractSIFTkeypoints<pcl::PointXYZRGB>(points2,min_scale, nr_octaves, nr_octaves_per_scale, min_contrast,keypoints2);

//	Compute PFH features
	const float feature_radius = 0.08;
	/* This is a little bit messy: since our keypoint detection returns PointWithScale points, but we want to
	* use them as an input to our PFH estimation, which expects clouds of PointXYZRGB points.  To get around this,
	* we'll use copyPointCloud to convert "keypoints" (a cloud of type PointCloud<PointWithScale>) to
	* "keypoints_xyzrgb" (a cloud of type PointCloud<PointXYZRGB>).  Note that the original cloud doesn't have any RGB
	* values, so when we copy from PointWithScale to PointXYZRGB, the new r,g,b fields will all be zero.
	*/
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints1_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud (*keypoints1, *keypoints1_xyzrgb);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints2_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud (*keypoints2, *keypoints2_xyzrgb);


	PFH<pcl::PointXYZRGB>(downsampled1, normals1,  feature_radius, descriptors1,keypoints1_xyzrgb);
	PFH<pcl::PointXYZRGB>(downsampled2, normals2,  feature_radius, descriptors2,keypoints2_xyzrgb);


//	 Find feature correspondences
	std::vector<int> correspondences_descriptors1_to_descriptors2;
	std::vector<float> correspondences_descriptors1_to_descriptors2_scores;

	FindFeatureCorrespondences<pcl::PFHSignature125> (descriptors1, descriptors2, correspondences_descriptors1_to_descriptors2, correspondences_descriptors1_to_descriptors2_scores);
	boost::shared_ptr<pcl::Correspondences > all_correspondences(new pcl::Correspondences );
	CreateCorrespondences(all_correspondences,correspondences_descriptors1_to_descriptors2,correspondences_descriptors1_to_descriptors2_scores);
	cout<<"all_correspondences->size(): " << all_correspondences->size()  <<endl;
	double MedianFactor=1.0;
	boost::shared_ptr<pcl::Correspondences> correspondences_after_median(new pcl::Correspondences);
	MedianDistance_Correspondence_Rejector(keypoints1_xyzrgb, keypoints2_xyzrgb,MedianFactor,all_correspondences, correspondences_after_median );
	cout<<"correspondences_after_median->size(): " << correspondences_after_median->size() <<endl;
	double RANSAC_threshold=0.03;
	int RANSAC_MaxIterations=5000;
	boost::shared_ptr<pcl::Correspondences> correspondences_after_ransac(new pcl::Correspondences);
	Eigen::Matrix4f  transformation;
	RANSAC_Correspondence_Rejector<pcl::PointXYZRGB>(keypoints1_xyzrgb, keypoints2_xyzrgb,correspondences_after_median,RANSAC_threshold, RANSAC_MaxIterations,  correspondences_after_ransac,transformation );
	cout<< "correspondences_after_ransac->size() "<< correspondences_after_ransac->size()<<endl;
	VisualizeCorrespondences <pcl::PointXYZRGB>(points1, keypoints1, points2, keypoints2,correspondences_after_ransac );
	return;
}

//void ReadingPLYPointCloud(std::string PLYfile_name, pcl::PointCloud< pcl::PointXYZRGB > &cloud)
void ReadingPLYPointCloud(std::string PLYfile_name, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud)

{
	pcl::PLYReader PLYFileReader;
	const int offset=0;
	PLYFileReader.read< pcl::PointXYZRGB >(PLYfile_name,*cloud,offset);
	return;
}

void ReadingPLYPointCloud_Tes(const char ** argv)
{
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud_ptr(new PointCloud<pcl::PointXYZRGB>);
	ReadingPLYPointCloud(argv[1],  cloud_ptr);
	cout<<cloud_ptr->size()  <<endl;
	
	pcl::io::savePCDFileASCII( argv[2],  *cloud_ptr);
	
	
// 	pcl::RangeImage range_image;
// 	Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
// 	CreateRangeImage<pcl::PointXYZRGB>(cloud_ptr,range_image,scene_sensor_pose);
// 	VisualizeRangeImage<pcl::PointXYZRGB>(range_image,cloud_ptr);

	
// 	pcl::visualization::CloudViewer myviewer("title:");
// 	myviewer.showCloud( cloud_ptr);
// 	while(!myviewer.wasStopped() )
// 	{
// 	}
	
	
}

int main (int argc, const char ** argv)
{
	SimpleCloudViewer_Test(argv);

//	SimpleVisualizer_Test(argv);

//	AddNormalsToPointCloudTest(argv);

//	VisualizeRangeImage_Test(argv);

//	ExtractNARFkeypoints_Test(argv);

//	ExtractNARFFeatures_Test(argv);

//	ExtractSIFTkeypoints_Test(argv);

//	NormalEstimation_Test(argv);

//	ExtractSIFTkeypointsTest(argv);

//	VisualizeHistogram_Test(argv);

//	DownSamplingPointCloudUsingVoxelGridFilter_Test(argv);

//	FindTransformationMatrix_Test(argv);

//	NormalEstimationUsingIntegralImages_Test(argv);

//	ExtractSIFTkeypoints_Test(argv);

//	VisualizeHistogram_Test(argv);

//	UpdateVisulalizer_Test(argv);

//	Temp(argv);

//	VisualizeRangeImage_Test(argv);

//	NARFRegistration_Test(argv);

//	ExtractHarrisKeypoint_Test(argv);

//	FindTransformationMatrix_Test(argv);

//	VisualizeNormals_Test<pcl::PointXYZRGB >(argv);

//	PFH_Test<pcl::PointXYZRGB >(argv);

//	CorrespondencesDemo(argv);

//	HarrisCorrespondencesDemo(argv);

//	ReadingPLYPointCloud_Tes( argv);

    SimpleCloudViewer_Test(argv);

	return 0;
}
/*

matrix from tutorial:
1 -1.95289e-08  3.01015e-08 -5.96046e-08
1.93124e-08            1 -7.27084e-10 -1.49012e-08
-4.02429e-08 -7.27084e-10            1  1.19209e-07
0            0            0            1
*/

