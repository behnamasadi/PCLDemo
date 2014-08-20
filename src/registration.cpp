#include "registration.h"

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
#include "pcl/features/fpfh.h"

#include <pcl/visualization/histogram_visualizer.h>


#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_estimation.h>

#include <pcl/visualization/pcl_visualizer.h>


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/*
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
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>

#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>

#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/impl/sift_keypoint.hpp>

#include <pcl/features/integral_image_normal.h>


#include <pcl/features/pfh.h>
#include "pcl/features/fpfh.h"

#include <pcl/visualization/histogram_visualizer.h>


#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_estimation.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>

#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>


#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>

#include <pcl/features/range_image_border_extractor.h>
*/

using namespace Eigen;
using namespace std;
// dont use using namespace pcl!
//using namespace pcl;



Registration::Registration(QWidget *parent) : QWidget(parent)
{
	ui.setupUi(this);
	InitializingVariables();
	SetingUpSignals();
	UpdateDisplayImageWindow();

}

Registration::~Registration()
{

}

template<typename to, typename from>
to lexical_cast(from const &x)
{
	std::stringstream os;
	to ret;
	os << x;
	os >> ret;
	return ret;
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

void SimpleVisualizer(std::vector < boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB > >  > PointCloudPtrVector)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	for(size_t i=0;i<PointCloudPtrVector.size();i++)
	{
		viewer->addPointCloud< pcl::PointXYZRGB >(PointCloudPtrVector.at(i) ,lexical_cast<std::string>(i));
	}
	viewer->spin();
}

template<typename PointT>
void DownSamplingPointCloudUsingVoxelGridFilter(boost::shared_ptr<pcl::PointCloud<PointT> > cloud_in,float LeafSize,boost::shared_ptr<pcl::PointCloud<PointT> >& cloud_out)
{
	pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud (cloud_in);
	sor.setLeafSize (LeafSize, LeafSize, LeafSize);
	sor.filter (*cloud_out);
}

void Registration::SetingUpSignals()
{
	connect(ui.SelectFolderContainingPCDFilespushButton ,SIGNAL( clicked()),this,SLOT( SelectFolderContainingPCDFilespushButtonClicked()));
	connect (ui.VoxelGridSize , SIGNAL(valueChanged(int)) ,this,SLOT(VoxelGridSizeSlidervalueChanged() ) );
	connect (ui.VoxelGridSize , SIGNAL( sliderReleased()) ,this,SLOT(VoxelGridSizeSlidersliderReleased() ) );
	connect(ui.DownSamplepushButton ,SIGNAL( clicked()),this,SLOT( DownSamplepushButtonclicked()));
	connect(ui.CalculateCorrespondencespushButton ,SIGNAL( clicked()),this,SLOT( CalculateCorrespondencespushButtonclicked()));
}

void Registration::SelectFolderContainingPCDFilespushButtonClicked()
{

	QString SelectedDirectoryName =QFileDialog::getExistingDirectory(this,QString("Please select the directory containing images:"),QString("."));
	this->FolderContainingPCDFilesFullPath=SelectedDirectoryName.toStdString()+"/";
	FileExtension=this->FileExtension;
	QDir dir(QString(FolderContainingPCDFilesFullPath.c_str() ) );
	dir.setFilter(QDir::Files | QDir::Hidden | QDir::NoSymLinks);
	dir.setSorting(QDir::Name  | QDir::Reversed);

	Qt::DateFormat format = Qt::TextDate;
	QFileInfoList list = dir.entryInfoList();
	QString qStr = QString::number(list.size() );
	for (size_t i = 0; i < list.size(); ++i)
	{
		QFileInfo fileInfo = list.at(i);
		if( FileExtension.compare(fileInfo.suffix().toLower().toStdString())==0 )
		{
			this->ListofFiles.push_back(fileInfo.fileName().toStdString()  );
		}
	}
	return;
}

void Registration::VoxelGridSizeSlidervalueChanged()
{
	float voxel_grid_size= (ui.VoxelGridSize->value()*1.0)/(ui.VoxelGridSize->maximum()*10.0);
//	QMessageBox::information(this,QString("voxel grid size is: "),QString::number( voxel_grid_size  )   );
	ui.VoxelGridSizelabel->setText(QString::number( voxel_grid_size  ) );
}

void Registration::InitializingVariables()
{
	ui.VoxelGridSizelabel->setText(QString("") );

	this->FileExtension="pcd";
	ui.VoxelGridSize->setMaximum(100);
	ui.VoxelGridSize->setMinimum(0);
	ui.VoxelGridSize->setValue(20);
}

void Registration::DownSamplepushButtonclicked()
{
		float voxel_grid_size= (ui.VoxelGridSize->value()*1.0)/(ui.VoxelGridSize->maximum()*10.0);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr down_sampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
		std::string FullPathToPCDFile= this->FolderContainingPCDFilesFullPath+this->ListofFiles.at(0);
		pcl::io::loadPCDFile<pcl::PointXYZRGB>(FullPathToPCDFile.c_str(),*cloud_ptr);
		DownSamplingPointCloudUsingVoxelGridFilter<pcl::PointXYZRGB>(cloud_ptr,voxel_grid_size,down_sampled_cloud);
//		std::vector< boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB > >  > PointCloudPtrVector;
//		PointCloudPtrVector.push_back(cloud_ptr);
		PointCloudVisualizer<pcl::PointXYZRGB> (down_sampled_cloud);
}

void Registration::VoxelGridSizeSlidersliderReleased()
{
	float voxel_grid_size= (ui.VoxelGridSize->value()*1.0)/(ui.VoxelGridSize->maximum()*1.0);
//	QMessageBox::information(this,QString("voxel grid size is: "),QString::number( voxel_grid_size  )   );

}

void Registration::UpdateDisplayImageWindow()
{
	return;
}

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


}

template <typename PointT>
void VisualizeCorrespondences(

		const boost::shared_ptr< pcl::PointCloud<PointT> > points1,
		        const pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints1,
		        const boost::shared_ptr< pcl::PointCloud<PointT> > points2,
		        const pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints2,
		        boost::shared_ptr<pcl::Correspondences> &correspondences)
{

	//	 We want to visualize two clouds side-by-side, so do to this, we'll make copies of the clouds and transform them
	//	 by shifting one to the left and the other to the right.  Then we'll draw lines between the corresponding points
	//
	//	 Create some new point clouds to hold our transformed data
		boost::shared_ptr< pcl::PointCloud<PointT> > points_left (new pcl::PointCloud<PointT>);
		pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_left (new pcl::PointCloud<pcl::PointWithScale>);
		boost::shared_ptr< pcl::PointCloud<PointT> > points_right(new pcl::PointCloud<PointT>);
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
	sift.compute( *result );
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
	const double NORMALS_RADIUS = 0.03;
	const double FEATURES_RADIUS = 0.05;
	pcl::PFHEstimation< PointT,pcl::Normal,pcl::PFHSignature125> PFH_Estimator;
	boost::shared_ptr<pcl::search::KdTree< PointT> > search_method_ptr(new pcl::search::KdTree<PointT>());
	PFH_Estimator.setSearchMethod( search_method_ptr);
	PFH_Estimator.setSearchSurface (Cloud);
	PFH_Estimator.setRadiusSearch(FeaturesRadius);
	PFH_Estimator.setInputNormals(Cloud_Normals);
	PFH_Estimator.setInputCloud(Keypoints);
	PFH_Estimator.compute(*PFHSignature);
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

void CorrespondencesDemo (std::string FullPathToFirstPCDFile, std::string FullPathToSecondPCDFile)
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



	pcl::io::loadPCDFile (FullPathToFirstPCDFile.c_str(), *points1);
	pcl::io::loadPCDFile (FullPathToSecondPCDFile.c_str(), *points2);


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

//	Compute sift keypoints
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
/*
	double RANSAC_threshold=0.03;
	int RANSAC_MaxIterations=5000;
	boost::shared_ptr<pcl::Correspondences> correspondences_after_ransac(new pcl::Correspondences);
	Eigen::Matrix4f  transformation;
	RANSAC_Correspondence_Rejector<pcl::PointXYZRGB>(keypoints1_xyzrgb, keypoints2_xyzrgb,correspondences_after_median,RANSAC_threshold, RANSAC_MaxIterations,  correspondences_after_ransac,transformation );
	cout<< "correspondences_after_ransac->size() "<< correspondences_after_ransac->size()<<endl;
    cout<< transformation <<endl;
  */
    VisualizeCorrespondences <pcl::PointXYZRGB>(points1, keypoints1, points2, keypoints2,correspondences_after_median );



/*
	pcl::io::loadPCDFile (FullPathToFirstPCDFile.c_str(), *points1);
	pcl::io::loadPCDFile (FullPathToSecondPCDFile.c_str(), *points2);

	pcl::visualization::PCLVisualizer viz;
	viz.addPointCloud <pcl::PointXYZRGB> (points1, "points1");
//	viz.addPointCloud <pcl::PointXYZRGB>(points2, "points2");

	boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> > points2_transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud (*points2, *points2_transformed, transformation);
//
	viz.addPointCloud <pcl::PointXYZRGB>(points2_transformed, "points2_transformed");
	viz.spin ();

*/


	return;
}

void Registration::CalculateCorrespondencespushButtonclicked()
{
	std::string FullPathToFirstPCDFile= this->FolderContainingPCDFilesFullPath+this->ListofFiles.at(0);
	std::string FullPathToSecondPCDFile= this->FolderContainingPCDFilesFullPath+this->ListofFiles.at(1);
	CorrespondencesDemo (FullPathToFirstPCDFile,FullPathToSecondPCDFile);
}
