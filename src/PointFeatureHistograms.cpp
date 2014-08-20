/*
 * PointFeatureHistograms.cpp
 *
 *  Created on: May 12, 2012
 *      Author: behnam
 */
#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <iostream>

using namespace std;

int main(int argc, const char ** argv)
{
	//typedef boost::shared_ptr<PointCloud<PointT> > Ptr;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);// cloud is a pointer!
	// the second parameter has been called by reference ,"cloud" is pointer so "*cloud" would be the object
	pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud);

	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_for_points_normal (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree_for_points_normal);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (0.03);

	// Compute the features
	ne.compute (*cloud_normals);

	///////////////////Till here we just compute the normals, which obtained form the code: NormalEstimation.cpp \\\\\\\\\\\\\


	// Create the PFH estimation class, and pass the input dataset+normals to it
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
	pfh.setInputCloud (cloud);
	pfh.setInputNormals (cloud_normals);
	// alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals (cloud);

	// Create an empty kdtree representation, and pass it to the PFH estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	//pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_for_point_feature_histogram (new pcl::search::KdTree<pcl::PointXYZ> ());
	pfh.setSearchMethod(tree_for_point_feature_histogram);

	// Output datasets
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs (new pcl::PointCloud<pcl::PFHSignature125> ());

	// Use all neighbors in a sphere of radius 5cm
	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	pfh.setRadiusSearch (0.05);

	// Compute the features
	pfh.compute (*pfhs);

	// pfhs->points.size () should have the same size as the input cloud->points.size ()*
	//std::cout<< pfhs->points.size() <<std::endl;

// Display and retrieve the shape context descriptor vector for the 0th point.
	pcl::PFHSignature125 descriptor = pfhs->points[0];
	std::cout << descriptor << std::endl;

	pcl::visualization::PCLHistogramVisualizer pclHistogramVisualizer;
	int hsize=200;
	const std::string &id="cloud";
	int win_width=640;
	int win_height=200;
	//pclHistogramVisualizer.addFeatureHistogram(*cloud,hsize,"cloud",win_width,win_height);


// alternatively
//	for(int i=0;i<125;i++)
//	{
//		std::cout<<pfhs->points[0].histogram[i]  <<std::endl;
//	}

	return 0;
}



