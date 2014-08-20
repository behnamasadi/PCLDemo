/*
 * FastPointFeatureHistograms.cpp
 *
 *  Created on: May 12, 2012
 *      Author: behnam
 */
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>

//http://pointclouds.org/documentation/tutorials/fpfh_estimation.php
int main()
{
	//typedef boost::shared_ptr<PointCloud<PointT> > Ptr;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);// cloud is a pointer!
	// the second parameter has been called by reference ,"cloud" is pointer so "*cloud" would be the object
	pcl::io::loadPCDFile<pcl::PointXYZ> ("pcd_files/test_pcd.pcd", *cloud);

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



	// Create the FPFH estimation class, and pass the input dataset+normals to it
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	fpfh.setInputCloud (cloud);
	fpfh.setInputNormals (cloud_normals);



	// Create an empty kdtree representation, and pass it to the FPFH estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_for_fast_point_feature_histogram(new pcl::search::KdTree<pcl::PointXYZ>);

	fpfh.setSearchMethod (tree_for_fast_point_feature_histogram);

	// Output datasets
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

	// Use all neighbors in a sphere of radius 5cm
	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	fpfh.setRadiusSearch (0.05);

	// Compute the features
	fpfh.compute (*fpfhs);

	// fpfhs->points.size () should have the same size as the input cloud->points.size ()*


	pcl::FPFHSignature33 descriptor = fpfhs->points[0];
	std::cout << descriptor << std::endl;




}



