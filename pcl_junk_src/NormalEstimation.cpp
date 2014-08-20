/*
 * NormalEstimation.cpp
 *
 *  Created on: May 9, 2012
 *      Author: behnam
 */

//http://pointclouds.org/documentation/tutorials/normal_estimation.php#normal-estimation
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>

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
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (0.03);

	// Compute the features
	ne.compute (*cloud_normals);

	// cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
	std::cout<<cloud_normals->points.size () <<std::endl;


	return 0;
}



