#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>


#include <pcl/sample_consensus/sac_model_sphere.h>

#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>


#include <pcl/filters/crop_box.h>

boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  


  viewer->initCameraParameters ();
  return (viewer);
}

void surface_equation(double &x,double &y, double &z)
{
		double a,b,c;
		a=-0.5;
		b=0;
		c=0;
		y=a*x*x+b*x+c;




}

void create_quadratic_polynomial_curve( pcl::PointCloud<pcl::PointXYZ>::Ptr &quadratic_polynomial_curve_cloud)
{
	double x_min,x_max, y_min,y_max,z_min,z_max;

	double x_step_size,y_step_size,z_step_size;
	double y=0;
	pcl::PointXYZ point;
	x_min=0;
	x_max=2;
	z_min=0;
	z_max=2;
	x_step_size=0.1;
	z_step_size=0.1;
	
	for(double z=z_min;z<z_max;z=z+z_step_size)
	{
		for(double x=x_min;x<x_max;x=x+x_step_size)
		{
			surface_equation( x,y, z);
			point.x=x;
			point.y=y;
			point.z=z;
			quadratic_polynomial_curve_cloud->push_back(point);
		}
	}



}



int main(int argc, char** argv)
{
/**/
	pcl::PointCloud<pcl::PointXYZ>::Ptr quadratic_polynomial_curve_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<int> inliers;


	create_quadratic_polynomial_curve( quadratic_polynomial_curve_cloud);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	
	std::cout<< "quadratic_polynomial_curve_cloud->points.size(): "<<quadratic_polynomial_curve_cloud->points.size()  <<std::endl;




	viewer = simpleVis(quadratic_polynomial_curve_cloud);
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}


	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (quadratic_polynomial_curve_cloud));
	
/////////////////////////////////////// fitting the plane number 1 ////////////////////////////////////

/*
	Eigen::VectorXf plane1_initial_param_model_coefficients;

	//0,1,2,3 -> a,b,c,d
	plane1_initial_param_model_coefficients[0]=0;
	plane1_initial_param_model_coefficients[1]=0;
	plane1_initial_param_model_coefficients[2]=0;
	plane1_initial_param_model_coefficients[3]=0;


	double 	threshold=0.4; //in meter
	std::vector< int > inliers;
	model_p->selectWithinDistance(plane1_initial_param_model_coefficients, threshold, inliers);	

*/


	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
	ransac.setDistanceThreshold (.01);
	ransac.computeModel();
	ransac.getInliers(inliers);

	pcl::copyPointCloud<pcl::PointXYZ>(*quadratic_polynomial_curve_cloud, inliers, *final);

	

	

	viewer = simpleVis(final);

	 std::vector<int> samples;
	Eigen::VectorXf model_coefficients;

	int center_point=inliers.size()/2;

	samples.push_back(inliers.at(center_point));
	samples.push_back(inliers.at(center_point+1));
	samples.push_back(inliers.at(center_point-1));

	if(model_p->computeModelCoefficients(samples,model_coefficients))
	{
		//a, b, c, d (ax+by+cz+d=0)
		float a,b,c,d;
		a=model_coefficients[0];
		b=model_coefficients[1];
		c=model_coefficients[2];
		d=model_coefficients[3];
		std::cout<<"a,b,c,d: "<<a<<" , " <<b<<" , " << c<<" , " <<d <<std::endl;
	}

	

	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}

}
