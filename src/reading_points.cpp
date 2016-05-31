/*
 * ReadingFile.cpp
 *
 *  Created on: Apr 13, 2012
 *      Author: behnam
 */
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>



#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>


#include <pcl/filters/crop_box.h>
#include <pcl/visualization/pcl_visualizer.h> 


#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>


#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/sample_consensus/sac_model_parallel_line.h>

#include <pcl/features/moment_invariants.h>



using namespace std;

unsigned int text_id = 0;
void mouseEventOccurred (const pcl::visualization::MouseEvent &event, void* viewer_void)

{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton && event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
  {
    std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;
    char str[512];

    sprintf (str, "text#%03d", text_id ++);
    viewer->addText ("clicked here", event.getX (), event.getY (), str);
  }
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym () == "r" && event.keyDown ())
  {
    std::cout << "r was pressed => removing all text" << std::endl;

    char str[512];
    for (unsigned int i = 0; i < text_id; ++i)
    {
      sprintf (str, "text#%03d", i);
      viewer->removeShape (str);
    }
    text_id = 0;
  }
}

void PointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* cookie) 
{ 
        int idx = event.getPointIndex (); 
        if (idx == -1) 
                return; 
        // Get the point that was picked
		pcl::PointXYZ picked_point;
        
        event.getPoint ( picked_point.x, picked_point.y, picked_point.z);

		std::cout << "point picked at (" <<  picked_point.x << ", " <<  picked_point.y<< ", "<<  picked_point.z<< ")" << std::endl;
        
        pcl::visualization::PCLVisualizer * v_visualizer = (pcl::visualization::PCLVisualizer*)(cookie); 
        v_visualizer->removeShape("PickedPoint"); 

        v_visualizer->addSphere(picked_point, 10.0, 1.0, 0.0, 1.0, "PickedPoint", 0); 
}

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

struct callback_args{
  // structure used to pass arguments to the callback function
  PointCloudT::Ptr clicked_points_3d;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};


void pp_callback (const pcl::visualization::PointPickingEvent& event, void* args)
{
  std::cout<<"-------------------------------------" <<std::endl;

    struct callback_args* data = (struct callback_args *)args;
  if (event.getPointIndex () == -1)
    return;
  PointT current_point;
  event.getPoint(current_point.x, current_point.y, current_point.z);
  data->clicked_points_3d->points.push_back(current_point);





  // Draw clicked points in red:
  pcl::visualization::PointCloudColorHandlerCustom<PointT> red (data->clicked_points_3d, 255, 0, 0);
  data->viewerPtr->removePointCloud("clicked_points");
  data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
  data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
  std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}

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

template<typename to, typename from>
to lexical_cast(from const &x)
{
    std::stringstream os;
    to ret;
    os << x;
    os >> ret;
    return ret;
}

void populate_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{

}

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

std::vector<std::string> split(const std::string &s, char delim) 
{
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

void reading_file_at_once()
{
    std::string path_to_file("points.txt");
    std::ifstream in(path_to_file.c_str());
    std::stringstream buffer;
    buffer << in.rdbuf();
    std::string contents(buffer.str());
    std::cout<<contents <<std::endl;
}

int reading_line_by_line(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  string line;
  char delim=',';
  pcl::PointXYZ point;
  double x,y,z;
  ifstream myfile ("points.txt");
  double scale=1000;
  if (myfile.is_open())
  {
    while ( getline (myfile,line) )
    {
      //getline (myfile,line);
      //cout <<"..." <<line << "..."<<endl;
		std::vector<std::string> single_line_points= split(line, delim);
//		cout <<"x: " <<single_line_points.at(0)<<endl;
		//cout <<"y: " <<single_line_points.at(1)<<endl;
		//cout <<"z: " <<single_line_points.at(2)<<endl;
		
		x=lexical_cast<double>(single_line_points.at(0));
		y=lexical_cast<double>(single_line_points.at(1));
		z=lexical_cast<double>(single_line_points.at(2));


		point.x=x/scale;
		point.y=y/scale;
		point.z=z/scale;
		cloud->push_back(point);



    }
	std::cout<<"size of cloud " <<cloud->points.size() <<std::endl;
    myfile.close();
  }

  else
  {
	  cout << "Unable to open file...";
  }
  return 0;
}

void cropping_box()
{



}




int reading_pcd_file ()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cout << "    " << cloud->points[i].x
              << " "    << cloud->points[i].y
              << " "    << cloud->points[i].z << std::endl;

  return (0);
}
void truck_visulizer()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);


    reading_line_by_line(cloud);

    std::cout<<"size of cloud " <<cloud->points.size() <<std::endl;


    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.1f, 0.1f, 0.1f);
    sor.filter (*cloud_filtered);



    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;


    //viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
    viewer = simpleVis(cloud_filtered);


    struct callback_args cb_args;
    PointCloudT::Ptr clicked_points_3d (new PointCloudT);
    cb_args.clicked_points_3d = clicked_points_3d;
    cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr( viewer);

//    viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);
    viewer->registerPointPickingCallback (pp_callback, (void*)&cb_args);


//    while (!viewer->wasStopped ())
//    {
//        viewer->spinOnce (100);
//        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//    }



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    boost::shared_ptr<pcl::CropBox< pcl::PointXYZ> > Crop_Box_ConstPtr(new pcl::CropBox< pcl::PointXYZ> );



    //Crop_Box_ConstPtr->setInputCloud() 4x1
    Eigen::Vector4f min_pt,max_pt;

//    min_pt(0,0)=314;
    min_pt(0,0)=308;
    min_pt(1,0)=22;
    min_pt(2,0)=3;


//    max_pt(0,0)=322;
    max_pt(0,0)=325;
    max_pt(1,0)=27;
    max_pt(2,0)=4;
    Crop_Box_ConstPtr->setMin(min_pt);
    Crop_Box_ConstPtr->setMax(max_pt);
    Crop_Box_ConstPtr->setInputCloud(cloud);


    Crop_Box_ConstPtr->filter(*cropped_cloud);

    std::cout<<"size of cropped_cloud " <<cropped_cloud->points.size() <<std::endl;



    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cropped_cloud));
//    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));



    //Eigen::VectorXf plane1_initial_param_model_coefficients;
    Eigen::Vector4f plane1_initial_param_model_coefficients;

    // plane1_initial_param_model_coefficients are ax+by+cz+d=0
    //for the palne1 s and y could be anything and y must be something around -2.48/2 so a=0, c=0 and y=-d/b therefore d=-2.448 and b=2
    plane1_initial_param_model_coefficients(0,0)=0; //a
    plane1_initial_param_model_coefficients(1,0)=0; //b
    plane1_initial_param_model_coefficients(2,0)=1; //c
    plane1_initial_param_model_coefficients(3,0)=-3.5; //d


    double 	distance=0.20; //in meter
    std::vector< int > inliers;
    model_p->selectWithinDistance(plane1_initial_param_model_coefficients, distance, inliers);





    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
    ransac.setDistanceThreshold (0.05);
    ransac.computeModel();
    ransac.getInliers(inliers);

    pcl::copyPointCloud<pcl::PointXYZ>(*cropped_cloud, inliers, *final);
//    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);

    std::cout<<"size of final " <<final->points.size() <<std::endl;



    Eigen::VectorXf computed_plane1_initial_param_model_coefficients;
    ransac.getModelCoefficients(computed_plane1_initial_param_model_coefficients );



    std::cout<<"a: " <<computed_plane1_initial_param_model_coefficients(0,0) <<std::endl;
    std::cout<<"b: " <<computed_plane1_initial_param_model_coefficients(1,0) <<std::endl;
    std::cout<<"c: " <<computed_plane1_initial_param_model_coefficients(2,0) <<std::endl;
    std::cout<<"d: " <<computed_plane1_initial_param_model_coefficients(3,0) <<std::endl;



///////////////////////////////////////////////////////////         /////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    pcl::io::savePCDFile("final_shipping_container.pcd",*final);

    viewer = simpleVis(final);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }



/*
    boost::shared_ptr<pcl::CropBox< pcl::PointXYZ> > Crop_Box_ConstPtr(new pcl::CropBox< pcl::PointXYZ> );

    double width, heigh, lengh;
    //scales are in meter:
    width=4;
    heigh=5;
    lengh=15;

    //Crop_Box_ConstPtr->setInputCloud() 4x1
    Eigen::Vector4f min_pt,max_pt;

    min_pt(0,0)=-25;
    min_pt(1,0)=-2;
    min_pt(2,0)=0;


    max_pt(0,0)=0;
    max_pt(1,0)=2;
    max_pt(2,0)=5;


//    std::cout << "The matrix  is of size " << min_pt.rows() << "x" << min_pt.cols() << std::endl;


    Crop_Box_ConstPtr->setMin(min_pt);
    Crop_Box_ConstPtr->setMax(max_pt);
    Crop_Box_ConstPtr->setInputCloud(cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud (new pcl::PointCloud<pcl::PointXYZ>);


    pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);



    Crop_Box_ConstPtr->filter(*cropped_cloud);

    std::cout<<"size of cropped_cloud " <<cropped_cloud->points.size() <<std::endl;




    viewer = simpleVis(cropped_cloud);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cropped_cloud));


    //Eigen::VectorXf plane1_initial_param_model_coefficients;
    Eigen::Vector4f plane1_initial_param_model_coefficients;

    // plane1_initial_param_model_coefficients are ax+by+cz+d=0
    //for the palne1 s and y could be anything and y must be something around -2.48/2 so a=0, c=0 and y=-d/b therefore d=-2.448 and b=2
    plane1_initial_param_model_coefficients(0,0)=0; //a
    plane1_initial_param_model_coefficients(1,0)=2; //b
    plane1_initial_param_model_coefficients(2,0)=0; //c
    plane1_initial_param_model_coefficients(3,0)=-2.438; //d


    double 	threshold=0.05; //in meter
    std::vector< int > inliers;
    model_p->selectWithinDistance(plane1_initial_param_model_coefficients, threshold, inliers);





    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
    ransac.setDistanceThreshold (0.05);
    ransac.computeModel();
    ransac.getInliers(inliers);

    pcl::copyPointCloud<pcl::PointXYZ>(*cropped_cloud, inliers, *final);

    std::cout<<"size of final " <<final->points.size() <<std::endl;



    Eigen::VectorXf computed_plane1_initial_param_model_coefficients;
    ransac.getModelCoefficients(computed_plane1_initial_param_model_coefficients );



    std::cout<<"a: " <<computed_plane1_initial_param_model_coefficients(0,0) <<std::endl;
    std::cout<<"b: " <<computed_plane1_initial_param_model_coefficients(1,0) <<std::endl;
    std::cout<<"c: " <<computed_plane1_initial_param_model_coefficients(2,0) <<std::endl;
    std::cout<<"d: " <<computed_plane1_initial_param_model_coefficients(3,0) <<std::endl;


    //model_p->optimizeModelCoefficients();
    //model_p->computeModelCoefficients()


    //const std::vector<int> &samples, Eigen::VectorXf &model_coefficients

    //model_p->computeModelCoefficients


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    viewer = simpleVis(final);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    */
}


void shipping_container_cutter()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);


    pcl::io::loadPCDFile("/home/behnam/workspace/PCLDemo/build/shipping_container_0_.pcd",*cloud);
    std::cout<<"size of cloud " <<cloud->points.size() <<std::endl;





    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;


    viewer = simpleVis(cloud);

    struct callback_args cb_args;
    PointCloudT::Ptr clicked_points_3d (new PointCloudT);
    cb_args.clicked_points_3d = clicked_points_3d;
    cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr( viewer);

    viewer->registerPointPickingCallback (pp_callback, (void*)&cb_args);



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    boost::shared_ptr<pcl::CropBox< pcl::PointXYZ> > Crop_Box_ConstPtr(new pcl::CropBox< pcl::PointXYZ> );



    //Crop_Box_ConstPtr->setInputCloud() 4x1
    Eigen::Vector4f min_pt,max_pt;

//    min_pt(0,0)=314;
    min_pt(0,0)=-100;
    min_pt(1,0)=-100;
    min_pt(2,0)=-100;


//    max_pt(0,0)=322;
    max_pt(0,0)=100;
    max_pt(1,0)=100;
    max_pt(2,0)=100;
    Crop_Box_ConstPtr->setMin(min_pt);
    Crop_Box_ConstPtr->setMax(max_pt);
    Crop_Box_ConstPtr->setInputCloud(cloud);


    Crop_Box_ConstPtr->filter(*cropped_cloud);

    std::cout<<"size of cropped_cloud " <<cropped_cloud->points.size() <<std::endl;



    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cropped_cloud));
//    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));



    //Eigen::VectorXf plane1_initial_param_model_coefficients;
    Eigen::Vector4f plane1_initial_param_model_coefficients;

    // plane1_initial_param_model_coefficients are ax+by+cz+d=0
    //for the palne1 s and y could be anything and y must be something around -2.48/2 so a=0, c=0 and y=-d/b therefore d=-2.448 and b=2
    plane1_initial_param_model_coefficients(0,0)=0; //a
    plane1_initial_param_model_coefficients(1,0)=0; //b
    plane1_initial_param_model_coefficients(2,0)=1; //c
    plane1_initial_param_model_coefficients(3,0)=-3.5; //d


    double 	distance=0.20; //in meter
    std::vector< int > inliers;
    model_p->selectWithinDistance(plane1_initial_param_model_coefficients, distance, inliers);





    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
    ransac.setDistanceThreshold (0.05);
    ransac.computeModel();
    ransac.getInliers(inliers);

    pcl::copyPointCloud<pcl::PointXYZ>(*cropped_cloud, inliers, *final);
//    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);

    std::cout<<"size of final " <<final->points.size() <<std::endl;



    Eigen::VectorXf computed_plane1_initial_param_model_coefficients;
    ransac.getModelCoefficients(computed_plane1_initial_param_model_coefficients );



    std::cout<<"a: " <<computed_plane1_initial_param_model_coefficients(0,0) <<std::endl;
    std::cout<<"b: " <<computed_plane1_initial_param_model_coefficients(1,0) <<std::endl;
    std::cout<<"c: " <<computed_plane1_initial_param_model_coefficients(2,0) <<std::endl;
    std::cout<<"d: " <<computed_plane1_initial_param_model_coefficients(3,0) <<std::endl;



///////////////////////////////////////////////////////////         /////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }



/*
    boost::shared_ptr<pcl::CropBox< pcl::PointXYZ> > Crop_Box_ConstPtr(new pcl::CropBox< pcl::PointXYZ> );

    double width, heigh, lengh;
    //scales are in meter:
    width=4;
    heigh=5;
    lengh=15;

    //Crop_Box_ConstPtr->setInputCloud() 4x1
    Eigen::Vector4f min_pt,max_pt;

    min_pt(0,0)=-25;
    min_pt(1,0)=-2;
    min_pt(2,0)=0;


    max_pt(0,0)=0;
    max_pt(1,0)=2;
    max_pt(2,0)=5;


//    std::cout << "The matrix  is of size " << min_pt.rows() << "x" << min_pt.cols() << std::endl;


    Crop_Box_ConstPtr->setMin(min_pt);
    Crop_Box_ConstPtr->setMax(max_pt);
    Crop_Box_ConstPtr->setInputCloud(cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud (new pcl::PointCloud<pcl::PointXYZ>);


    pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);



    Crop_Box_ConstPtr->filter(*cropped_cloud);

    std::cout<<"size of cropped_cloud " <<cropped_cloud->points.size() <<std::endl;




    viewer = simpleVis(cropped_cloud);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cropped_cloud));


    //Eigen::VectorXf plane1_initial_param_model_coefficients;
    Eigen::Vector4f plane1_initial_param_model_coefficients;

    // plane1_initial_param_model_coefficients are ax+by+cz+d=0
    //for the palne1 s and y could be anything and y must be something around -2.48/2 so a=0, c=0 and y=-d/b therefore d=-2.448 and b=2
    plane1_initial_param_model_coefficients(0,0)=0; //a
    plane1_initial_param_model_coefficients(1,0)=2; //b
    plane1_initial_param_model_coefficients(2,0)=0; //c
    plane1_initial_param_model_coefficients(3,0)=-2.438; //d


    double 	threshold=0.05; //in meter
    std::vector< int > inliers;
    model_p->selectWithinDistance(plane1_initial_param_model_coefficients, threshold, inliers);





    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
    ransac.setDistanceThreshold (0.05);
    ransac.computeModel();
    ransac.getInliers(inliers);

    pcl::copyPointCloud<pcl::PointXYZ>(*cropped_cloud, inliers, *final);

    std::cout<<"size of final " <<final->points.size() <<std::endl;



    Eigen::VectorXf computed_plane1_initial_param_model_coefficients;
    ransac.getModelCoefficients(computed_plane1_initial_param_model_coefficients );



    std::cout<<"a: " <<computed_plane1_initial_param_model_coefficients(0,0) <<std::endl;
    std::cout<<"b: " <<computed_plane1_initial_param_model_coefficients(1,0) <<std::endl;
    std::cout<<"c: " <<computed_plane1_initial_param_model_coefficients(2,0) <<std::endl;
    std::cout<<"d: " <<computed_plane1_initial_param_model_coefficients(3,0) <<std::endl;


    //model_p->optimizeModelCoefficients();
    //model_p->computeModelCoefficients()


    //const std::vector<int> &samples, Eigen::VectorXf &model_coefficients

    //model_p->computeModelCoefficients


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    viewer = simpleVis(final);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    */
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis ( pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

void tmp(int argc, char* argv[])
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile(argv[1],*cloudrgb_ptr);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setRadiusSearch (0.1);
    ne.setInputCloud (cloudrgb_ptr);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (0.05);
    ne.compute (*cloud_normals1);

    viewer = normalsVis(cloudrgb_ptr, cloud_normals1);
    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}


void creating_template()
{

    double x_min, x_max, y_min,y_max, z_min, z_max;
//    321.191 25.774 3.668
//    321.253 23.555 3.95176
//    315.371 25.648 3.695
//    315.342 23.6616 3.91428

    x_min=316;
    x_max=322;

    y_min=23;
    y_max=25;

    z_min=3.6;
    z_max=3.90;

    double x_step_size, y_step_size, z_step_size;
     x_step_size=0.05;
     y_step_size=0.05;
     z_step_size=0.15;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ point;

    for(double x=x_min; x<x_max;x=x+x_step_size)
    {

        for(double y=y_min; y<y_max;y=y+y_step_size)
        {
//            for(double z=z_min; z<z_max;z=z+z_step_size)
//            {
//                point.x=x;
//                point.y=y;
//                point.z=z;
//                cloud_ptr->push_back(point);
//            }
            point.x=x;
            point.y=y;
            point.z=3.80;
            cloud_ptr->push_back(point);
        }
    }
    pcl::io::savePCDFile("artificially_generated_container_roof.pcd",*cloud_ptr);

}

int main(int argc, char* argv[])
{
//    truck_visulizer();
//    shipping_container_cutter();
    creating_template();
}
