#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>


#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>


class SimpleOpenNIViewer
{
int i;

template<typename to, typename from>
to lexical_cast(from const &x)
{
	std::stringstream os;
	to ret;
	os << x;
	os >> ret;
	return ret;
}

public:
     SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

     void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
     {
		if (!viewer.wasStopped())
		{
			viewer.showCloud (cloud);
			std::string filename=lexical_cast<std::string>(i);
			//pcl::io::savePCDFileASCII(filename+ ".pcd",*cloud);
		}
     }

     void run ()
     {
		i=0;
    	 pcl::Grabber* interface = new pcl::OpenNIGrabber();

		boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);
		interface->registerCallback (f);
		interface->start ();
		while (!viewer.wasStopped())
		{
		   boost::this_thread::sleep (boost::posix_time::seconds (1));
		   i++;
		   cout<<"i= " <<i++<<endl;
		}
		interface->stop ();
     }

     pcl::visualization::CloudViewer viewer;
 };

 int main ()
 {
   SimpleOpenNIViewer v;
   v.run ();
   return 0;
 }
