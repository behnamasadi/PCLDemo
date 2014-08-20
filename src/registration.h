#ifndef REGISTRATION_H
#define REGISTRATION_H

#include <QtGui/QWidget>
#include <QMouseEvent>
#include <QDebug>
#include <QEvent>
#include <QFileDialog>
#include <QtGui>
#include <QDir>
#include <QSlider>
#include <QCoreApplication>
#include <QtCore/QDateTime>
#include <boost/shared_ptr.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread.hpp>

#include "ui_registration.h"

class Registration : public QWidget
{
    Q_OBJECT
    private slots:
    void SelectFolderContainingPCDFilespushButtonClicked();
    void VoxelGridSizeSlidervalueChanged();
    void DownSamplepushButtonclicked();
    void VoxelGridSizeSlidersliderReleased();
    void CalculateCorrespondencespushButtonclicked();

public:
    Registration(QWidget *parent = 0);
    ~Registration();
    void SetingUpSignals();
    void InitializingVariables();
    void UpdateDisplayImageWindow();
    template<typename PointT>
    void PointCloudVisualizer( const boost::shared_ptr< pcl::PointCloud<PointT> > points)
    {
    	if(this->Visualizer==boost::shared_ptr<pcl::visualization::PCLVisualizer>())
    	{
    		this->Visualizer=boost::shared_ptr<pcl::visualization::PCLVisualizer>( new pcl::visualization::PCLVisualizer("3D Viewer"));
			this->Visualizer->setBackgroundColor (0, 0, 0);
			this->Visualizer->addPointCloud< pcl::PointXYZRGB >(points ,"points");
    	}
    	else
    	{
    		this->Visualizer->close();

    		this->Visualizer->updatePointCloud < pcl::PointXYZRGB > (points);
//    		this->Visualizer->removeAllPointClouds();
//    		this->Visualizer->spinOnce (100);
    	}
//    	while (!this->Visualizer->wasStopped ())
//    	{
//			this->Visualizer->spinOnce (100);
//			boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//    	}

		this->Visualizer->spin();
/*
    	pcl::visualization::PCLVisualizer viz;
    	viz.addPointCloud ( points, "points");
    	viz.spin ();
*/
    	return;
    }

private:
    Ui::RegistrationClass ui;
    std::string FileExtension;
    std::vector<std::string> ListofFiles;
    std::string FolderContainingPCDFilesFullPath;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> Visualizer;
};

#endif // REGISTRATION_H
