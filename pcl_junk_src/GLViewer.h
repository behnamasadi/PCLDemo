#pragma once
#undef NOMINMAX
#define NOMINMAX

#include <pcl/point_types.h>
#include <QGLWidget>
#include <GL/glu.h>
#include <vector>
#include <QMouseEvent>
#include <qpoint.h>
#include "Pose.h"

class GLViewer : public QGLWidget
{
	Q_OBJECT

public:
	GLViewer(QWidget *parent = 0);
	~GLViewer();
	
	void setCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr getSelection();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud();
	void select(double x, double y, double z);
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void wheelEvent(QWheelEvent *event);
	
protected:
	void initializeGL();
	void resizeGL(int w, int h);
	void paintGL();
	
private:
	boost::shared_ptr<Pose> camera;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr selectedPoints;
	enum TransformationMode{
        Translation,
        Rotation,
        Zoom,
		Select
    };
	TransformationMode tMode;
	QPoint clickPosition;
	QPoint firstClickPosition;
	std::vector<double> focusPosition;
	
	void translateCamera(double deltaX, double deltaY);
	void zoomCamera(double deltaZoom);
	void rotateCamera(double angleYaw, double anglePitch);
	void selectAt(QPoint clickPosition);
	
public slots:
	void changeTMode_Translation(void);
	void changeTMode_Rotation(void);
	void changeTMode_Zoom(void);
	void changeTMode_Select(void);

signals:
	void selectionChanged();
};