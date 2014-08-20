#include "GLViewer.h"

GLViewer::GLViewer(QWidget *parent)
	:QGLWidget(parent)
{
	camera = boost::shared_ptr<Pose>(new Pose());
	//normal view (Z=upvector)
	camera->rotate(90.0, 1,0,0);
	camera->rotate(-90.0, 0,0,1);

	cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
	selectedPoints = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	
	focusPosition.resize(3);
	focusPosition[0] = 0.0; focusPosition[1] = 0.0; focusPosition[2] = 0.0;

	tMode = Rotation;
}

GLViewer::~GLViewer()
{
}

void GLViewer::setCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud)
{
	cloud = newCloud;

	//set focus position to center of gravity
	double cx = 0.0, cy = 0.0, cz = 0.0;
	int cloudSize = cloud->points.size();
	pcl::PointCloud<pcl::PointXYZRGB>::iterator it;
	for(it = cloud->begin(); it < cloud->end(); it++)
	{
		cx += it->x;
		cy += it->y;
		cz += it->z;
	}
	focusPosition[0] = cx/cloudSize; focusPosition[1] = cy/cloudSize; focusPosition[2] = cz/cloudSize; 
	
	updateGL();
}

void GLViewer::initializeGL()
{	
	glShadeModel(GL_SMOOTH);

	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
}

void GLViewer::resizeGL(int w, int h)
{
	if (this->height()==0)
	{
		this->resize(this->width(), 1);
	}

	glViewport(0, 0, width(), height());
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	
	updateGL();
}

void GLViewer::paintGL()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	float cameraAngle = 45.0f;
	gluPerspective(cameraAngle, (GLfloat)width()/(GLfloat)height(), 0.1f, 100.0f);

	glMatrixMode(GL_MODELVIEW);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	//handle camera pose
	glLoadIdentity();
	glMultMatrixd(camera->getInverseMatrixArr());
	
	//draw pointcloud
	glPointSize(1);
	glBegin(GL_POINTS);
	pcl::PointCloud<pcl::PointXYZRGB>::iterator it;
	for(it = cloud->begin(); it < cloud->end(); it++)
	{
		glColor3ub(it->r, it->g, it->b);
		glVertex3d(it->x, it->y, it->z);
	}
	glEnd();

	//draw selected Points
	glPointSize(3);
	glBegin(GL_POINTS);
	glColor3ub(255,128,0); //orange
	pcl::PointCloud<pcl::PointXYZ>::iterator it2;
	for(it2 = selectedPoints->begin(); it2 < selectedPoints->end(); it2++)
	{
		glVertex3d(it2->x, it2->y, it2->z);
	}
	glEnd();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr GLViewer::getSelection()
{
	return selectedPoints;
}

void GLViewer::mousePressEvent(QMouseEvent *event)
{
	clickPosition = event->pos();
	firstClickPosition = event->pos();
	mouseMoveEvent(event);
}

void GLViewer::mouseReleaseEvent(QMouseEvent *event)
{
	clickPosition.setX(-1);
	clickPosition.setY(-1);
}

void GLViewer::mouseMoveEvent(QMouseEvent *event)
{
	if(clickPosition.x() == -1 && clickPosition.y() == -1)
		return;

	double deltaX = (double)(event->x() - clickPosition.x());
	double deltaY = (double)(event->y() - clickPosition.y());

	switch(tMode)
	{
		case Translation:
			translateCamera(deltaX, deltaY);
			break;
		case Rotation:
			rotateCamera(deltaX, deltaY);
			break;
		case Zoom:
			zoomCamera(deltaY);
			break;
		case Select:
			selectAt(clickPosition);
	}

	clickPosition = event->pos();

	updateGL();
}

void GLViewer::wheelEvent(QWheelEvent *event)
{
	zoomCamera(- event->delta());
	updateGL();
}

void GLViewer::translateCamera(double deltaX, double deltaY)
{
	camera->translate(-deltaX*0.01, deltaY*0.01, 0.0);
}

void GLViewer::zoomCamera(double deltaZoom)
{
	camera->translate(0.0, 0.0, -deltaZoom*0.01);
}

void GLViewer::rotateCamera(double angleYaw, double anglePitch)
{
	std::vector<GLdouble> upVector(3);
	upVector[0] = 0; upVector[1] = 0; upVector[2] = 1;
	camera->rotate(-angleYaw*0.25, focusPosition, upVector);
	std::vector<double> axis(camera->getMatrix());
	axis.resize(3);
	camera->rotate(-anglePitch*0.25,focusPosition, axis, upVector);
}

void GLViewer::selectAt(QPoint clickPosition)
{
	if(cloud->empty())
		return;

	//Draw cloud in buffer (with their unique colors)
	glDisable(GL_DITHER);
	glMatrixMode(GL_MODELVIEW);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	glLoadIdentity();
	glMultMatrixd(camera->getInverseMatrixArr());
	
	glPointSize(3);
	glBegin(GL_POINTS);
	int cloudSize = cloud->points.size();
	for(int i = 0; i < cloudSize; i++)
	{
		int r = (0xff0000 & i+1) >> 16;
		int g = (0x00ff00 & i+1) >> 8;
		int b = (0x0000ff & i+1);
		glColor3ub(r,g,b);
		glVertex3d(cloud->at(i).x, cloud->at(i).y, cloud->at(i).z);
	}
	glEnd();

	glEnable(GL_DITHER);

	//read pixel color at position
	GLint viewport[4];
	GLubyte pixel[3];

	glGetIntegerv(GL_VIEWPORT,viewport);

	glReadPixels(clickPosition.x(),viewport[3]-clickPosition.y(),1,1,GL_RGB,GL_UNSIGNED_BYTE,(void *)pixel);
	
	//get index for that color
	int idx = (pixel[0] << 16) | (pixel[1] << 8) | pixel[2] - 1;

	selectedPoints->clear();
	if(idx > -1)
		selectedPoints->push_back(pcl::PointXYZ(cloud->at(idx).x, cloud->at(idx).y, cloud->at(idx).z));
	emit selectionChanged();
}

void GLViewer::select(double x, double y, double z)
{
	selectedPoints->clear();
	selectedPoints->push_back(pcl::PointXYZ(x,y,z));
	updateGL();
}

void GLViewer::changeTMode_Translation()
{
	this->tMode = Translation;
}

void GLViewer::changeTMode_Rotation()
{
	this->tMode = Rotation;
}

void GLViewer::changeTMode_Zoom()
{
	this->tMode = Zoom;
}

void GLViewer::changeTMode_Select()
{
	this->tMode = Select;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr GLViewer::getCloud()
{
	return cloud;
}