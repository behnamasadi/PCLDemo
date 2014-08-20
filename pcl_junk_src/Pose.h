#pragma once
#undef NOMINMAX
#define NOMINMAX

#include <Eigen/Geometry>
#include <qgl.h>
#include <vector>
#include <qmath.h>

class Pose
{
public:
	Pose(void);
	Pose(std::vector<GLdouble> matrix);
	static Pose fromMatrix4f(float* mdata);
	~Pose(void);
	GLdouble* getMatrixArr();
	std::vector<GLdouble> getMatrix();
	GLdouble *getInverseMatrixArr();
	void setMatrix(std::vector<GLdouble> matrix);
	void translate(double deltaX, double deltaY, double deltaZ);
	void translateInWorld(double deltaX, double deltaY, double deltaZ);
	void rotate(double angle, double axisX, double axisY, double axisZ);
	void rotate(double angle, const std::vector<double>& position, const std::vector<double>& vector);
	void rotate(double angle, const std::vector<double>& position, const std::vector<double>& vector, const std::vector<double>& upVector);
	void reset();
	Eigen::Affine3f toAffine3f() const;
	void setPosition(double x, double y, double z);

private:
	std::vector<GLdouble> matrix;
};

