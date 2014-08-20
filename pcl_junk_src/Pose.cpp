#include "Pose.h"


Pose::Pose(void)
{
	matrix.resize(16);
	
	matrix[ 0] = 1.0;	matrix[ 4] = 0.0;	matrix[ 8] = 0.0;	matrix[12] = 0.0;
	matrix[ 1] = 0.0;	matrix[ 5] = 1.0;	matrix[ 9] = 0.0;	matrix[13] = 0.0;
	matrix[ 2] = 0.0;	matrix[ 6] = 0.0;	matrix[10] = 1.0;	matrix[14] = 0.0;
	matrix[ 3] = 0.0;	matrix[ 7] = 0.0;	matrix[11] = 0.0;	matrix[15] = 1.0;
}

Pose::Pose(std::vector<GLdouble> matrix)
{
	this->matrix = matrix;
}

Pose::~Pose(void)
{
}

Pose Pose::fromMatrix4f(float* mdata)
{
	std::vector<GLdouble> matrix(16);
	matrix[ 0] = mdata[ 0];	matrix[ 4] = mdata[ 4];	matrix[ 8] = mdata[ 8];	matrix[12] = mdata[12];
	matrix[ 1] = mdata[ 1];	matrix[ 5] = mdata[ 5];	matrix[ 9] = mdata[ 9];	matrix[13] = mdata[13];
	matrix[ 2] = mdata[ 2];	matrix[ 6] = mdata[ 6];	matrix[10] = mdata[10];	matrix[14] = mdata[14];
	matrix[ 3] = mdata[ 3];	matrix[ 7] = mdata[ 7];	matrix[11] = mdata[11];	matrix[15] = mdata[15];
	
	return Pose(matrix);
}

void Pose::reset()
{
	Pose tmp;
	this->setMatrix(tmp.getMatrix());
}

GLdouble* Pose::getMatrixArr()
{
	GLdouble* arrayMatrix = new GLdouble[16];
	for(int i=0; i<16; i++)
		arrayMatrix[i] = matrix[i];

	return arrayMatrix;
}

std::vector<GLdouble> Pose::getMatrix()
{
	return matrix;
}

GLdouble* Pose::getInverseMatrixArr()
{
	GLdouble* inv = new GLdouble[16];
	inv[ 0] = matrix[ 0];	inv[ 4] = matrix[ 1];	inv[ 8] = matrix[ 2];	inv[12] = -(matrix[ 0] * matrix[12] + matrix[ 1] * matrix[13] + matrix[ 2] * matrix[14]);
	inv[ 1] = matrix[ 4];	inv[ 5] = matrix[ 5];	inv[ 9] = matrix[ 6];	inv[13] = -(matrix[ 4] * matrix[12] + matrix[ 5] * matrix[13] + matrix[ 6] * matrix[14]);
	inv[ 2] = matrix[ 8];	inv[ 6] = matrix[ 9];	inv[10] = matrix[ 0];	inv[14] = -(matrix[ 8] * matrix[12] + matrix[ 9] * matrix[13] + matrix[10] * matrix[14]);
	inv[ 3] = 0.0;			inv[ 7] = 0.0;			inv[11] = 0.0;			inv[15] = 1.0;
	return inv;
}

void Pose::setMatrix(std::vector<GLdouble> matrix)
{
	this->matrix = matrix;
}

void Pose::translate(double deltaX, double deltaY, double deltaZ)
{
	// NTO = WTO * OTN
	// NTO = New matrix in World coords
	// WTO = Old matrix in World coords
	// OTN = New matrix in Old matrix' coords (only translation p = [deltaX; deltaY; deltaZ])
	
	matrix[12] += deltaX * matrix[ 0] + deltaY * matrix[ 4] + deltaZ * matrix[ 8];
	matrix[13] += deltaX * matrix[ 1] + deltaY * matrix[ 5] + deltaZ * matrix[ 9];
	matrix[14] += deltaX * matrix[ 2] + deltaY * matrix[ 6] + deltaZ * matrix[10];
}

void Pose::translateInWorld(double deltaX, double deltaY, double deltaZ)
{
	matrix[12] += deltaX;
	matrix[13] += deltaY;
	matrix[14] += deltaZ;
}

void Pose::rotate(double angle, double axisX, double axisY, double axisZ)
{
	//upvector only needed for virtual camera
	std::vector<double> upVector(3);
	upVector[0] = upVector[1] = upVector[2] = 0;
	//position is origin of coordinate system
	std::vector<double> position(3);
	position[0] = position[1] = position[2] = 0;
	//rotation axis is given from parameters
	std::vector<double> vector(3);
	vector[0] = axisX;
	vector[1] = axisY;
	vector[2] = axisZ;
	this->rotate(angle, position, vector, upVector);
}

void Pose::rotate(double angle, const std::vector<double>& position, const std::vector<double>& vector)
{
	std::vector<double> upVector(3);
	upVector[0] = upVector[1] = upVector[2] = 0;
	this->rotate(angle, position, vector, upVector);
}

void Pose::rotate(double angle, const std::vector<double>& position, const std::vector<double>& vector, const std::vector<double>& upVector)
{
	translateInWorld(-position[0], -position[1], -position[2]);

	double alpha = angle * M_PI / 180.0;

	double  z1 = cos(alpha) + (vector[2]*vector[2]) * (1 - cos(alpha));
	double  z2 = vector[1] * vector[2] * (1 - cos(alpha));
	double  z3 = z2 + vector[0] * sin(alpha);
	double  z4 = vector[0] * vector[2] * (1 - cos(alpha));
	double  z5 = z4 - vector[1] * sin(alpha);
	double  z6 = z2 - vector[0] * sin(alpha);
	double  z7 = cos(alpha) + (vector[1]*vector[1]) * (1 - cos(alpha));
	double  z8 = z2 + vector[2] * sin(alpha);
	double  z9 = z4 + vector[1] * sin(alpha);
	double z10 = vector[0] * vector[1] * (1 - cos(alpha)) - vector[2] * sin(alpha);
	double z11 = cos(alpha) + (vector[0]*vector[0]) * (1 - cos(alpha));

	GLdouble* matNew = new GLdouble[16];
	matNew[ 0] = z11 * matrix[0] + z10 * matrix[1] + z9 * matrix[2];
	matNew[ 1] = z8 * matrix[0] + z7 * matrix[1] + z6 * matrix[2];
	matNew[ 2] = z5 * matrix[0] + z3 * matrix[1] + z1 * matrix[2];
	matNew[ 3] = 0;
	matNew[ 4] = z11 * matrix[4] + z10 * matrix[5] + z9 * matrix[6];
	matNew[ 5] = z8 * matrix[4] + z7 * matrix[5] + z6 * matrix[6];
	matNew[ 6] = z5 * matrix[4] + z3 * matrix[5] + z1 * matrix[6];
	matNew[ 7] = 0;
	matNew[ 8] = z11 * matrix[8] + z10 * matrix[9] + z9 * matrix[10];
	matNew[ 9] = z8 * matrix[8] + z7 * matrix[9] + z6 * matrix[10];
	matNew[10] = z5 * matrix[8] + z3 * matrix[9] + z1 * matrix[10];
	matNew[11] = 0;
	matNew[12] = z11 * matrix[12] + z10 * matrix[13] + z9 * matrix[14];
	matNew[13] = z8 * matrix[12] + z7 * matrix[13] + z6 * matrix[14];
	matNew[14] = z5 * matrix[12] + z3 * matrix[13] + z1 * matrix[14];
	matNew[15] = 1;
	
	//if upVector is not 0-vector, limit the rotation that camera-z-axis is not +/- upVector
	if(!(std::abs(upVector[0])<std::numeric_limits<double>::epsilon() && std::abs(upVector[1])<std::numeric_limits<double>::epsilon() && std::abs(upVector[2])<std::numeric_limits<double>::epsilon()))
	{
		//Log::getInstance()->log(Log::Debug, QString("Y to upVector is %1").arg(matNew[4]*upVector[0]+matNew[5]*upVector[1]+matNew[6]*upVector[2]));
		if(matNew[4]*upVector[0]+matNew[5]*upVector[1]+matNew[6]*upVector[2] >= 0)
		{
			for(int i=0; i<16; i++)
				matrix[i] = matNew[i];
		}
	}
	else
	{
		for(int i=0; i<16; i++)
			matrix[i] = matNew[i];
	}

	delete[] matNew;

	translateInWorld(position[0], position[1], position[2]);
}

Eigen::Affine3f Pose::toAffine3f() const
{
	Eigen::Affine3f output;
	output.matrix() <<
		matrix[ 0], matrix[ 4], matrix[ 8], matrix[12],
		matrix[ 1], matrix[ 5], matrix[ 9], matrix[13],
		matrix[ 2], matrix[ 6], matrix[10], matrix[14],
		matrix[ 3], matrix[ 7], matrix[11], matrix[15];
	return output;
}

void Pose::setPosition(double x, double y, double z)
{
	matrix[12] = x;
	matrix[13] = y;
	matrix[14] = z;
}