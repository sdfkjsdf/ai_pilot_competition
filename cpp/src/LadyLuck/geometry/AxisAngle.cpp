#include "LadyLuck/geometry/AxisAngle.h"
#include "LadyLuck/geometry/Matrix3.h"
#include "LadyLuck/geometry/Quaternion.h"
#include "LadyLuck/geometry/EulerAngle.h"

namespace BT_Geometry
{

AxisAngle AxisAngle::_Zero(Vector3::Zero(), 0.0);

AxisAngle::AxisAngle(Matrix3 const& v)
{
	v.toAxisAngle(this);
}

AxisAngle::AxisAngle(Quaternion const& v)
{
	v.toAxisAngle(this);
}

AxisAngle::AxisAngle(EulerAngle const& v)
{
	v.toAxisAngle(this);
}

void AxisAngle::toQuaternion(Quaternion* dest) const
{
	Angle halfAngle (0.5 * AngleValue);
    double sinValue = sin(halfAngle );
    dest->W = cos(halfAngle );
    dest->X = sinValue * Axis.X;
    dest->Y = sinValue * Axis.Y;
    dest->Z = sinValue * Axis.Z;
}

Quaternion AxisAngle::toQuaternion(void) const
{
	Quaternion quaternion;
	toQuaternion(&quaternion);
	return quaternion;
}

void AxisAngle::toMatrix(Matrix3* dest) const
{
	double cosAngle = cos(AngleValue);
    double sinAngle  = sin(AngleValue);
    double oneMinusCos = 1.0 - cosAngle;
    double X2 = Axis.X * Axis.X;
    double Y2 = Axis.Y * Axis.Y;
    double Z2 = Axis.Z * Axis.Z;
    double XYM  = Axis.X * Axis.Y * oneMinusCos;
    double XZM  = Axis.X * Axis.Z * oneMinusCos;
    double YZM  = Axis.Y * Axis.Z * oneMinusCos;
    double sinX = Axis.X * sinAngle ;
    double sinY = Axis.Y * sinAngle ;
    double sinZ = Axis.Z * sinAngle ;

    dest->M00 = X2 * oneMinusCos + cosAngle;
    dest->M10 = XYM  - sinZ;
    dest->M20 = XZM  + sinY;
    dest->M01 = XYM  + sinZ;
    dest->M11 = Y2 * oneMinusCos + cosAngle;
    dest->M21 = YZM  - sinX;
    dest->M02 = XZM  - sinY;
    dest->M12 = YZM  + sinX;
    dest->M22 = Z2 * oneMinusCos + cosAngle;

}

Matrix3 AxisAngle::toMatrix(void) const
{
	Matrix3 result;
	toMatrix(&result);
	return result;
}

void AxisAngle::toEulerAngle(EulerAngle* dest) const
{
	//TODO : �ð����� �����ϰ���
	*dest = this->toMatrix().toEulerAngle();
}

EulerAngle AxisAngle::toEulerAngle(void) const
{
	EulerAngle result;
	toEulerAngle(&result);
	return result;
}
}
