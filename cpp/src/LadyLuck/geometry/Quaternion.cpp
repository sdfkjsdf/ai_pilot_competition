#include "LadyLuck/geometry/Quaternion.h"
#include "LadyLuck/geometry/Vector4.h"
#include "LadyLuck/geometry/Matrix3.h"
#include "LadyLuck/geometry/Vector3.h"
#include "LadyLuck/geometry/AxisAngle.h"
#include "LadyLuck/geometry/EulerAngle.h"

namespace BT_Geometry
{
Quaternion Quaternion::_Identity = Quaternion(0.0, 0.0, 0.0, 1.0);

Quaternion::Quaternion(AxisAngle const& a)
{
	a.toQuaternion(this);
}

Quaternion::Quaternion(Matrix3 const& m)
{
	m.toQuaternion(this);
}

Quaternion::Quaternion(EulerAngle const& e)
{
	e.toQuaternion(this);
}

double Quaternion::length(void) const
{
	return sqrt(lengthSquared());
}

double Quaternion::lengthSquared(void) const
{
	return (X * X) + (Y * Y) + (Z * Z) + (W * W);
}
void Quaternion::normalize(void)
{
	double l = 1.0 / length();
	X *= l;
	Y *= l;
	Z *= l;
	W *= l;
}
void Quaternion::conjugate(void)
{
	X = -X;
	Y = -Y;
	Z = -Z;
}
Quaternion Quaternion::inverse(void) const
{
	Quaternion result;
	inverse(&result);
	return result;
}

void Quaternion::inverse(Quaternion* dest) const
{ 
	double length = lengthSquared();

	if (length > 0.0)
	{
		double invLength = 1.0 / length;
		dest->X = -X * invLength;
		dest->Y = -Y * invLength;
		dest->Z = -Z * invLength;
		dest->W = W * invLength;
	}
	else
	{
		*dest = Identity();
	}
}

double Quaternion::dot(Quaternion const& rhv) const
{
	return (X * rhv.X) + (Y * rhv.Y) + (Z * rhv.Z) + (W * rhv.W);
}

void Quaternion::lerp(Quaternion const& start, Quaternion const& end, double amount)
{
	double inverse = 1.0 - amount;
	double dotValue = start.dot(end);

	if (dotValue >= 0.0)
	{
		X = (inverse * start.X) + (amount * end.X);
		Y = (inverse * start.Y) + (amount * end.Y);
		Z = (inverse * start.Z) + (amount * end.Z);
		W = (inverse * start.W) + (amount * end.W);

	}
	else
	{
		X = (inverse * start.X) - (amount * end.X);
		Y = (inverse * start.Y) - (amount * end.Y);
		Z = (inverse * start.Z) - (amount * end.Z);
		W = (inverse * start.W) - (amount * end.W);
	}

	double invLength = 1.0 / length();
	X *= invLength;
	Y *= invLength;
	Z *= invLength;
	W *= invLength;
}

void Quaternion::sLerp(Quaternion const& startPoint, Quaternion const& endPoint, double factor, bool clampToMinorAngle)
{
	Quaternion localEndPoint;
	
	double dot = startPoint.dot(endPoint);

	if(clampToMinorAngle == true)
	{
		if(dot >= 0.0)
		{
			localEndPoint = endPoint;
		}
		else
		{
			localEndPoint = -endPoint;
			dot = -dot;
		}
	}
	else
	{
		localEndPoint = endPoint;
	}

	if(dot <= 0.95)
	{
		double interAngle = acos(dot);
		double sinInverseAmountAngle = sin(interAngle * (1.0 - factor));
		double sinAmountAngle = sin(interAngle * factor);
		double sinDenominator = sin(interAngle);

		X = ((startPoint.X * sinInverseAmountAngle) + (localEndPoint.X * sinAmountAngle)) / sinDenominator;
		Y = ((startPoint.Y * sinInverseAmountAngle) + (localEndPoint.Y * sinAmountAngle)) / sinDenominator;
		Z = ((startPoint.Z * sinInverseAmountAngle) + (localEndPoint.Z * sinAmountAngle)) / sinDenominator;
		W = ((startPoint.W * sinInverseAmountAngle) + (localEndPoint.W * sinAmountAngle)) / sinDenominator;
	}
	else
	{
		lerp(startPoint, localEndPoint, factor);
		
	/*	Matrix3 orientationMatrix = toMatrix();
		Vector3 xAxis = orientationMatrix.getXAxis();
		Vector3 zAxis = orientationMatrix.getZAxis();
		Vector3 yAxis = zAxis.cross(xAxis);
		xAxis = yAxis.cross(zAxis);
		orientationMatrix.setXAxis(xAxis);
		orientationMatrix.setYAxis(yAxis);

		*this = orientationMatrix.toQuaternion();*/
	}

	normalize();
}

Matrix3 Quaternion::toMatrix(void) const
{
	Matrix3 result;
	toMatrix(&result);
	return result;
}


void Quaternion::toMatrix(Matrix3* dest) const
{
	double xx = X * X;
	double yy = Y * Y;
	double zz = Z * Z;
	double xy = X * Y;
	double zw = Z * W;
	double zx = Z * X;
	double yw = Y * W;
	double yz = Y * Z;
	double xw = X * W;
	dest->M00 = 1.0 - (2.0 * (yy + zz));
	dest->M01 = 2.0 * (xy + zw);
	dest->M02 = 2.0 * (zx - yw);
	dest->M10 = 2.0 * (xy - zw);
	dest->M11 = 1.0 - (2.0 * (zz + xx));
	dest->M12 = 2.0 * (yz + xw);
	dest->M20 = 2.0 * (zx + yw);
	dest->M21 = 2.0 * (yz - xw);
	dest->M22 = 1.0 - (2.0 * (yy + xx));


}

AxisAngle Quaternion::toAxisAngle(void) const
{
	AxisAngle axisAngle;
	toAxisAngle(&axisAngle);
	return axisAngle;
}

void Quaternion::toAxisAngle(AxisAngle* dest) const
{
	double length = X * X + Y * Y + Z * Z;
    if ( length > 0.0 )
    {
        dest->AngleValue = 2.0 * cos(W);
        double InvLength = 1.0 / sqrt(length);
        dest->Axis.X = X * InvLength;
        dest->Axis.Y = Y * InvLength;
        dest->Axis.Z = Z * InvLength;
    }
    else
    {
        // angle is 0 (mod 2 * pi), so anY aXis will do
        dest->AngleValue = 0.0;
        dest->Axis.X = 1.0;
        dest->Axis.X = 0.0;
        dest->Axis.X = 0.0;
    }
}

EulerAngle Quaternion::toEulerAngle(void) const
{
	EulerAngle result;
	toEulerAngle(&result);
	return result;
}

void Quaternion::toEulerAngle(EulerAngle* dest) const
{
	double test = Y * Z - X * W;
	if (test < -0.499999999)
	{
		dest->Yaw = 2 * atan2(Y, W);
		dest->Pitch = PI_OVER_2;
		dest->Roll = 0;
		return;
	}
	if (test > 0.499999999) 
	{
		dest->Yaw = -2 * atan2(Y, W);
		dest->Pitch = -PI_OVER_2;
		dest->Roll = 0;
		return;
	}

	double sqx = X * X;
	double sqy = Y * Y;
	double sqz = Z * Z;

	dest->Yaw = atan2(2 * X * Z + 2 * Y * W, 1 - 2 * sqx - 2 * sqy);
	dest->Pitch = -asin(2 * test);
	dest->Roll = atan2(2 * X * Y + 2 * W * Z, 1 - 2 * sqx - 2 * sqz);
}
}
