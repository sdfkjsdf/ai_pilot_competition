#include "LadyLuck/geometry/EulerAngle.h"
#include "LadyLuck/geometry/Quaternion.h"
#include "LadyLuck/geometry/AxisAngle.h"
#include "LadyLuck/geometry/Matrix3.h"

namespace BT_Geometry
{

	EulerAngle EulerAngle::_Zero(0.0, 0.0, 0.0);

	EulerAngle::EulerAngle(AxisAngle const& v)
	{
		v.toEulerAngle(this);
	}

	EulerAngle::EulerAngle(Matrix3 const& v)
	{
		v.toEulerAngle(this);
	}

	EulerAngle::EulerAngle(Quaternion const& v)
	{
		v.toEulerAngle(this);
	}

	void EulerAngle::toQuaternion(Quaternion* dest) const
	{
		double c1 = cos(Yaw / 2);   
		double s1 = sin(Yaw / 2);     
		double c2 = cos(Pitch / 2);   
		double s2 = sin(Pitch / 2);   
		double c3 =  cos(Roll / 2);    
		double s3 =	 sin(Roll / 2); 

		double c1c2 = c1*c2;    
		double s1s2 = s1*s2;    

		dest->W =c1c2*c3 + s1s2*s3;
		dest->X =c1*s2*c3 + s1*c2*s3;
		dest->Y =s1*c2*c3 - c1*s2*s3;
		dest->Z =c1*c2*s3 - s1*s2*c3;
		dest->normalize();
	}

	Quaternion EulerAngle::toQuaternion(void) const
	{
		Quaternion result;
		toQuaternion(&result);
		return result;
	}

	void EulerAngle::toMatrix(Matrix3* dest) const
	{    	
		double cy = cos(Yaw);
		double sy = sin(Yaw);
		double cp = cos(Pitch);
		double sp = sin(Pitch);
		double cr = cos(Roll);
		double sr = sin(Roll);

		Matrix3 rotY(cy, 0, -sy, 0, 1, 0, sy, 0, cy);
		Matrix3 rotP(1, 0, 0, 0, cp, sp, 0, -sp, cp);
		Matrix3 rotR(cr, sr, 0, -sr, cr, 0, 0, 0, 1);

		*dest = (rotY * rotP) * rotR;
	}

	Matrix3 EulerAngle::toMatrix(void) const
	{
		Matrix3 result;
		toMatrix(&result);
		return result;
	}

	void EulerAngle::toAxisAngle(AxisAngle* dest) const
	{
		double c1 = cos(Yaw/2.0);
		double s1 = sin(Yaw/2.0);
		double c2 = cos(Pitch/2.0);
		double s2 = sin(Pitch/2.0);
		double c3 = cos(Roll/2.0);
		double s3 = sin(Roll/2.0);
		double c1c2 = c1 * c2;
		double s1s2 = s1 * s2;

		double w;
		dest->Axis.X = c1*s2*c3 + s1*c2*s3;
		dest->Axis.Y = s1*c2*c3 - c1*s2*s3;
		dest->Axis.Z = c1c2*s3 - s1s2*c3;
		w = c1c2*c3 + s1s2*s3;
		dest->AngleValue = 2.0 * acos(w);

		double norm = dest->Axis.X * dest->Axis.X + dest->Axis.Y * dest->Axis.Y + dest->Axis.Z * dest->Axis.Z;

		if (norm < 0.000001) 
		{ 	
			dest->Axis.X = 1.0;
			dest->Axis.Y = dest->Axis.Z = 0.0;
		}
		else 
		{
			norm = sqrt(norm);
			dest->Axis.X /= norm;
			dest->Axis.Y /= norm;
			dest->Axis.Z /= norm;
		}
	}

	AxisAngle EulerAngle::toAxisAngle(void) const
	{
		AxisAngle result;
		toAxisAngle(&result);
		return result;
	}
}
