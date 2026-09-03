#include "LadyLuck/geometry/Matrix3.h"
#include "LadyLuck/geometry/Quaternion.h"
#include "LadyLuck/geometry/EulerAngle.h"
#include "LadyLuck/geometry/AxisAngle.h"
#include "LadyLuck/geometry/Math.h"

namespace BT_Geometry
{
Matrix3 Matrix3::_Identity = Matrix3
	(1.0, 0.0, 0.0,
	 0.0, 1.0, 0.0,
	 0.0, 0.0, 1.0); 

Matrix3::Matrix3 (Quaternion const& v)
{
	v.toMatrix(this);
}

Matrix3::Matrix3 (AxisAngle const& v)
{
	v.toMatrix(this);
}

Matrix3::Matrix3 (EulerAngle const& v)
{
	v.toMatrix(this);
}

void Matrix3::inverse(Matrix3* dest) const
{

	dest->M00 = M11*M22 - M21*M12; dest->M10 = M20*M12 - M10*M22; dest->M20 = M10*M21 - M20*M11;
    dest->M01 = M21*M02 - M01*M22; dest->M11 = M00*M22 - M20*M02; dest->M21 = M20*M01 - M00*M21;
	dest->M02 = M01*M12 - M11*M02; dest->M12 = M10*M02 - M00*M12; dest->M22 = M00*M11 - M10*M01;

    double det =
        M00 * dest->M00 +
        M10 * dest->M01+
        M20 * dest->M02;

    if ( fabs(det) <= 1e-06)
	//if ( fabs(det) <= DBL_EPSILON)
	{
		*dest = Identity();
	}
	else
	{
		double invDet = 1.0 / det;
		for (int row = 0; row < 3; row++)
		{
			for (int col = 0; col < 3; col++)
				(*dest)[row * 3 + col] *= invDet;
		}
	}

}
Matrix3 Matrix3::inverse(void) const
{
	Matrix3 result;
	inverse(&result);
	return result;
}



void Matrix3::lerp(Matrix3 const& startMat, Matrix3 const& endMat, double amount)
{
	M00 = startMat.M00 + ((endMat.M00 - startMat.M00) * amount);
	M01 = startMat.M01 + ((endMat.M01 - startMat.M01) * amount);
	M02 = startMat.M02 + ((endMat.M02 - startMat.M02) * amount);

	M10 = startMat.M10 + ((endMat.M10 - startMat.M10) * amount);
	M11 = startMat.M11 + ((endMat.M11 - startMat.M11) * amount);
	M12 = startMat.M12 + ((endMat.M12 - startMat.M12) * amount);

	M20 = startMat.M20 + ((endMat.M20 - startMat.M20) * amount);
	M21 = startMat.M21 + ((endMat.M21 - startMat.M21) * amount);
	M22 = startMat.M22 + ((endMat.M22 - startMat.M22) * amount);
	
	
}

void Matrix3::toQuaternion	(Quaternion* dest) const
{
	double trace = M00+M11+M22;

    if ( trace > 0.0 )
    {
		double sqrtScale;

        sqrtScale = sqrt(trace + 1.0);  // 2w
        dest->W = 0.5*sqrtScale;
        sqrtScale = 0.5/sqrtScale;  // 1/(4w)
        dest->X = (M12-M21)*sqrtScale;
        dest->Y = (M20-M02)*sqrtScale;
        dest->Z = (M01-M10)*sqrtScale;
    }
	else if ( (M00 >= M11) && (M00 >= M22))
	{
		double sqrtScale = sqrt(1.0 + M00 - M11 - M22);
		double half = 0.5 / sqrtScale;

		dest->X = 0.5 * sqrtScale;
		dest->Y = (M01 + M10) * half;
		dest->Z = (M02 + M20) * half;
		dest->W = (M12 - M21) * half;
	}
	else if (M11 > M22)
	{
		double sqrtScale = sqrt(1.0 + M11 -M00 - M22);
		double half = 0.5 / sqrtScale;

		dest->X = (M10 + M01) * half;
		dest->Y = 0.5 * sqrtScale;
		dest->Z = (M21 + M12) * half;
		dest->W = (M20 - M02) * half;
	}
	else
	{
		double sqrtScale = sqrt(1.0 + M22 - M00 - M11);
		double half = 0.5 / sqrtScale;

		dest->X = (M20 + M02) * half;
		dest->Y = (M21 + M12) * half;
		dest->Z = 0.5 * sqrtScale;
		dest->W = (M01 - M10) * half;
	}
   
	dest->normalize();
}

Quaternion	Matrix3::toQuaternion	(void) const
{
	Quaternion quaternion;
	toQuaternion(&quaternion);	
	return quaternion;
}

void Matrix3::toAxisAngle(AxisAngle* dest) const
{
	double trace = M00 + M11 + M22;
    double cosValue = 0.5 * (trace-1.0);
    dest->AngleValue = cos(cosValue);  // in [0,PI]

    if ( dest->AngleValue > 0.0)
    {
        if ( dest->AngleValue < PI)
        {
            dest->Axis.X = M12 - M21;
            dest->Axis.Y = M20 - M02;
            dest->Axis.Z = M01 - M10;
            dest->Axis.normalize();
        }
        else
        {
            // angle is PI
            double halfInverse;
            if ( M00 >= M11 )
            {
                // r00 >= r11
                if ( M00 >= M22 )
                {
                    // r00 is maximum diagonal term
                    dest->Axis.X = 0.5 * sqrt(M00 -
                        M11 - M22 + 1.0);
                    halfInverse = 0.5 / dest->Axis.X;
                    dest->Axis.Y = halfInverse * M10;
                    dest->Axis.Z = halfInverse * M20;
                }
                else
                {
                    // r22 is maximum diagonal term
                    dest->Axis.Z = 0.5 * sqrt(M22 -
                        M00 - M11 + 1.0);
                    halfInverse = 0.5/dest->Axis.Z;
                    dest->Axis.X = halfInverse * M20;
                    dest->Axis.Y = halfInverse * M21;
                }
            }
            else
            {
                // r11 > r00
                if ( M11 >= M22 )
                {
                    // r11 is maximum diagonal term
                    dest->Axis.Y = 0.5 * sqrt(M11 -
                        M00 - M22 + 1.0);
                    halfInverse  = 0.5 / dest->Axis.Y;
                    dest->Axis.X = halfInverse * M10;
                    dest->Axis.Z = halfInverse * M21;
                }
                else
                {
                    // r22 is maximum diagonal term
                    dest->Axis.Z = 0.5 * sqrt(M22 -
                        M00 - M11 + 1.0);
                    halfInverse = 0.5 / dest->Axis.Z;
                    dest->Axis.X = halfInverse * M20;
                    dest->Axis.Y = halfInverse * M21;
                }
            }
        }
    }
    else
    {
        // The angle is 0 and the matrix is the identity.  Any axis will
        // work, so just use the x-axis.
        dest->Axis.X = 1.0;
        dest->Axis.Y = 0.0;
        dest->Axis.Z = 0.0;
    }	
}

AxisAngle Matrix3::toAxisAngle(void) const
{
	AxisAngle result;
	toAxisAngle(&result);
	return result;
}

void Matrix3::toEulerAngle(EulerAngle* dest) const
{
	if (M21 < -0.999999998)
	{
		dest->Yaw = atan2(-M02, M00);
		dest->Pitch = PI_OVER_2;
		dest->Roll = 0;

		return;	
	}
	if (M21 > 0.9999999998)
	{
		dest->Yaw = atan2(M02, M00);
		dest->Pitch = -PI_OVER_2;
		dest->Roll = 0;
		return;
	}

	dest->Yaw = atan2(M20, M22);
	dest->Pitch = -asin(M21);
	dest->Roll = atan2(M01, M11);
		
}

EulerAngle Matrix3::toEulerAngle(void) const
{   
	EulerAngle result;
	toEulerAngle(&result);
	return result;
}

#define swap(a,b) t = a; a = b; b = t;
void Matrix3::transpose()
{
	double t;
	swap(M10, M01);
	swap(M20, M02);
	swap(M12, M21);

}
}

