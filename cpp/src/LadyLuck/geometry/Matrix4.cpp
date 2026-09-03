#include "LadyLuck/geometry/Matrix4.h"
#include "LadyLuck/geometry/Matrix3.h"
#include "LadyLuck/geometry/Quaternion.h"

namespace BT_Geometry
{

Matrix4 Matrix4::_Identity = Matrix4
	(1.0, 0.0, 0.0, 0.0,
	 0.0, 1.0, 0.0, 0.0,
	 0.0, 0.0, 1.0, 0.0,
	 0.0, 0.0, 0.0, 1.0); 


double Matrix4::determinant() const
{
	Vector4 minor, v1, v2, v3;
    double det;

    v1.X = M00; v1.Y = M10; v1.Z = M20; v1.W = M30;
    v2.X = M01; v2.Y = M11; v2.Z = M21; v2.W = M31;
    v3.X = M02; v3.Y = M12; v3.Z = M22; v3.W = M32;

    minor.cross(v1, v2, v3);
    det =  - (M03 * minor.X + M13 * minor.Y + M23 * minor.Z + M33 * minor.W);
    return det;
}
void Matrix4::inverse(Matrix4* dest) const
{
	if (M03 != 0 || M13 != 0 || M23 != 0 || M33 != 1)
	{
        double m00 = M00, m10 = M10, m20 = M20, m30 = M30;
        double m01 = M01, m11 = M11, m21 = M21, m31 = M31;
        double m02 = M02, m12 = M12, m22 = M22, m32 = M32;
        double m03 = M03, m13 = M13, m23 = M23, m33 = M33;

        double v0 = m02 * m13 - m12 * m03;
        double v1 = m02 * m23 - m22 * m03;
        double v2 = m02 * m33 - m32 * m03;
        double v3 = m12 * m23 - m22 * m13;
        double v4 = m12 * m33 - m32 * m13;
        double v5 = m22 * m33 - m32 * m23;

        double t00 = + (v5 * m11 - v4 * m21 + v3 * m31);
        double t01 = - (v5 * m01 - v2 * m21 + v1 * m31);
        double t02 = + (v4 * m01 - v2 * m11 + v0 * m31);
        double t03 = - (v3 * m01 - v1 * m11 + v0 * m21);

        double invDet = 1 / (t00 * m00 + t01 * m10 + t02 * m20 + t03 * m30);

        double d00 = t00 * invDet;
        double d01 = t01 * invDet;
        double d02 = t02 * invDet;
        double d03 = t03 * invDet;

        double d10 = - (v5 * m10 - v4 * m20 + v3 * m30) * invDet;
        double d11 = + (v5 * m00 - v2 * m20 + v1 * m30) * invDet;
        double d12 = - (v4 * m00 - v2 * m10 + v0 * m30) * invDet;
        double d13 = + (v3 * m00 - v1 * m10 + v0 * m20) * invDet;

        v0 = m01 * m13 - m11 * m03;
        v1 = m01 * m23 - m21 * m03;
        v2 = m01 * m33 - m31 * m03;
        v3 = m11 * m23 - m21 * m13;
        v4 = m11 * m33 - m31 * m13;
        v5 = m21 * m33 - m31 * m23;

        double d20 = + (v5 * m10 - v4 * m20 + v3 * m30) * invDet;
        double d21 = - (v5 * m00 - v2 * m20 + v1 * m30) * invDet;
        double d22 = + (v4 * m00 - v2 * m10 + v0 * m30) * invDet;
        double d23 = - (v3 * m00 - v1 * m10 + v0 * m20) * invDet;

        v0 = m12 * m01 - m02 * m11;
        v1 = m22 * m01 - m02 * m21;
        v2 = m32 * m01 - m02 * m31;
        v3 = m22 * m11 - m12 * m21;
        v4 = m32 * m11 - m12 * m31;
        v5 = m32 * m21 - m22 * m31;

        double d30 = - (v5 * m10 - v4 * m20 + v3 * m30) * invDet;
        double d31 = + (v5 * m00 - v2 * m20 + v1 * m30) * invDet;
        double d32 = - (v4 * m00 - v2 * m10 + v0 * m30) * invDet;
        double d33 = + (v3 * m00 - v1 * m10 + v0 * m20) * invDet;

		dest->M00 = d00; dest->M10 = d10; dest->M20 = d20; dest->M30 = d30;
		dest->M01 = d01; dest->M11 = d11; dest->M21 = d21; dest->M31 = d31;
		dest->M02 = d02; dest->M12 = d12; dest->M22 = d22; dest->M32 = d32;
		dest->M03 = d03; dest->M13 = d13; dest->M23 = d23; dest->M33 = d33;
	}
	else
	{
        double m01 = M01, m11 = M11, m21 = M21;
        double m02 = M02, m12 = M12, m22 = M22;

        double t00 = m22 * m11 - m12 * m21;
        double t01 = m02 * m21 - m22 * m01;
        double t02 = m12 * m01 - m02 * m11;

        double m00 = M00, m10 = M10, m20 = M20;

        double invDet = 1 / (m00 * t00 + m10 * t01 + m20 * t02);

        t00 *= invDet; t01 *= invDet; t02 *= invDet;

        m00 *= invDet; m10 *= invDet; m20 *= invDet;

        double r00 = t00;
        double r10 = m20 * m12 - m10 * m22;
        double r20 = m10 * m21 - m20 * m11;

        double r01 = t01;
        double r11 = m00 * m22 - m20 * m02;
        double r21 = m20 * m01 - m00 * m21;

        double r02 = t02;
        double r12 = m10 * m02 - m00 * m12;
        double r22 = m00 * m11 - m10 * m01;

        double m30 = M30, m31 = M31, m32 = M32;

        double r30 = - (r00 * m30 + r10 * m31 + r20 * m32);
        double r31 = - (r01 * m30 + r11 * m31 + r21 * m32);
        double r32 = - (r02 * m30 + r12 * m31 + r22 * m32);

		dest->M00 = r00; dest->M10 = r10; dest->M20 = r20; dest->M30 = r30;
		dest->M01 = r01; dest->M11 = r11; dest->M21 = r21; dest->M31 = r31;
		dest->M02 = r02; dest->M12 = r12; dest->M22 = r22; dest->M32 = r32;
		dest->M03 = 0;	 dest->M13 = 0;	  dest->M23 = 0;   dest->M33 = 1;
	}
}

Matrix4 Matrix4::inverse(void) const
{
	Matrix4 result; 
	this->inverse(&result);
	return result;
}



void Matrix4::lerp(Matrix4 const& startMat, Matrix4 const& endMat, double amount)
{
	M00 = startMat.M00 + ((endMat.M00 - startMat.M00) * amount);
	M01 = startMat.M01 + ((endMat.M01 - startMat.M01) * amount);
	M02 = startMat.M02 + ((endMat.M02 - startMat.M02) * amount);
	M03 = startMat.M03 + ((endMat.M03 - startMat.M03) * amount);
	M10 = startMat.M10 + ((endMat.M10 - startMat.M10) * amount);
	M11 = startMat.M11 + ((endMat.M11 - startMat.M11) * amount);
	M12 = startMat.M12 + ((endMat.M12 - startMat.M12) * amount);
	M13 = startMat.M13 + ((endMat.M13 - startMat.M13) * amount);
	M20 = startMat.M20 + ((endMat.M20 - startMat.M20) * amount);
	M21 = startMat.M21 + ((endMat.M21 - startMat.M21) * amount);
	M22 = startMat.M22 + ((endMat.M22 - startMat.M22) * amount);
	M23 = startMat.M23 + ((endMat.M23 - startMat.M23) * amount);
	M30 = startMat.M30 + ((endMat.M30 - startMat.M30) * amount);
	M31 = startMat.M31 + ((endMat.M31 - startMat.M31) * amount);
	M32 = startMat.M32 + ((endMat.M32 - startMat.M32) * amount);
	M33 = startMat.M33 + ((endMat.M33 - startMat.M33) * amount);
}


#define swap(a,b) t = a; a = b; b = t;
void Matrix4::transpose()
{
	double t;
	swap(M10, M01);
	swap(M20, M02);
	swap(M30, M03);
	swap(M12, M21);
	swap(M13, M31);
	swap(M23, M32);
}
}
