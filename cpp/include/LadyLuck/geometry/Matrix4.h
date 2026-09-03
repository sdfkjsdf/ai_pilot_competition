#pragma once

#include "Math.h"
#include "Vector3.h"
#include "Vector4.h"
#include "Matrix3.h"

#include <sstream>
#include <cstdlib>
#include <limits>
#include <iomanip>

namespace BT_Geometry
{
class Matrix3;

/// <summary>
/// double precision column major Matrix4 class
/// 
/// Column Major M(column,row)
/// M00  M10  M20  M30     M00  M10  M20  M30 
/// M01  M11  M21  M31	 M01  M11  M21  M31
/// M02  M12  M22  M32	 M02  M12  M22  M32
/// M03  M13  M23  M33	 M03  M13  M23  M33 
/// X Axis: M00, M01, M02, M03
/// Y Axis: M10, M11, M12, M13
/// Z Axis: M20, M21, M22, M23
/// Position: M30, M31, M32, M33
///
/// Matrix a, b, c ������ ����� ���Ҷ� c * b * a
/// </summary>
class Matrix4
{
public:		// Constructors & Destructor
	/// <summary>
	/// ������
	/// </summary>
	Matrix4(void);
	/// <summary>
	/// row major���� ������
	/// </summary>
	Matrix4(	double m00, double m01, double m02, double m03, 
				double m10, double m11, double m12, double m13, 
				double m20, double m21, double m22, double m23, 
				double m30, double m31, double m32, double m33);

	Matrix4(Matrix3 const& m, Vector4 const& v);
	Matrix4(Matrix3 const& m, Vector3 const& tran);
	/// <summary>
	/// ������
	/// </summary>
	Matrix4(Matrix4 const& rhs);
	/// <summary>
	/// �Ҹ���
	/// </summary>
	~Matrix4(void);

public:		// Methods
	
	/// <summary>
	/// ��Ʈ������ ���� ������Ų��.
	/// </summary>
	/// <param name="startMat">���� ��Ʈ����</param>
	/// <param name="endMat">�� ��Ʈ����</param>
	/// <param name="amount">���� factor(0 ~ 1����)</param>
	void	lerp				(Matrix4 const& startMat, Matrix4 const& endMat, double amount);

	/// <summary>
	/// ������� ���Ѵ�.
	/// ��ü�� ���¸� ��ȭ��Ű�� �ʴ´�.
	/// </summary>
	/// <returns>����� ��Ʈ����</returns>
	Matrix4	inverse				(void) const;

	/// <summary>
	/// ������� ���Ѵ�.
	/// ��ü�� ���¸� ��ȭ��Ű�� �ʴ´�.
	/// </summary>
	/// <param name="target">�����</param>
	/// <returns></returns>
	void inverse		(Matrix4* dest) const;
	
	/// <summary>
	/// ����� determinant�� ���Ѵ�.
	/// </summary>
	/// <returns>determinant��</returns>
	double  determinant			(void) const;
	
	/// <summary>
	/// �ڽ��� ��� ���� ��ȯ�Ѵ�.
	/// </summary>
	void	transpose			(void);


	/// <summary>
	/// X���� ���� ���͸� �����Ѵ�.
	/// </summary>
	/// <param name="axisVector">������ ��������</param>	
	void setXAxis(Vector3 const& xAxis);

	/// <summary>
	/// Y���� ���� ���͸� �����Ѵ�.
	/// </summary>
	/// <param name="axisVector">������ ��������</param>	
	void setYAxis(Vector3 const& yAxis);
	/// <summary>
	/// Z���� ���� ���͸� �����Ѵ�.
	/// </summary>
	/// <param name="axisVector">������ ��������</param>	
	void setZAxis(Vector3 const& zAxis);

	void setTranslation(Vector3 const& translation);
	void setScale(Vector3 const& scale);
	void setRotation(Matrix3 const& rotation);

	void getRotation(Matrix3* matrix) const;
	Matrix3 getRotation(void) const;

	Vector3 getTranslation(void) const;
	Vector3 getScale(void) const;

	/// <summary>
	/// X���� ���� ���͸� ���´�.
	/// </summary>
	/// <returns>��������</returns>
	Vector3 getXAxis		(void) const;
	/// <summary>
	/// Y���� ���� ���͸� ���´�.
	/// </summary>
	/// <returns>��������</returns>
	Vector3 getYAxis		(void) const;
	/// <summary>
	/// Z���� ���� ���͸� ���´�.
	/// </summary>
	/// <returns>��������</returns>
	Vector3 getZAxis		(void) const;

public:		// Operators
	
	Matrix4&		operator*=	(Matrix4 const& rhs);
	Matrix4			operator*	(Matrix4 const& rhs) const;
	Vector4			operator*	(Vector4 const& rhs) const;
	Vector3			operator*	(Vector3 const& rhs) const;
	Matrix4			operator*	(double rhs) const;
	Matrix4&	operator=	(Matrix4 const& rhs);

	double&			operator()	(unsigned int row, unsigned int col);
	double const&	operator()	(unsigned int row, unsigned int col) const;
	double&			operator[]	(unsigned int index);
	double const&	operator[]	(unsigned int index) const;

	template<class E, class U>
	friend std::basic_ostream<E, U>& operator<< (std::basic_ostream<E, U>& os, Matrix4 const& p)
	{
		os << "(" << std::setprecision(std::numeric_limits<double>::digits10) << p.M00 << "," << std::setprecision(std::numeric_limits<double>::digits10) << p.M01 << "," << std::setprecision(std::numeric_limits<double>::digits10) << p.M02 << "," << std::setprecision(std::numeric_limits<double>::digits10) << p.M03 << ","   
				  << std::setprecision(std::numeric_limits<double>::digits10) << p.M10 << "," << std::setprecision(std::numeric_limits<double>::digits10) << p.M11 << "," << std::setprecision(std::numeric_limits<double>::digits10) << p.M12 << "," << std::setprecision(std::numeric_limits<double>::digits10) << p.M13 << ","   
				  << std::setprecision(std::numeric_limits<double>::digits10) << p.M20 << "," << std::setprecision(std::numeric_limits<double>::digits10) << p.M21 << "," << std::setprecision(std::numeric_limits<double>::digits10) << p.M22 << "," << std::setprecision(std::numeric_limits<double>::digits10) << p.M23 << ","   
				  << std::setprecision(std::numeric_limits<double>::digits10) << p.M30 << "," << std::setprecision(std::numeric_limits<double>::digits10) << p.M31 << "," << std::setprecision(std::numeric_limits<double>::digits10) << p.M32 << "," << std::setprecision(std::numeric_limits<double>::digits10) << p.M33 << ")";
		return os;
	}

public:		// Getters & Setters
	/// <summary>
	/// hash code�� �����´�.
	/// </summary>
	/// <returns>M00 * M11 * M22 * M33</returns>
	int getHashCode() const
	{
		return (int)(M00 * M11 * M22 * M33);
	}

public:
	static Matrix4 const& Identity();

public:		// Member Variables
	double M00; //col - 0, row - 0
	double M01; //col - 0, row - 1
	double M02; //col - 0, row - 2
	double M03; //col - 0, row - 3
	double M10; //col - 1, row - 0
	double M11; //col - 1, row - 1
	double M12; //col - 1, row - 2
	double M13; //col - 1, row - 3
	double M20; //col - 2, row - 0
	double M21; //col - 2, row - 1
	double M22; //col - 2, row - 2
	double M23; //col - 2, row - 3
	double M30; //col - 3, row - 0
	double M31; //col - 3, row - 1
	double M32; //col - 3, row - 2
	double M33; //col - 3, row - 3

private:
	static Matrix4 _Identity;
};


///////////////////////////////////////////////////////////////////////////////////////
// Matrix4's inline functions

//////////////////////////////////////////////
// Constructors & Destructors
inline Matrix4::Matrix4()
					:	M00(1.0), M01(0.0), M02(0.0), M03(0.0), 
						M10(0.0), M11(1.0), M12(0.0), M13(0.0), 
						M20(0.0), M21(0.0), M22(1.0), M23(0.0), 
						M30(0.0), M31(0.0), M32(0.0), M33(1.0)
{
}
inline Matrix4::Matrix4(double m00, double m01, double m02, double m03, 
					  double m10, double m11, double m12, double m13, 
					  double m20, double m21, double m22, double m23, 
					  double m30, double m31, double m32, double m33)
					:	M00(m00), M01(m01), M02(m02), M03(m03), 
						M10(m10), M11(m11), M12(m12), M13(m13), 
						M20(m20), M21(m21), M22(m22), M23(m23), 
						M30(m30), M31(m31), M32(m32), M33(m33)
{
}

inline Matrix4::Matrix4(Matrix4 const& rhs)
					:	M00(rhs.M00), M01(rhs.M01), M02(rhs.M02), M03(rhs.M03), 
						M10(rhs.M10), M11(rhs.M11), M12(rhs.M12), M13(rhs.M13), 
						M20(rhs.M20), M21(rhs.M21), M22(rhs.M22), M23(rhs.M23), 
						M30(rhs.M30), M31(rhs.M31), M32(rhs.M32), M33(rhs.M33)
{

}

inline Matrix4::Matrix4(Matrix3 const& m, Vector4 const& v)
: M00(m.M00), M01(m.M01), M02(m.M02), M03(0), 
  M10(m.M10), M11(m.M11), M12(m.M12), M13(0),
  M20(m.M20), M21(m.M21), M22(m.M22), M23(0),
  M30(v.X), M31(v.Y), M32(v.Z), M33(v.W)
{
}

inline Matrix4::Matrix4(Matrix3 const& m, Vector3 const& tran)
: M00(m.M00), M01(m.M01), M02(m.M02), M03(0), 
  M10(m.M10), M11(m.M11), M12(m.M12), M13(0),
  M20(m.M20), M21(m.M21), M22(m.M22), M23(0),
  M30(tran.X), M31(tran.Y), M32(tran.Z), M33(1)
{
}

inline Matrix4::~Matrix4(void) {}
//////////////////////////////////////////////

//////////////////////////////////////////////

//////////////////////////////////////////////
// Operations


inline bool operator!=(Matrix4 const& lhs, Matrix4 const& rhs) 
{
	return (!Equals(lhs.M00, rhs.M00) || !Equals(lhs.M01, rhs.M01) || !Equals(lhs.M02, rhs.M02) || !Equals(lhs.M03, rhs.M03) ||
			!Equals(lhs.M10, rhs.M10) || !Equals(lhs.M11, rhs.M11) || !Equals(lhs.M12, rhs.M12) || !Equals(lhs.M13, rhs.M13) ||
			!Equals(lhs.M20, rhs.M20) || !Equals(lhs.M21, rhs.M21) || !Equals(lhs.M22, rhs.M22) || !Equals(lhs.M23, rhs.M23) ||
			!Equals(lhs.M30, rhs.M30) || !Equals(lhs.M31, rhs.M31) || !Equals(lhs.M32, rhs.M32) || !Equals(lhs.M33, rhs.M33));
}
inline bool operator==(Matrix4 const& lhs, Matrix4 const& rhs)
{
	return !operator!=(lhs,rhs);
}

inline double& Matrix4::operator() (unsigned int row, unsigned int col)
{
	return *(&M00 + (row * 4 + col));
}
inline double const& Matrix4::operator()	(unsigned int row, unsigned int col) const
{
	return *(&M00 + (row * 4 + col));
}
inline double& Matrix4::operator[] (unsigned int index)
{
	return *(&M00 + index);
}
inline double const& Matrix4::operator[]	(unsigned int index) const
{
	return *(&M00 + index);
}

inline Matrix4& Matrix4::operator*=(Matrix4 const& rhs)
{
	if ( *this == Identity() )
	{
		*this = rhs;
		return *this;
	}
	else if ( rhs == Identity() )
	{
		return *this;
	}
	
	*this = (*this) * (rhs) ;

	return *this;
}

inline Matrix4 Matrix4::operator*(Matrix4 const& rhs) const
{
 //M00  M10  M20  M30		M00  M10  M20  M30
 //M01  M11  M21  M31	    M01  M11  M21  M31
 //M02  M12  M22  M32	    M02  M12  M22  M32
 //M03  M13  M23  M33	    M03  M13  M23  M33 

	
	return Matrix4 (this->M00*rhs.M00 + this->M10*rhs.M01 + this->M20*rhs.M02 + this->M30*rhs.M03,
					this->M01*rhs.M00 + this->M11*rhs.M01 + this->M21*rhs.M02 + this->M31*rhs.M03,
					this->M02*rhs.M00 + this->M12*rhs.M01 + this->M22*rhs.M02 + this->M32*rhs.M03,
					this->M03*rhs.M00 + this->M13*rhs.M01 + this->M23*rhs.M02 + this->M33*rhs.M03,

					this->M00*rhs.M10 + this->M10*rhs.M11 + this->M20*rhs.M12 + this->M30*rhs.M13,
					this->M01*rhs.M10 + this->M11*rhs.M11 + this->M21*rhs.M12 + this->M31*rhs.M13,
					this->M02*rhs.M10 + this->M12*rhs.M11 + this->M22*rhs.M12 + this->M32*rhs.M13,
					this->M03*rhs.M10 + this->M13*rhs.M11 + this->M23*rhs.M12 + this->M33*rhs.M13,

					this->M00*rhs.M20 + this->M10*rhs.M21 + this->M20*rhs.M22 + this->M30*rhs.M23,
					this->M01*rhs.M20 + this->M11*rhs.M21 + this->M21*rhs.M22 + this->M31*rhs.M23,
					this->M02*rhs.M20 + this->M12*rhs.M21 + this->M22*rhs.M22 + this->M32*rhs.M23,
					this->M03*rhs.M20 + this->M13*rhs.M21 + this->M23*rhs.M22 + this->M33*rhs.M23,

					this->M00*rhs.M30 + this->M10*rhs.M31 + this->M20*rhs.M32 + this->M30*rhs.M33,
					this->M01*rhs.M30 + this->M11*rhs.M31 + this->M21*rhs.M32 + this->M31*rhs.M33,
					this->M02*rhs.M30 + this->M12*rhs.M31 + this->M22*rhs.M32 + this->M32*rhs.M33,
					this->M03*rhs.M30 + this->M13*rhs.M31 + this->M23*rhs.M32 + this->M33*rhs.M33);
}


  

inline Matrix4 Matrix4::operator*(double rhs) const
{
	return Matrix4(	this->M00 * rhs,
					this->M01 * rhs,
					this->M02 * rhs,
					this->M03 * rhs,

					this->M10 * rhs,
					this->M11 * rhs,
					this->M12 * rhs,
					this->M13 * rhs,

					this->M20 * rhs,
					this->M21 * rhs,
					this->M22 * rhs,
					this->M23 * rhs,

					this->M30 * rhs,
					this->M31 * rhs,
					this->M32 * rhs,
					this->M33 * rhs);

}

inline Matrix4& Matrix4::operator=(Matrix4 const& rhs)
{
	this->M00 = rhs.M00; this->M01 = rhs.M01; this->M02 = rhs.M02; this->M03 = rhs.M03;
	this->M10 = rhs.M10; this->M11 = rhs.M11; this->M12 = rhs.M12; this->M13 = rhs.M13;
	this->M20 = rhs.M20; this->M21 = rhs.M21; this->M22 = rhs.M22; this->M23 = rhs.M23;
	this->M30 = rhs.M30; this->M31 = rhs.M31; this->M32 = rhs.M32; this->M33 = rhs.M33;
	
	return *this;
}

inline Vector4	Matrix4::operator* (Vector4 const& rhs) const
{
	return Vector4 (M00 * rhs.X + M10 * rhs.Y + M20 * rhs.Z + M30 * rhs.W,
					M01 * rhs.X + M11 * rhs.Y + M21 * rhs.Z + M31 * rhs.W,
					M02 * rhs.X + M12 * rhs.Y + M22 * rhs.Z + M32 * rhs.W,
					M03 * rhs.X + M13 * rhs.Y + M23 * rhs.Z + M33 * rhs.W);
}

inline Vector3	Matrix4::operator* (Vector3 const& rhs) const
{
    Vector3 r;

    r.X = (M00 * rhs.X + M10 * rhs.Y + M20 * rhs.Z + M30);
    r.Y = (M01 * rhs.X + M11 * rhs.Y + M21 * rhs.Z + M31);
    r.Z = (M02 * rhs.X + M12 * rhs.Y + M22 * rhs.Z + M32);

    return r;
}



//////////////////////////////////////////////

//////////////////////////////////////////////
// Getters & Setters
inline void	Matrix4::setXAxis(Vector3 const& xAxis)
{
	M00 = xAxis.X;
	M01 = xAxis.Y;
	M02 = xAxis.Z;
}
inline void	Matrix4::setYAxis(Vector3 const& yAxis)
{
	M10 = yAxis.X;
	M11 = yAxis.Y;
	M12 = yAxis.Z;
}
inline void	Matrix4::setZAxis(Vector3 const& zAxis)
{
	M20 = zAxis.X;
	M21 = zAxis.Y;
	M22 = zAxis.Z;
}

inline Vector3 Matrix4::getXAxis		(void) const
{
	return Vector3(M00, M01, M02);
}

inline Vector3 Matrix4::getYAxis		(void) const
{
	return Vector3(M10, M11, M12);
}

inline Vector3 Matrix4::getZAxis		(void) const
{
	return Vector3(M20, M21, M22);
}

inline void Matrix4::setTranslation(Vector3 const& translation)
{
	M30 = translation.X;
	M31 = translation.Y;
	M32 = translation.Z;
	M33 = 1.0;
}

inline void Matrix4::setScale(Vector3 const& scale)
{
	M00 = scale.X;
	M11 = scale.Y;
	M22 = scale.Z;
}

inline void Matrix4::setRotation(Matrix3 const& rotation)
{
	this->M00 = rotation.M00; this->M01 = rotation.M01; this->M02 = rotation.M02;
	this->M10 = rotation.M10; this->M11 = rotation.M11; this->M12 = rotation.M12;
	this->M20 = rotation.M20; this->M21 = rotation.M21; this->M22 = rotation.M22;
}

inline void Matrix4::getRotation(Matrix3* dest) const
{
	dest->M00 = M00; dest->M01 = M01; dest->M02 = M02; 
	dest->M10 = M10; dest->M11 = M11; dest->M12 = M12; 
	dest->M20 = M20; dest->M21 = M21; dest->M22 = M22;
}

inline Matrix3 Matrix4::getRotation(void) const
{
	Matrix3 result;
	getRotation(&result);
	return result;
}

inline Vector3 Matrix4::getTranslation(void) const
{
	return Vector3(M30, M31, M32);
}

inline Vector3 Matrix4::getScale(void) const
{
	if (IsZero(M01)&&IsZero(M02)&&IsZero(M10)&&IsZero(M12)&&IsZero(M20)&&IsZero(M21))
	{
		return Vector3(M00, M11, M22);
	}
	else
	{
		return Vector3(sqrt(M00 * M00 + M01 * M01 + M02 * M02),
			sqrt(M10 * M10 + M11 * M11 + M12 * M12),
			sqrt(M20 * M20 + M21 * M21 + M22 * M22));
	}
}

inline Matrix4 const& Matrix4::Identity()
{
	return _Identity;
}

//////////////////////////////////////////////
}