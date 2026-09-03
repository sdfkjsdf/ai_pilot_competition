#pragma once

#include <sstream>
#include <cstdlib>
#include <limits>
#include <iomanip>

#include "Math.h"
#include "Vector3.h"

namespace BT_Geometry
{

class Vector3;
class Quaternion;
class EulerAngle;
class AxisAngle;
class Matrix4;

/// <summary>
/// double precision Matrix3 class
///
/// Column Major M(column,row)
/// M00  M10  M20 
/// M01  M11  M21 
/// M02  M12  M22 
/// X Axis: M00, M01, M02
/// Y Axis: M10, M11, M12
/// Z Axis: M20, M21, M22
///
/// </summary>
class Matrix3
{

public:		// Constructors & Destructor

	Matrix3 (void);
	explicit Matrix3 (Quaternion const& v);
	explicit Matrix3 (AxisAngle const& v);
	explicit Matrix3 (EulerAngle const& v);
	Matrix3	(	double m00, double m01, double m02, 
				double m10, double m11, double m12, 
				double m20, double m21, double m22);
	Matrix3	(Matrix3 const& v);
	~Matrix3	(void);

public:		// Methods
		
	/// <summary>
	/// ��Ʈ������ ���� ������Ų��.
	/// </summary>
	/// <param name="startMat">���� ��Ʈ����</param>
	/// <param name="endMat">�� ��Ʈ����</param>
	/// <param name="amount">���� factor(0 ~ 1����)</param>
	void	lerp				(Matrix3 const& startMat, Matrix3 const& endMat, double amount);
	
	/// <summary>
	/// ������� ���Ѵ�.
	/// ��ü�� ���¸� ��ȭ��Ű�� �ʴ´�.
	/// </summary>
	/// <returns>����� ��Ʈ����</returns>
	Matrix3	inverse(void) const;

	/// <summary>
	/// ������� ���Ѵ�.
	/// ��ü�� ���¸� ��ȭ��Ű�� �ʴ´�.
	/// </summary>
	/// <param name="target">������� ���� ��� ���</param>
	/// <returns></returns>
	void inverse(Matrix3* target) const;


	/// <summary>
	/// rotation Matrix3�� axis angle�� ��ȯ�Ѵ�.
	/// </summary>
	/// <returns>axis angle</returns>
	AxisAngle	toAxisAngle(void) const;

	void		toAxisAngle(AxisAngle* dest) const;
	/// <summary>
	/// rotation Matrix3�� eular angle�� ��ȯ�Ѵ�.
	/// </summary>
	/// <returns>eular angle</returns>
	EulerAngle	toEulerAngle(void) const;

	void		toEulerAngle(EulerAngle* dest) const;

	/// <summary>
	/// rotation Matrix3�� quaternion�� ��ȯ�Ѵ�.
	/// </summary>
	/// <returns>quaternion</returns>
	Quaternion	toQuaternion	(void) const;

	/// <summary>
	/// rotation Matrix3�� quaternion�� ��ȯ�Ѵ�.
	/// </summary>
	/// <param name = "quaternion">quaternion</param>
	void	toQuaternion	(Quaternion* dest) const;

	/// <summary>
	/// �ڽ��� ��� ���� ��ȯ�Ѵ�.
	/// </summary>
	void	transpose			(void);

	/// <summary>
	/// X���� ���� ���͸� �����Ѵ�.
	/// </summary>
	/// <param name="axisVector">������ ��������</param>	
	void	setXAxis		(Vector3 const& axisVector);

	/// <summary>
	/// Y���� ���� ���͸� �����Ѵ�.
	/// </summary>
	/// <param name="axisVector">������ ��������</param>	
	void	setYAxis		(Vector3 const& axisVector);
	/// <summary>
	/// Z���� ���� ���͸� �����Ѵ�.
	/// </summary>
	/// <param name="axisVector">������ ��������</param>	
	void	setZAxis		(Vector3 const& axisVector);

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
	/// <summary>
	/// �� ����� ���Ѵ�.
	/// this = this * rhs
	/// </summary>
	/// <param name="rhs">���� ��� ���</param>
	/// <returns>��ȯ�� ���</returns>
	Matrix3&			operator*=	(Matrix3 const& rhs);

	/// <summary>
	/// �� ����� ���ϴ� operator
	/// ret = this * rhs
	/// </summary>
	/// <param name="rhs"></param>
	/// <returns></returns>
	Matrix3			operator*	(Matrix3 const& rhs) const;
	Vector3			operator*	(Vector3 const& rhs) const;
	
	/// <summary>
	/// ��Ŀ� scalar���� ���Ѵ�.
	/// </summary>
	/// <param name="rhs"></param>
	/// <returns></returns>
	Matrix3			operator*	(double rhs) const;
	
	/// <summary>
	/// ��� ���� ������
	/// </summary>
	/// <param name="rhs">������ ���</param>
	/// <returns></returns>
	Matrix3&	operator=	(Matrix3 const& rhs);
	
	/// <summary>
	/// ���ϴ� row, column�� �����Ѵ�.
	/// </summary>
	/// <param name="row">��</param>
	/// <param name="col">��</param>
	/// <returns>�ش� ��,���� �ִ� ��</returns>	
	double&			operator()	(unsigned int row, unsigned int col);
	/// <summary>
	/// ���ϴ� row, column�� �����Ѵ�.
	/// </summary>
	/// <param name="row">��</param>
	/// <param name="col">��</param>
	/// <returns>�ش� ��,���� �ִ� ��</returns>	
	double const&	operator()	(unsigned int row, unsigned int col) const;
	/// <summary>
	/// �ش� index�� ���� �����´�
	/// </summary>
	/// <param name="index">��ġ</param>
	/// <returns>�ش� ��ġ�� �ش��ϴ� ��</returns>
	double&			operator[]	(unsigned int index);
	/// <summary>
	/// 
	/// </summary>
	/// <param name="index"></param>
	/// <returns></returns>
	double const&	operator[]	(unsigned int index) const;

	template<class E, class U>
	friend std::basic_ostream<E, U>& operator<< (std::basic_ostream<E, U>& os, Matrix3 const& p)
	{
		os << "(" << std::setprecision(std::numeric_limits<double>::digits10) << p.M00 << "," << std::setprecision(std::numeric_limits<double>::digits10) << p.M01 << "," << std::setprecision(std::numeric_limits<double>::digits10) << p.M02 << ","
		          << std::setprecision(std::numeric_limits<double>::digits10) << p.M10 << "," << std::setprecision(std::numeric_limits<double>::digits10) << p.M11 << "," << std::setprecision(std::numeric_limits<double>::digits10) << p.M12 << ","
		          << std::setprecision(std::numeric_limits<double>::digits10) << p.M20 << "," << std::setprecision(std::numeric_limits<double>::digits10) << p.M21 << "," << std::setprecision(std::numeric_limits<double>::digits10) << p.M22 << ")";

		return os;
	}

public:		// Getters & Setters
	/// <summary>
	/// hash code�� �����´�.
	/// </summary>
	/// <returns>M00 * M11 * M22 * M33</returns>
	int getHashCode() const
	{
		return (int)(M00 * M11 * M22);
	}

public:
	static Matrix3 const& Identity();

public:		// Member Variables
	double M00; //0
	double M01; //1
	double M02; //2
	double M10; //3
	double M11; //4
	double M12; //5
	double M20; //6
	double M21; //7
	double M22; //8



public:
	static Matrix3 _Identity;
};


///////////////////////////////////////////////////////////////////////////////////////
// Matrix3's inline functions

//////////////////////////////////////////////
// Constructors & Destructors
inline Matrix3::Matrix3()
					:	M00(1.0), M01(0.0), M02(0.0),
						M10(0.0), M11(1.0), M12(0.0),
						M20(0.0), M21(0.0), M22(1.0)
{
}
inline Matrix3::Matrix3(double m00, double m01, double m02 ,
					  double m10, double m11, double m12,
					  double m20, double m21, double m22)
					:	M00(m00), M01(m01), M02(m02),
						M10(m10), M11(m11), M12(m12),
						M20(m20), M21(m21), M22(m22)
						
{
}

inline Matrix3::Matrix3(Matrix3 const& rhs)
					:	M00(rhs.M00), M01(rhs.M01), M02(rhs.M02),
						M10(rhs.M10), M11(rhs.M11), M12(rhs.M12),
						M20(rhs.M20), M21(rhs.M21), M22(rhs.M22)
{

}

inline Matrix3::~Matrix3(void) {}
//////////////////////////////////////////////

//////////////////////////////////////////////

//////////////////////////////////////////////
// Operations


inline bool operator!=(Matrix3 const& a, Matrix3 const& b) 
{
	return (!Equals(a.M00, b.M00) || !Equals(a.M01, b.M01) || !Equals(a.M02, b.M02) || 
			!Equals(a.M10, b.M10) || !Equals(a.M11, b.M11) || !Equals(a.M12, b.M12) || 
			!Equals(a.M20, b.M20) || !Equals(a.M21, b.M21) || !Equals(a.M22, b.M22));
}
inline bool operator==(Matrix3 const& a, Matrix3 const& b)
{
	return !operator!=(a,b);
}

inline double& Matrix3::operator() (unsigned int row, unsigned int col)
{
	return *(&M00 + (row * 3 + col));
}
inline double const& Matrix3::operator()	(unsigned int row, unsigned int col) const
{
	return *(&M00 + (row * 3 + col));
}
inline double& Matrix3::operator[] (unsigned int index)
{
	return *(&M00 + index);
}
inline double const& Matrix3::operator[]	(unsigned int index) const
{
	return *(&M00 + index);
}
inline Matrix3& Matrix3::operator*=(Matrix3 const& rhs)
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


inline Matrix3 Matrix3::operator*(Matrix3 const& rhs) const
{
	return Matrix3 (this->M00*rhs.M00 + this->M10*rhs.M01 + this->M20*rhs.M02,
					this->M01*rhs.M00 + this->M11*rhs.M01 + this->M21*rhs.M02,
					this->M02*rhs.M00 + this->M12*rhs.M01 + this->M22*rhs.M02,
					
		
					this->M00*rhs.M10 + this->M10*rhs.M11 + this->M20*rhs.M12,
					this->M01*rhs.M10 + this->M11*rhs.M11 + this->M21*rhs.M12,
					this->M02*rhs.M10 + this->M12*rhs.M11 + this->M22*rhs.M12,
									
					this->M00*rhs.M20 + this->M10*rhs.M21 + this->M20*rhs.M22,
					this->M01*rhs.M20 + this->M11*rhs.M21 + this->M21*rhs.M22,
					this->M02*rhs.M20 + this->M12*rhs.M21 + this->M22*rhs.M22);

}

inline Matrix3 Matrix3::operator*(double rhs) const
{
	return Matrix3(	this->M00 * rhs,
					this->M01 * rhs,
					this->M02 * rhs,

					this->M10 * rhs,
					this->M11 * rhs,
					this->M12 * rhs,

					this->M20 * rhs,
					this->M21 * rhs,
					this->M22 * rhs);
}

inline Matrix3& Matrix3::operator=(Matrix3 const& rhs)
{
	this->M00 = rhs.M00; this->M01 = rhs.M01; this->M02 = rhs.M02;
	this->M10 = rhs.M10; this->M11 = rhs.M11; this->M12 = rhs.M12;
	this->M20 = rhs.M20; this->M21 = rhs.M21; this->M22 = rhs.M22;
	
	return *this;
}



//////////////////////////////////////////////
// Getters & Setters

inline void	Matrix3::setXAxis(Vector3 const& axisVector)
{
	M00 = axisVector.X;
	M01 = axisVector.Y;
	M02 = axisVector.Z;
}
inline void	Matrix3::setYAxis(Vector3 const& axisVector)
{
	M10 = axisVector.X;
	M11 = axisVector.Y;
	M12 = axisVector.Z;
}
inline void	Matrix3::setZAxis(Vector3 const& axisVector)
{
	M20 = axisVector.X;
	M21 = axisVector.Y;
	M22 = axisVector.Z;
}

inline Vector3 Matrix3::getXAxis		(void) const
{
	return Vector3(M00, M01, M02);
}

inline Vector3 Matrix3::getYAxis		(void) const
{
	return Vector3(M10, M11, M12);
}

inline Vector3 Matrix3::getZAxis		(void) const
{
	return Vector3(M20, M21, M22);
}

inline Vector3 Matrix3::operator*	(Vector3 const& rhs) const
{
	return Vector3(	M00 * rhs.X + M10 * rhs.Y + M20 * rhs.Z,
					M01 * rhs.X + M11 * rhs.Y + M21 * rhs.Z,
					M02 * rhs.X + M12 * rhs.Y + M22 * rhs.Z);
}

inline Matrix3 const& Matrix3::Identity()
{
	return _Identity;
}
}