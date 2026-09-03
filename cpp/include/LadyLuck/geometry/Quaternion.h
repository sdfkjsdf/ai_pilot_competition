#pragma once
#include <iostream>
#include <cmath>
#include "Math.h"
#include "Vector3.h"

#include <sstream>
#include <limits>
#include <iomanip>



namespace BT_Geometry
{

class Matrix3;
class AxisAngle;
class EulerAngle;

/// <summary>
/// double���е��� ���� ���ʹϾ�
/// </summary>
class Quaternion
{
private:
	
public:		// Constructors & Destructor
	Quaternion(void);
	Quaternion(double x, double y, double z, double w );
	Quaternion(Quaternion const& q);
	explicit Quaternion(AxisAngle const& a);
	explicit Quaternion(EulerAngle const& e);
	explicit Quaternion(Matrix3 const& m);


public:		// Methods
	/// <summary>
	/// ���ʹϾ��� ���̸� ���Ѵ�.
	/// </summary>
	/// <returns>���ʹϾ� ����</returns>
	double length(void) const;
	
	/// <summary>
	/// ���ʹϾ��� ������ ������ ���Ѵ�. sqrt������ ���ϱ� ������ length()���� �� ������.
	/// </summary>
	/// <returns>���ʹϾ� ����</returns>	
	double lengthSquared(void) const;
	
	/// <summary>
	/// ���ʹϾ��� ����ȭ ��Ų��.
	/// </summary>
	void normalize(void);
	/// <summary>
	/// ���ʹϾ��� xyz���п� -1�� ���Ѵ�.
	/// </summary>
	void conjugate(void);

	/// <summary>
	/// ���� ���ʹϾ��� �ݴ밪�� ���Ѵ�.
	/// </summary>
	Quaternion inverse(void) const;
	
	/// <summary>
	/// ���� ���ʹϾ��� �ݴ밪�� ���Ѵ�.
	/// </summary>
	void inverse(Quaternion* dest) const;
	
	/// <summary>
	/// �������� ����� �����Ѵ�.
	/// </summary>
	/// <param name="start">�������� ���� ���ʹϾ�</param>
	/// <param name="end">���� ���� ������ ���ʹϾ�</param>
	/// <param name="amount">0 ~ 1������ ��, 0.0 : begin, 1.0 : end</param>
	void lerp(Quaternion const& start, Quaternion const& end, double amount);

	/// <summary>
	/// ���� �������� ����� �����Ѵ�.
	/// </summary>
	/// <param name="startPoint">�������� ���� ���ʹϾ�</param>
	/// <param name="endPoint">���� ���� ������ ���ʹϾ�</param>
	/// <param name="factor">0 ~ 1������ ��, 0.0 : begin, 1.0 : end</param>
	void sLerp(Quaternion const & startPoint, Quaternion const& endPoint, double factor, bool clampToMinorAngle = true);

	/// <summary>
	/// ���ʹϾ��� ������ ���Ѵ�.
	/// </summary>
	/// <param name="rhs">������ �ʿ��� ���ʹϾ�</param>
	/// <returns>������ ��Į�� ��</returns>
	double dot(Quaternion const& rhs) const;
	
	
	/// <summary>
	/// ���ʹϾ��� ��Ʈ������ ��ȯ�Ѵ�.
	/// </summary>
	/// <returns>���ʹϾ� �����Ǵ� ��Ʈ����</returns>
	Matrix3		toMatrix		(void) const;

	/// <summary>
	/// ���ʹϾ��� ��Ʈ������ ��ȯ�Ѵ�.
	/// </summary>
	/// <param name="rotationMat">���ʹϾ� �����Ǵ� ��Ʈ����</param>
	void toMatrix		(Matrix3* dest) const;

	/// <summary>
	/// ���ʹϾ��� �ֽĽ��ޱ۷� ��ȯ�Ѵ�.
	/// </summary>
	/// <returns>���ʹϾ� �����Ǵ� �ֽĽ��ޱ�</returns>
	AxisAngle toAxisAngle(void) const;

	/// <summary>
	/// ���ʹϾ��� �ֽĽ��ޱ۷� ��ȯ�Ѵ�.
	/// </summary>
	/// <param name="axisAngle">���ʹϾ� �����Ǵ� �ֽĽ��ޱ�</param>
	void toAxisAngle(AxisAngle* dest) const;

	/// <summary>
	/// ���ʹϾ��� ���Ϸ��ޱ۷� ��ȯ�Ѵ�.
	/// </summary>
	/// <returns>���ʹϾ� �����Ǵ� ���Ϸ��ޱ�</returns>
	EulerAngle toEulerAngle(void) const;

	/// <summary>
	/// ���ʹϾ��� ���Ϸ��ޱ۷� ��ȯ�Ѵ�.
	/// </summary>
	/// <param name="axisAngle">���ʹϾ� �����Ǵ� ���Ϸ��ޱ�</param>
	void toEulerAngle(EulerAngle* dest) const;

public:
	Quaternion operator+(Quaternion const& Vector4Single) const; 
    Quaternion operator-(Quaternion const& rhs) const;
    Quaternion operator*(Quaternion const& rhs) const;
	Vector3 operator*(Vector3 const& rhs) const;
    Quaternion operator/(Quaternion const& rhs) const; 
    Quaternion operator-() const;

    Quaternion& operator+=(Quaternion const& rhs); 
    Quaternion& operator-=(Quaternion const& rhs); 
    Quaternion& operator*=(Quaternion const& rhs); 
    Quaternion& operator/=(Quaternion const& rhs); 
	
	
	Quaternion& operator=(Quaternion const& rhs);

public:
	template<class E, class U>
	friend std::basic_ostream<E, U>& operator<< (std::basic_ostream<E, U>& os, Quaternion const& rhs)
	{
		os << L"(" << std::setprecision(std::numeric_limits<double>::digits10) << rhs.X << L"," 
			<< std::setprecision(std::numeric_limits<double>::digits10) << rhs.Y << L"," 
			<< std::setprecision(std::numeric_limits<double>::digits10) << rhs.Z << L"," 
			<< std::setprecision(std::numeric_limits<double>::digits10) << rhs.W << L")";
		return os;
	}

public:		// Getters & Setters
	int getHashCode() const
	{
		return (int)(X * Y * Z * W * 1000000);
	}

public:
	static Quaternion const& Identity();

public:		// Member Variables
	double X;
	double Y;
	double Z;
	double W;

private:		// Static Member Variables
	static Quaternion _Identity;
};

inline Quaternion::Quaternion(void) : X(0.0), Y(0.0), Z(0.0), W(1.0)
{
}

inline Quaternion::Quaternion(double x, double y, double z, double w ) : X(x), Y(y), Z(z), W(w)
{
}

inline Quaternion::Quaternion(Quaternion const& v) : X(v.X), Y(v.Y), Z(v.Z), W(v.W)
{
}

//////////////////////////////////////////////
// vector/vector operations
inline Quaternion Quaternion::operator+(Quaternion const& rhs) const
{
	return Quaternion(this->X + rhs.X, this->Y + rhs.Y, this->Z + rhs.Z, this->W + rhs.W);
}
 
inline Quaternion Quaternion::operator-(Quaternion const& rhs) const
{
	return Quaternion(this->X - rhs.X, this->Y - rhs.Y, this->Z - rhs.Z, this->W - rhs.W);
}

inline Quaternion Quaternion::operator*(Quaternion const& rhs) const
{
	return Quaternion(	W * rhs.X + X * rhs.W + Y * rhs.Z - Z * rhs.Y,
						W * rhs.Y + Y * rhs.W + Z * rhs.X - X * rhs.Z,
						W * rhs.Z + Z * rhs.W + X * rhs.Y - Y * rhs.X,
						W * rhs.W - X * rhs.X - Y * rhs.Y - Z * rhs.Z);

}

inline Vector3 Quaternion::operator*(Vector3 const& rhs) const
{
	Vector3 uv, uuv;
	Vector3 qvec(X, Y, Z);
	uv = qvec.cross(rhs);
	uuv = qvec.cross(uv);
	uv *= (2.0 * W);
	uuv *= 2.0;

	return rhs + uv + uuv;
}

inline Quaternion Quaternion::operator/(Quaternion const& rhs) const
{
	return Quaternion(this->X / rhs.X, this->Y / rhs.Y, this->Z / rhs.Z, this->W / rhs.W);
}

inline Quaternion Quaternion::operator-() const
{
	return Quaternion(-(this->X), -(this->Y), -(this->Z), -(this->W));
}

inline Quaternion& Quaternion::operator+=(Quaternion const& rhs)
{
	this->X += rhs.X;
	this->Y += rhs.Y;
	this->Z += rhs.Z;
	this->W += rhs.W;
	return *this;
}

inline Quaternion& Quaternion::operator-=(Quaternion const& rhs)
{
	this->X -= rhs.X;
	this->Y -= rhs.Y;
	this->Z -= rhs.Z;
	this->W -= rhs.W;
	return *this;
}
 
inline Quaternion& Quaternion::operator*=(Quaternion const& rhs)
{
	*this = *this * rhs;
	return *this;
}

inline Quaternion& Quaternion::operator/=(Quaternion const& rhs)
{
	this->X /= rhs.X;
	this->Y /= rhs.Y;
	this->Z /= rhs.Z;
	this->W /= rhs.W;
	return *this;
} 

inline Quaternion& Quaternion::operator=(Quaternion const& rhs)
{
	this->X = rhs.X;
	this->Y = rhs.Y;
	this->Z = rhs.Z;
	this->W = rhs.W;
	return *this;
}

inline Quaternion const& Quaternion::Identity()
{
	return _Identity;
}

inline bool operator!=(Quaternion const& a, Quaternion const& b)
{
	return (!Equals(a.X, b.X) || !Equals(a.Y, b.Y) || !Equals(a.Z, b.Z) || !Equals(a.W, b.W));
}

inline bool operator==(Quaternion const& a, Quaternion const& b)
{
	return !operator!=(a,b);
}
}