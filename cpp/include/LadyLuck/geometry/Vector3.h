#pragma once

#pragma warning(disable : 4293) 

#include <sstream>
#include <iostream>
#include <limits>
#include <iomanip>

#include "Math.h"

namespace BT_Geometry
{
class Matrix4;
class Vector4;
class Quaternion;

class Vector3
{
public:
	typedef double Scalar_Type;
	// Constructors & Destructor
	Vector3();
	Vector3(double x, double y, double z);
	Vector3(Vector3 const& rhs);
	~Vector3(void);
	
public:		// Methods
	void			normalize(void);
	Vector3	cross	(Vector3 const& rhs) const;

	double 	distance				(Vector3 const& rhs) const;
	double 	distanceSquared			(Vector3 const& rhs) const;
	double 	length					(void) const;
	double 	lengthSquared			(void) const;
	double 	dot						(Vector3 const& rhs) const;
	void	clamp					(Vector3 const& min, Vector3 const& max);
	void	lerp					(Vector3 const& startPoint, Vector3 const& endPoint, double factor);
	void	sLerp					(Vector3 const& startPoint, Vector3 const& endPoint, double factor, bool clampToMinorAngle = false);
	double	angleBetween			(Vector3 const& rhs) const;
	

public:		// Operations
	//////////////////////////////////////////////
	// vector/scalar operations
	Vector3 operator+(double rhs) const;
    Vector3 operator-(double rhs) const; 
    Vector3 operator*(double rhs) const;
    Vector3 operator/(double rhs) const;
     
    Vector3& operator+=(double rhs);
    Vector3& operator-=(double rhs);
    Vector3& operator*=(double rhs);
    Vector3& operator/=(double rhs); 
	//////////////////////////////////////////////

	//////////////////////////////////////////////
	// vector/vector operations
	Vector3 operator+(Vector3 const& rhs) const; 
    Vector3 operator-(Vector3 const& rhs) const;
    Vector3 operator*(Vector3 const& rhs) const; 
    Vector3 operator/(Vector3 const& rhs) const; 
    Vector3 operator-() const;

    Vector3& operator+=(Vector3 const& rhs); 
    Vector3& operator-=(Vector3 const& rhs); 
    Vector3& operator*=(Vector3 const& rhs); 
    Vector3& operator/=(Vector3 const& rhs); 
	
	Vector3& operator=(Vector3 const& rhs);
	//////////////////////////////////////////////

	template<class E, class U>
	friend std::basic_ostream<E, U>& operator<< (std::basic_ostream<E, U>& os, Vector3 const& rhs)
	{
		os << L"(" << std::setprecision(std::numeric_limits<double>::digits10) << rhs.X << L"," 
			<< std::setprecision(std::numeric_limits<double>::digits10) << rhs.Y << L"," 
			<< std::setprecision(std::numeric_limits<double>::digits10) << rhs.Z << L")";

		return os;
	}

public:		// Getters & Setters
	int getHashCode() const
	{
		return ((int)X * (int)Y * (int)Z);
	}

public:
	static Vector3 const& Zero();
	static Vector3 const& One();
	static Vector3 const& Right();
	static Vector3 const& Up();
	static Vector3 const& Forward();

public:		// Member Variables
	double X;
	double Y;
	double Z;

private:		// Static Variables
	static Vector3 _Zero;
	static Vector3 _One;
	static Vector3 _Right;
	static Vector3 _Up;
	static Vector3 _Forward;
};


///////////////////////////////////////////////////////////////////////////////////////
// Vector3's inline functions

//////////////////////////////////////////////
// Constructors & Destructors
inline Vector3::Vector3()
		: X(0.0), Y(0.0), Z(0.0)
{
}
inline Vector3::Vector3(double x, double y, double z)
		: X(x), Y(y), Z(z)
{
}
inline Vector3::Vector3(Vector3 const& rhs) : X(rhs.X), Y(rhs.Y), Z(rhs.Z)
{
}
inline Vector3::~Vector3(void)
{
}

//////////////////////////////////////////////

//////////////////////////////////////////////
// Methods
inline void Vector3::normalize(void) 
{
	double l = length();

	if (!(Equals(0.0, l)))
	{
		double m = 1.0 / l;
		X *= m;
		Y *= m;
		Z *= m;
	}
}

inline double Vector3::length() const 
{
	return sqrt(lengthSquared()); 
}

inline double Vector3::lengthSquared() const
{
	return X * X + Y * Y + Z * Z; 
}

inline double Vector3::distance(Vector3 const& rhs) const
{
	return sqrt(distanceSquared(rhs));
}

inline double Vector3::distanceSquared(Vector3 const& rhs) const
{
	double x = rhs.X - this->X;
	double y = rhs.Y - this->Y;
	double z = rhs.Z - this->Z;
	return x * x + y * y + z * z;
}

inline double Vector3::dot(Vector3 const& rhs) const
{
	return this->X * rhs.X + this->Y * rhs.Y + this->Z * rhs.Z;
}

inline void Vector3::clamp(Vector3 const& min, Vector3 const& max)
{
	X = (X > max.X) ? max.X : X;
	X = (X < min.X) ? min.X : X;

	Y = (Y > max.Y) ? max.Y : Y;
	Y = (Y < min.Y) ? min.Y : Y;

	Z = (Z > max.Z) ? max.Z : Z;
	Z = (Z < min.Z) ? min.Z : Z;
}

inline void Vector3::lerp(Vector3 const& startPoint, Vector3 const& endPoint, double factor)
{
	X = startPoint.X + ((endPoint.X - startPoint.X) * factor);
	Y = startPoint.Y + ((endPoint.Y - startPoint.Y) * factor);
	Z = startPoint.Z + ((endPoint.Z - startPoint.Z) * factor);
}

inline void Vector3::sLerp(Vector3 const& startPoint, Vector3 const& endPoint, double factor, bool clampToMinorAngle)
{
	Vector3 startPointNormalized = startPoint;
	startPointNormalized.normalize();
	Vector3 endPointNormalized = endPoint;
	endPointNormalized.normalize();

	double dotProduct = startPointNormalized.dot(endPointNormalized);
	Vector3 localEndPoint;

	if(clampToMinorAngle == true)
	{
		if(dotProduct >= 0.0)
		{
			localEndPoint = endPoint;
		}
		else
		{
			localEndPoint = -endPoint;
			dotProduct = -dotProduct;
		}
	}
	else
	{
		localEndPoint = endPoint;
	}

	if(dotProduct <= 0.95)
	{
		double interAngle = acos(dotProduct);
		double sinInverseAmountAngle = sin(interAngle * (1.0 - factor));
		double sinAmountAngle = sin(interAngle * factor);
		double sinDenominator = sin(interAngle);

		X = ((startPoint.X * sinInverseAmountAngle) + (localEndPoint.X * sinAmountAngle)) / sinDenominator;
		Y = ((startPoint.Y * sinInverseAmountAngle) + (localEndPoint.Y * sinAmountAngle)) / sinDenominator;
		Z = ((startPoint.Z * sinInverseAmountAngle) + (localEndPoint.Z * sinAmountAngle)) / sinDenominator;
	}
	else
	{
		lerp(startPoint, endPoint, factor);
	}
}

inline Vector3 Vector3::cross(Vector3 const& rhs) const
{
	return Vector3((this->Y * rhs.Z - this->Z * rhs.Y), (this->Z * rhs.X - this->X * rhs.Z), (this->X * rhs.Y - this->Y * rhs.X));
}

inline double Vector3::angleBetween(Vector3 const& rhs) const
{
	double lenProduct = length() * rhs.length();

	// Divide by zero check
	if(lenProduct < 1e-6f)
		lenProduct = 1e-6f;

	double d = dot(rhs) / lenProduct;

	d = Clamp(d, (double)-1.0, (double)1.0);
	return acos(d);
}

//////////////////////////////////////////////

//////////////////////////////////////////////
// vector/scalar operations
inline Vector3 Vector3::operator+(double rhs) const
{
	return Vector3(this->X + rhs, this->Y + rhs, this->Z + rhs);
}

inline Vector3 Vector3::operator-(double rhs) const
{
	return Vector3(this->X - rhs, this->Y - rhs, this->Z - rhs);
}

inline Vector3 Vector3::operator*(double rhs) const
{
	return Vector3(this->X * rhs, this->Y * rhs, this->Z * rhs);
}

inline Vector3 Vector3::operator/(double rhs) const
{
	return Vector3(this->X / rhs, this->Y / rhs, this->Z / rhs);
}
 
inline Vector3& Vector3::operator+=(double rhs)
{
	this->X += rhs;
	this->Y += rhs;
	this->Z += rhs;
	return *this;
}

inline Vector3& Vector3::operator-=(double rhs)
{
	this->X -= rhs;
	this->Y -= rhs;
	this->Z -= rhs;
	return *this;
}

inline Vector3& Vector3::operator*=(double rhs)
{
	this->X *= rhs;
	this->Y *= rhs;
	this->Z *= rhs;
	return *this;
}

inline Vector3& Vector3::operator/=(double rhs)
{
	this->X /= rhs;
	this->Y /= rhs;
	this->Z /= rhs;
	return *this;
}
inline Vector3 operator*(double lhs, Vector3 const& rhs)
{ 
	return Vector3(lhs * rhs.X, lhs * rhs.Y, lhs * rhs.Z);	
}

inline Vector3 operator/(double lhs, Vector3 const& rhs)
{ 
	return Vector3(lhs / rhs.X, lhs / rhs.Y, lhs / rhs.Z);	
}

inline Vector3 operator+(double lhs, Vector3 const& rhs)
{ 
	return Vector3(lhs + rhs.X, lhs + rhs.Y, lhs + rhs.Z);	
}

inline Vector3 operator-(double lhs, Vector3 const& rhs)
{ 
	return Vector3(lhs - rhs.X, lhs - rhs.Y, lhs - rhs.Z);	
}
//////////////////////////////////////////////

//////////////////////////////////////////////
// vector/vector operations
inline Vector3 Vector3::operator+(Vector3 const& rhs) const
{
	return Vector3(this->X + rhs.X, this->Y + rhs.Y, this->Z + rhs.Z);
}
 
inline Vector3 Vector3::operator-(Vector3 const& rhs) const
{
	return Vector3(this->X - rhs.X, this->Y - rhs.Y, this->Z - rhs.Z);
}

inline Vector3 Vector3::operator*(Vector3 const& rhs) const
{
	return Vector3(this->X * rhs.X, this->Y * rhs.Y, this->Z * rhs.Z);
}

inline Vector3 Vector3::operator/(Vector3 const& rhs) const
{
	return Vector3(this->X / rhs.X, this->Y / rhs.Y, this->Z / rhs.Z);
}

inline Vector3 Vector3::operator-() const
{
	return Vector3(-(this->X), -(this->Y), -(this->Z));
}

inline Vector3& Vector3::operator+=(Vector3 const& rhs)
{
	this->X += rhs.X;
	this->Y += rhs.Y;
	this->Z += rhs.Z;
	return *this;
}

inline Vector3& Vector3::operator-=(Vector3 const& rhs)
{
	this->X -= rhs.X;
	this->Y -= rhs.Y;
	this->Z -= rhs.Z;
	return *this;
}
 
inline Vector3& Vector3::operator*=(Vector3 const& rhs)
{
	this->X *= rhs.X;
	this->Y *= rhs.Y;
	this->Z *= rhs.Z;
	return *this;
}

inline Vector3& Vector3::operator/=(Vector3 const& rhs)
{
	this->X /= rhs.X;
	this->Y /= rhs.Y;
	this->Z /= rhs.Z;
	return *this;
} 

inline Vector3& Vector3::operator=(Vector3 const& rhs)
{
	this->X = rhs.X;
	this->Y = rhs.Y;
	this->Z = rhs.Z;
	return *this;
}

inline Vector3 const& Vector3::Zero()
{
	return _Zero;
}
inline Vector3 const& Vector3::One()
{
	return _One;
}
inline Vector3 const& Vector3::Right()
{
	return _Right;
}
inline Vector3 const& Vector3::Up()
{
	return _Up;
}
inline Vector3 const& Vector3::Forward()
{
	return _Forward;
}

inline bool operator!=(Vector3 const& a, Vector3 const& b)
{
	return (!Equals(a.X, b.X) || !Equals(a.Y, b.Y) || !Equals(a.Z, b.Z));
}

inline bool operator==(Vector3 const& a, Vector3 const& b)
{
	return !operator!=(a,b);
}

inline bool operator<(Vector3 const& a, Vector3 const& b)
{
	return a.X < b.X;
}
}