#pragma once
#include <sstream>
#include <iostream>
#include <cmath>
#include "Math.h"
#include "Vector3.h"
#include <limits>
#include <iomanip>


namespace BT_Geometry
{
	class Matrix4;
	class Vector3;
	class Quaternion;

class Vector4
{
public:		// Constructors & Destructor
	Vector4(void);
	Vector4(double x, double y, double z, double w);
	Vector4(Vector4 const& rhs);
	~Vector4(void);

public:		// Methods
	void		normalize		(void);
	void		normalize		(Vector4& dest) const;
	double		distance		(Vector4 const& rhs) const;
	double		distanceSquared	(Vector4 const& rhs) const;
	double		length			(void) const;
	double		lengthSquared	(void) const;
	double		dot				(Vector4 const& rhs) const;
	void		clamp			(Vector4 const& min, Vector4 const& max);
	void		lerp			(Vector4 const& startPoint, Vector4 const& endPoint, double factor);
	void		sLerp			(Vector4 const& startPoint, Vector4 const& endPoint, double factor, bool clampToMinorAngle = false);
	Vector4&	cross			(Vector4 const& v1, Vector4 const& v2, Vector4 const& v3);

public:		// Operations
	
	//////////////////////////////////////////////
	// vector/scalar operations
	Vector4 operator+(double rhs) const;
    Vector4 operator-(double rhs) const; 
    Vector4 operator*(double rhs) const;
    Vector4 operator/(double rhs) const;
     
    Vector4& operator+=(double rhs);
    Vector4& operator-=(double rhs);
    Vector4& operator*=(double rhs);
    Vector4& operator/=(double rhs); 
	//////////////////////////////////////////////

	//////////////////////////////////////////////
	// vector/vector operations
	Vector4 operator+(Vector4 const& rhs) const; 
    Vector4 operator-(Vector4 const& rhs) const;
    Vector4 operator*(Vector4 const& rhs) const; 
    Vector4 operator/(Vector4 const& rhs) const; 
    Vector4 operator-() const;

    Vector4& operator+=(Vector4 const& rhs); 
    Vector4& operator-=(Vector4 const& rhs); 
    Vector4& operator*=(Vector4 const& rhs); 
    Vector4& operator/=(Vector4 const& rhs); 
	
	Vector4& operator=(Vector4 const& rhs);
	Vector4& operator=(Vector3 const& rhs);
	//////////////////////////////////////////////

	template<class E, class U>
	friend std::basic_ostream<E, U>& operator<< (std::basic_ostream<E, U>& os, Vector4 const& rhs)
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
		return ((int)X * (int)Y * (int)Z * (int)W);
	}

public:
	static Vector4 const& Zero();

public:		// Member Variables
	double X;
	double Y;
	double Z;
	double W;

private:		// Static Variables
	static Vector4 _Zero;

};

///////////////////////////////////////////////////////////////////////////////////////
// Vector4's inline functions

//////////////////////////////////////////////
// Constructors & Destructors
inline Vector4::Vector4()
	: X(0.0), Y(0.0), Z(0.0), W(0.0)
{
}
inline Vector4::Vector4(double x, double y, double z, double w)
	: X(x), Y(y), Z(z), W(w)
{
}
inline Vector4::Vector4(Vector4 const& rhs) : X(rhs.X), Y(rhs.Y), Z(rhs.Z), W(rhs.W)
{
}
inline Vector4::~Vector4(void)
{
}
//////////////////////////////////////////////

//////////////////////////////////////////////
// Methods
inline void Vector4::normalize(void) 
{
	double l = length();

	if (!(Equals(0.0, l)))
	{
		double m = 1.0 / l;
		X *= m;
		Y *= m;
		Z *= m;
		W *= m;
	}
}

inline void Vector4::normalize(Vector4& dest) const
{
	dest = *this;
	dest.normalize();
}

inline double Vector4::length() const 
{
	return sqrt(lengthSquared()); 
}

inline double Vector4::lengthSquared() const
{
	return X * X + Y * Y + Z * Z + W * W; 
}

inline double Vector4::distance(Vector4 const& rhs) const
{
	return sqrt(distanceSquared(rhs));
}

inline double Vector4::distanceSquared(Vector4 const& rhs) const
{
	double x = rhs.X - this->X;
	double y = rhs.Y - this->Y;
	double z = rhs.Z - this->Z;
	double w = rhs.W - this->W;
	return x * x + y * y + z * z + w * w;
}
inline double Vector4::dot(Vector4 const& rhs) const
{
	return this->X * rhs.X + this->Y * rhs.Y + this->Z * rhs.Z + this->W * rhs.W;
}
inline Vector4& Vector4::cross(Vector4 const& v1, Vector4 const& v2, Vector4 const& v3)
{
    this->X = v1.Y * (v2.Z * v3.W - v3.Z * v2.W) - v1.Z * (v2.Y * v3.W - v3.Y * v2.W) + v1.W * (v2.Y * v3.Z - v2.Z *v3.Y);
    this->Y = -(v1.X * (v2.Z * v3.W - v3.Z * v2.W) - v1.Z * (v2.X * v3.W - v3.X * v2.W) + v1.W * (v2.X * v3.Z - v3.X * v2.Z));
    this->Z = v1.X * (v2.Y * v3.W - v3.Y * v2.W) - v1.Y * (v2.X *v3.W - v3.X * v2.W) + v1.W * (v2.X * v3.Y - v3.X * v2.Y);
    this->W = -(v1.X * (v2.Y * v3.Z - v3.Y * v2.Z) - v1.Y * (v2.X * v3.Z - v3.X *v2.Z) + v1.Z * (v2.X * v3.Y - v3.X * v2.Y));
	return *this;
}

inline void Vector4::sLerp(Vector4 const& startPoint, Vector4 const& endPoint, double factor, bool clampToMinorAngle)
{
	Vector4 startPointNormalized = startPoint;
	startPointNormalized.normalize();
	Vector4 endPointNormalized = endPoint;
	endPointNormalized.normalize();

	double dotProduct = startPointNormalized.dot(endPointNormalized);
	Vector4 localEndPoint;

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
		W = ((startPoint.W * sinInverseAmountAngle) + (localEndPoint.W * sinAmountAngle)) / sinDenominator;
	}
	else
	{
		lerp(startPoint, endPoint, factor);
	}
}

inline void Vector4::clamp(Vector4 const& min, Vector4 const& max)
{
	X = (X > max.X) ? max.X : X;
	X = (X < min.X) ? min.X : X;

	Y = (Y > max.Y) ? max.Y : Y;
	Y = (Y < min.Y) ? min.Y : Y;

	Z = (Z > max.Z) ? max.Z : Z;
	Z = (Z < min.Z) ? min.Z : Z;

	W = (W > max.W) ? max.W : W;
	W = (W < min.W) ? min.W : W;
}

inline void Vector4::lerp(Vector4 const& startPoint, Vector4 const& endPoint, double factor)
{
	X = startPoint.X + ((endPoint.X - startPoint.X) * factor);
	Y = startPoint.Y + ((endPoint.Y - startPoint.Y) * factor);
	Z = startPoint.Z + ((endPoint.Z - startPoint.Z) * factor);
	W = startPoint.W + ((endPoint.W - startPoint.W) * factor);
}
//////////////////////////////////////////////

//////////////////////////////////////////////
// vector/scalar operations
inline Vector4 Vector4::operator+(double rhs) const
{
	return Vector4(this->X + rhs, this->Y + rhs, this->Z + rhs, this->W + rhs);
}

inline Vector4 Vector4::operator-(double rhs) const
{
	return Vector4(this->X - rhs, this->Y - rhs, this->Z - rhs, this->W - rhs);
}

inline Vector4 Vector4::operator*(double rhs) const
{
	return Vector4(this->X * rhs, this->Y * rhs, this->Z * rhs, this->W * rhs);
}

inline Vector4 Vector4::operator/(double rhs) const
{
	return Vector4(this->X / rhs, this->Y / rhs, this->Z / rhs, this->W / rhs);
}
 
inline Vector4& Vector4::operator+=(double rhs)
{
	this->X += rhs;
	this->Y += rhs;
	this->Z += rhs;
	this->W += rhs;
	return *this;
}

inline Vector4& Vector4::operator-=(double rhs)
{
	this->X -= rhs;
	this->Y -= rhs;
	this->Z -= rhs;
	this->W -= rhs;
	return *this;
}

inline Vector4& Vector4::operator*=(double rhs)
{
	this->X *= rhs;
	this->Y *= rhs;
	this->Z *= rhs;
	this->W *= rhs;
	return *this;
}

inline Vector4& Vector4::operator/=(double rhs)
{
	this->X /= rhs;
	this->Y /= rhs;
	this->Z /= rhs;
	this->W /= rhs;
	return *this;
}
inline Vector4 operator*(double lhs, Vector4 const& rhs)
{ 
	return Vector4(lhs * rhs.X, lhs * rhs.Y, lhs * rhs.Z, lhs * rhs.W);	
}

inline Vector4 operator/(double lhs, Vector4 const& rhs)
{ 
	return Vector4(lhs / rhs.X, lhs / rhs.Y, lhs / rhs.Z, lhs / rhs.W);	
}

inline Vector4 operator+(double lhs, Vector4 const& rhs)
{ 
	return Vector4(lhs + rhs.X, lhs + rhs.Y, lhs + rhs.Z, lhs + rhs.W);	
}

inline Vector4 operator-(double lhs, Vector4 const& rhs)
{ 
	return Vector4(lhs - rhs.X, lhs - rhs.Y, lhs - rhs.Z, lhs - rhs.W);	
}


//////////////////////////////////////////////

//////////////////////////////////////////////
// vector/vector operations
inline Vector4 Vector4::operator+(Vector4 const& rhs) const
{
	return Vector4(this->X + rhs.X, this->Y + rhs.Y, this->Z + rhs.Z, this->W + rhs.W);
}
 
inline Vector4 Vector4::operator-(Vector4 const& rhs) const
{
	return Vector4(this->X - rhs.X, this->Y - rhs.Y, this->Z - rhs.Z, this->W - rhs.W);
}

inline Vector4 Vector4::operator*(Vector4 const& rhs) const
{
	return Vector4(this->X * rhs.X, this->Y * rhs.Y, this->Z * rhs.Z, this->W * rhs.W);
}

inline Vector4 Vector4::operator/(Vector4 const& rhs) const
{
	return Vector4(this->X / rhs.X, this->Y / rhs.Y, this->Z / rhs.Z, this->W / rhs.W);
}

inline Vector4 Vector4::operator-() const
{
	return Vector4(-(this->X), -(this->Y), -(this->Z), -(this->W));
}

inline Vector4& Vector4::operator+=(Vector4 const& rhs)
{
	this->X += rhs.X;
	this->Y += rhs.Y;
	this->Z += rhs.Z;
	this->W += rhs.W;
	return *this;
}

inline Vector4& Vector4::operator-=(Vector4 const& rhs)
{
	this->X -= rhs.X;
	this->Y -= rhs.Y;
	this->Z -= rhs.Z;
	this->W -= rhs.W;
	return *this;
}
 
inline Vector4& Vector4::operator*=(Vector4 const& rhs)
{
	this->X *= rhs.X;
	this->Y *= rhs.Y;
	this->Z *= rhs.Z;
	this->W *= rhs.W;
	return *this;
}

inline Vector4& Vector4::operator/=(Vector4 const& rhs)
{
	this->X /= rhs.X;
	this->Y /= rhs.Y;
	this->Z /= rhs.Z;
	this->W /= rhs.W;
	return *this;
} 



inline Vector4& Vector4::operator=(Vector4 const& rhs)
{
	this->X = rhs.X;
	this->Y = rhs.Y;
	this->Z = rhs.Z;
	this->W = rhs.W;
	return *this;
}

inline Vector4& Vector4::operator=(Vector3 const& rhs)
{
	this->X = rhs.X;
	this->Y = rhs.Y;
	this->Z = rhs.Z;
	this->W = 1.0;
	return *this;
}

inline Vector4 const& Vector4::Zero()
{
	return _Zero;
}

inline bool operator!=(Vector4 const& a, Vector4 const& b)
{
	return (!Equals(a.X, b.X) || !Equals(a.Y, b.Y) || !Equals(a.Z, b.Z) || !Equals(a.W, b.W));
}

inline bool operator==(Vector4 const& a, Vector4 const& b) 
{
	return !operator!=(a,b);
}
//////////////////////////////////////////////

}