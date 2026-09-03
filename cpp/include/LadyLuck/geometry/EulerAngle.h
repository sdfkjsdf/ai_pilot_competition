#pragma once

#include "Angle.h"
//#include "String.h"
#include "Math.h"
#include <sstream>

namespace BT_Geometry
{

class AxisAngle;
class Quaternion;
class Matrix3;

/// <summary>
/// Eular angle (yaw, pitch, roll) Ŭ����
/// 
/// ���� ������ ����Ѵ�. �ð�������� ȸ���ϰ� yaw, pich, roll ������ ȸ���Ѵ�.
/// </summary>
class EulerAngle
{
public:

	EulerAngle(void);
	EulerAngle(Angle yaw, Angle pitch, Angle roll);
	explicit EulerAngle(AxisAngle const& v);
	explicit EulerAngle(Matrix3 const& v);
	explicit EulerAngle(Quaternion const& v);
	~EulerAngle(void);

public:
	void toQuaternion(Quaternion* dest) const;
	Quaternion toQuaternion(void) const;
	void toMatrix(Matrix3* dest) const;
	Matrix3 toMatrix(void) const;
	void toAxisAngle(AxisAngle* dest) const;
	AxisAngle toAxisAngle(void) const;

public:		// Operations
	//////////////////////////////////////////////
	// eulerAngle/scalar operations
	EulerAngle operator+(double rhv) const;
    EulerAngle operator-(double rhv) const; 
    EulerAngle operator*(double rhv) const;
    EulerAngle operator/(double rhv) const;
     
    EulerAngle& operator+=(double rhv);
    EulerAngle& operator-=(double rhv);
    EulerAngle& operator*=(double rhv);
    EulerAngle& operator/=(double rhv); 
	//////////////////////////////////////////////

	//////////////////////////////////////////////
	// eulerAngle/eulerAngle operations
	EulerAngle operator+(EulerAngle const& rhv) const; 
    EulerAngle operator-(EulerAngle const& rhv) const;
    EulerAngle operator*(EulerAngle const& rhv) const; 
    EulerAngle operator/(EulerAngle const& rhv) const; 
    EulerAngle operator-() const;

    EulerAngle& operator+=(EulerAngle const& rhv); 
    EulerAngle& operator-=(EulerAngle const& rhv); 
    EulerAngle& operator*=(EulerAngle const& rhv); 
    EulerAngle& operator/=(EulerAngle const& rhv); 
	
	EulerAngle& operator=(EulerAngle const& rhv);
	//////////////////////////////////////////////

	template<class E, class U>
	friend std::basic_ostream<E, U>& operator<< (std::basic_ostream<E, U>& os, EulerAngle const& rhv)
	{
		os << L"(" << rhv.Yaw << L"," << rhv.Pitch <<  L"," << rhv.Roll << L")";
		return os;
	}

public:		// Getters & Setters
	int getHashCode() const
	{
		return (int)(Yaw * Pitch * Roll * 1000000);
	}

public:
	static EulerAngle const& Zero(void);

public:
	Angle Yaw;
	Angle Pitch;
	Angle Roll;

private:
	static EulerAngle _Zero;

};

inline EulerAngle::EulerAngle(void) : Yaw(0.0), Pitch(0.0), Roll(0.0) {}
inline EulerAngle::~EulerAngle(void) {}
inline EulerAngle::EulerAngle(Angle yaw, Angle pitch, Angle roll) 
: Yaw(yaw), Pitch(pitch), Roll(roll) {}

//////////////////////////////////////////////

//////////////////////////////////////////////
// eulerAngle/scalar operations
inline EulerAngle EulerAngle::operator+(double rhv) const
{
	return EulerAngle(this->Yaw + rhv, this->Pitch + rhv, this->Roll + rhv);
}

inline EulerAngle EulerAngle::operator-(double rhv) const
{
	return EulerAngle(this->Yaw - rhv, this->Pitch - rhv, this->Roll - rhv);
}

inline EulerAngle EulerAngle::operator*(double rhv) const
{
	return EulerAngle(this->Yaw * rhv, this->Pitch * rhv, this->Roll * rhv);
}

inline EulerAngle EulerAngle::operator/(double rhv) const
{
	return EulerAngle(this->Yaw / rhv, this->Pitch / rhv, this->Roll / rhv);
}
 
inline EulerAngle& EulerAngle::operator+=(double rhv)
{
	this->Yaw += rhv;
	this->Pitch += rhv;
	this->Roll += rhv;
	return *this;
}

inline EulerAngle& EulerAngle::operator-=(double rhv)
{
	this->Yaw -= rhv;
	this->Pitch -= rhv;
	this->Roll -= rhv;
	return *this;
}

inline EulerAngle& EulerAngle::operator*=(double rhv)
{
	this->Yaw *= rhv;
	this->Pitch *= rhv;
	this->Roll *= rhv;
	return *this;
}

inline EulerAngle& EulerAngle::operator/=(double rhv)
{
	this->Yaw /= rhv;
	this->Pitch /= rhv;
	this->Roll /= rhv;
	return *this;
}
inline EulerAngle operator*(double lhv, EulerAngle const& rhv)
{ 
	return EulerAngle(lhv * rhv.Yaw, lhv * rhv.Pitch, lhv * rhv.Roll);	
}

inline EulerAngle operator/(double lhv, EulerAngle const& rhv)
{ 
	return EulerAngle(lhv / rhv.Yaw, lhv / rhv.Pitch, lhv / rhv.Roll);	
}

inline EulerAngle operator+(double lhv, EulerAngle const& rhv)
{ 
	return EulerAngle(lhv + rhv.Yaw, lhv + rhv.Pitch, lhv + rhv.Roll);	
}

inline EulerAngle operator-(double lhv, EulerAngle const& rhv)
{ 
	return EulerAngle(lhv - rhv.Yaw, lhv - rhv.Pitch, lhv - rhv.Roll);	
}
//////////////////////////////////////////////

//////////////////////////////////////////////
// eulerAngle/eulerAngle operations
inline EulerAngle EulerAngle::operator+(EulerAngle const& rhv) const
{
	return EulerAngle(this->Yaw + rhv.Yaw, this->Pitch + rhv.Pitch, this->Roll + rhv.Roll);
}
 
inline EulerAngle EulerAngle::operator-(EulerAngle const& rhv) const
{
	return EulerAngle(this->Yaw - rhv.Yaw, this->Pitch - rhv.Pitch, this->Roll - rhv.Roll);
}

inline EulerAngle EulerAngle::operator*(EulerAngle const& rhv) const
{
	return EulerAngle(this->Yaw * rhv.Yaw, this->Pitch * rhv.Pitch, this->Roll * rhv.Roll);
}

inline EulerAngle EulerAngle::operator/(EulerAngle const& rhv) const
{
	return EulerAngle(this->Yaw / rhv.Yaw, this->Pitch / rhv.Pitch, this->Roll / rhv.Roll);
}

inline EulerAngle EulerAngle::operator-() const
{
	return EulerAngle(-(this->Yaw), -(this->Pitch), -(this->Roll));
}

inline EulerAngle& EulerAngle::operator+=(EulerAngle const& rhv)
{
	this->Yaw += rhv.Yaw;
	this->Pitch += rhv.Pitch;
	this->Roll += rhv.Roll;
	return *this;
}

inline EulerAngle& EulerAngle::operator-=(EulerAngle const& rhv)
{
	this->Yaw -= rhv.Yaw;
	this->Pitch -= rhv.Pitch;
	this->Roll -= rhv.Roll;
	return *this;
}
 
inline EulerAngle& EulerAngle::operator*=(EulerAngle const& rhv)
{
	this->Yaw *= rhv.Yaw;
	this->Pitch *= rhv.Pitch;
	this->Roll *= rhv.Roll;
	return *this;
}

inline EulerAngle& EulerAngle::operator/=(EulerAngle const& rhv)
{
	this->Yaw /= rhv.Yaw;
	this->Pitch /= rhv.Pitch;
	this->Roll /= rhv.Roll;
	return *this;
} 


inline EulerAngle& EulerAngle::operator=(EulerAngle const& rhv)
{
	this->Yaw = rhv.Yaw;
	this->Pitch = rhv.Pitch;
	this->Roll = rhv.Roll;
	return *this;
}

inline EulerAngle const& EulerAngle::Zero()
{
	return _Zero;
}

inline bool operator!=(EulerAngle const& a, EulerAngle const& b)
{
	return (!Equals(a.Yaw, b.Yaw) || !Equals(a.Pitch, b.Pitch) || !Equals(a.Roll, b.Roll));
}

inline bool operator==(EulerAngle const& a, EulerAngle const& b)
{
	return !operator!=(a,b);
}
}