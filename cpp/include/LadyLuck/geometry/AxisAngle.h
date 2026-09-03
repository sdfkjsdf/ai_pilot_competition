#pragma once
#include "Angle.h"
#include "Math.h"
#include "Vector3.h"

namespace BT_Geometry
{

class Quaternion;
class Matrix3;
class EulerAngle;

/// <summary>
/// Ư�� ���� �������� ȸ���ϴ� �ڷᱸ��
/// ���� ������ ����Ѵ�.
/// </summary>
class AxisAngle
{
public:
	AxisAngle(void);
	AxisAngle(AxisAngle const& a);
	AxisAngle(double axisX, double axisY, double axisZ, Angle angle);
	AxisAngle(Vector3 const& axis, Angle angle);
	explicit AxisAngle(EulerAngle const& e);
	explicit AxisAngle(Matrix3 const& m);
	explicit AxisAngle(Quaternion const& q);
	~AxisAngle(void);

public:

	/// <summary>
	/// �ֽĽ��ޱۿ� �����Ǵ� ���ʹϾ��� ���Ѵ�.
	/// </summary>
	/// <param name="dest">�ֽĽ��ޱۿ� �����Ǵ� ���ʹϾ�</param>
	void toQuaternion(Quaternion* dest) const;
	/// <summary>
	/// �ֽĽ��ޱۿ� �����Ǵ� ���ʹϾ��� ���Ѵ�.
	/// </summary>
	/// <returns>�ֽĽ��ޱۿ� �����Ǵ� ���ʹϾ�</returns>
	Quaternion toQuaternion(void) const;

	/// <summary>
	/// �ֽĽ��ޱۿ� �����Ǵ� ���Ϸ��ޱ��� ���Ѵ�.
	/// </summary>
	/// <param name="dest">�ֽĽ��ޱۿ� �����Ǵ� ���Ϸ��ޱ�</param>
	void toEulerAngle(EulerAngle* dest) const;
	
	/// <summary>
	/// �ֽĽ��ޱۿ� �����Ǵ� ���Ϸ��ޱ��� ���Ѵ�.
	/// </summary>
	/// <return>�ֽĽ��ޱۿ� �����Ǵ� ���Ϸ��ޱ�</return>
	EulerAngle toEulerAngle(void) const;

	/// <summary>
	/// �ֽĽ��ޱۿ� �����Ǵ� ����� ���Ѵ�.
	/// </summary>
	/// <param name="dest">�ֽĽ��ޱۿ� �����Ǵ� ���</param>
	void toMatrix(Matrix3* dest) const;

	/// <summary>
	/// �ֽĽ��ޱۿ� �����Ǵ� ����� ���Ѵ�.
	/// </summary>
	/// <return>�ֽĽ��ޱۿ� �����Ǵ� ���</return>
	Matrix3  toMatrix(void) const;


public:		// Operations
	
	
	AxisAngle& operator=(AxisAngle const& rhv);

	template<class E, class U>
	friend std::basic_ostream<E, U>& operator<< (std::basic_ostream<E, U>& os, AxisAngle const& rhv)
	{
		os << L"(" << rhv.Axis.X << L"," << rhv.Axis.Y <<  L"," << rhv.Axis.Z << L"," << rhv.AngleValue << L")";
		return os;
	}

public:		// Getters & Setters
	int getHashCode() const
	{
		return (int)(Axis.getHashCode() * AngleValue);
	}

public:
	static AxisAngle const& Zero(void);

public:
	Vector3 Axis;
	Angle AngleValue;

private:
	static AxisAngle _Zero;	
};

inline AxisAngle::AxisAngle(void) : AngleValue(0.0){}
inline AxisAngle::~AxisAngle(void) {}
inline AxisAngle::AxisAngle(double axisX, double axisY, double axisZ, Angle angle)
: Axis(axisX, axisY, axisZ), AngleValue(angle) {}
inline AxisAngle::AxisAngle(Vector3 const& axis, Angle angle)
: Axis(axis), AngleValue(angle) {}
inline AxisAngle::AxisAngle(AxisAngle const& rhv)
:Axis(rhv.Axis), AngleValue(rhv.AngleValue){}

inline AxisAngle& AxisAngle::operator=(AxisAngle const& rhv)
{
	this->Axis.X = rhv.Axis.X;
	this->Axis.Y = rhv.Axis.Y;
	this->Axis.Z = rhv.Axis.Z;
	this->AngleValue = rhv.AngleValue;
	return *this;
}

inline AxisAngle const& AxisAngle::Zero(void)
{
	return _Zero;
}

inline bool operator!=(AxisAngle const& a, AxisAngle const& b) 
{
	return (!Equals(a.Axis.X, b.Axis.X) || !Equals(a.Axis.Y, b.Axis.Y) || !Equals(a.Axis.Z, b.Axis.Z) || !Equals(a.AngleValue, b.AngleValue));
}

inline bool operator==(AxisAngle const& a, AxisAngle const& b) 
{
	return !operator!=(a,b);
}

}


