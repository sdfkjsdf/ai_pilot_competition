#pragma once

#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <float.h>

#ifdef PI
#undef PI
#endif

namespace BT_Geometry
{

	class Vector3;
	class Vector4;
	class Matrix3;
	class Matrix4;
	class Quaternion;

	const double PI = 3.14159265358979323846;
	const double PI_OVER_2 = 1.57079632679489661923;
	const double DEGTORAD = PI / 180.0;
	const double RADTODEG = 180.0 / PI;
	const double ROOT_2 = 1.41421356237;
	const double RECIPROCAL_ROOT_2 = 1.0 / ROOT_2;
	const double ROOT_3 = 1.73205080757;
	const double RECIPROCAL_ROOT_3 = 1.0 / ROOT_3;

	const float SINGLE_ROUNDING_ERROR = 1.192092896e-07F;
	const double DOUBLE_ROUNDING_ERROR = 2.2204460492503131e-016;
	const unsigned int IEEE_1_0	= 0x3f800000; // integer representation of 1.0
	const unsigned int IEEE_255_0 = 0x437f0000; // integer representation of 255.0
	const double EARTH_RADIUS = 6378137;

	const double MAX_DOUBLE_VALUE = DBL_MAX;
	const double MIN_DOUBLE_VALUE = DBL_MIN;
	const float MAX_FLOAT_VALUE = FLT_MAX;
	const float MIN_FLOAT_VALUE = FLT_MIN;

	template <class T> bool SAME_SIGN(T a, T b) { return (a<0 && b<0   ||   a>0 && b>0); }
	template <class T> T    Clamp(T const& x, T const& min_v, T const& max_v) { return x<min_v?  min_v: (x>max_v? max_v: x); }
	template <class T> T    Lerp(T const& a,T const& b, double t) { return a*(1.0-t) + b*t; }

	/// <summary>
	/// float�� ��(�������� : SINGLE_ROUNDING_ERROR)
	/// </summary>
	/// <param name="a">�񱳴��1</param>
	/// <param name="b">�񱳴��2</param>
	/// <returns>true: �� ���� �������� ��, false: �� ���� �������� ��</returns>
	inline bool Equals(float a, float b)
	{
		return (a + SINGLE_ROUNDING_ERROR >= b) && (a - SINGLE_ROUNDING_ERROR <= b);
	}
	/// <summary>
	/// float�� ��(�������� : SINGLE_ROUNDING_ERROR)
	/// </summary>
	/// <param name="a">�񱳴��1</param>
	/// <param name="b">�񱳴��2</param>
	/// <returns>
	/// true: (a - b)�� �������� ���� ����.
	/// false: (a - b)�� �������� ���� ŭ.
	/// </returns>
	inline bool LessThanOrEquals(float a, float b)
	{
		return (a - b) < SINGLE_ROUNDING_ERROR;
	}
	/// <summary>
	/// float�� ��(�������� : SINGLE_ROUNDING_ERROR)
	/// </summary>
	/// <param name="a">�񱳴��1</param>
	/// <param name="b">�񱳴��2</param>
	/// <returns>
	/// true: (b - a)�� �������� ���� ����.
	/// false: (b - a)�� �������� ���� ŭ.
	/// </returns>
	inline bool GreaterThanOrEquals(float a, float b)
	{
		return (b - a) < SINGLE_ROUNDING_ERROR;
	}

	inline bool LessThan(float a, float b)
	{
		return !GreaterThanOrEquals(a, b);
	}

	inline bool GreaterThan(float a, float b)
	{
		return !LessThanOrEquals(a, b);
	}

	/// <summary>
	/// -SINGLE_ROUNDING_ERROR ~ SINGLE_ROUNDING_ERROR������ ���̸� zero�� �Ǵ�
	/// </summary>
	/// <param name="a">zero������ ��</param>
	/// <returns>���� ���</returns>
	inline bool IsZero(float a)
	{
		return fabs ( a ) < SINGLE_ROUNDING_ERROR;
	}
	/// <summary>
	/// double�� ��(�������� : DOUBLE_ROUNDING_ERROR)
	/// </summary>
	/// <param name="a">�񱳴��1</param>
	/// <param name="b">�񱳴��2</param>
	/// <returns>true: �� ���� �������� ��, false: �� ���� �������� ��</returns>
	inline bool Equals(double a, double b)
	{
		return (a + DOUBLE_ROUNDING_ERROR >= b) && (a - DOUBLE_ROUNDING_ERROR <= b);
	}
	/// <summary>
	/// float�� ��(�������� : DOUBLE_ROUNDING_ERROR)
	/// </summary>
	/// <param name="a">�񱳴��1</param>
	/// <param name="b">�񱳴��2</param>
	/// <returns>
	/// true: (a - b)�� �������� ���� ����.
	/// false: (a - b)�� �������� ���� ŭ.
	/// </returns>
	inline bool LessThanOrEquals(double a, double b)
	{
		return (a - b) < DOUBLE_ROUNDING_ERROR;
	}
	/// <summary>
	/// float�� ��(�������� : DOUBLE_ROUNDING_ERROR)
	/// </summary>
	/// <param name="a">�񱳴��1</param>
	/// <param name="b">�񱳴��2</param>
	/// <returns>
	/// true: (b - a)�� �������� ���� ����.
	/// false: (b - a)�� �������� ���� ŭ.
	/// </returns>
	inline bool GreaterThanOrEquals(double a, double b)
	{
		return (b - a) < DOUBLE_ROUNDING_ERROR;
	}

	inline bool LessThan(double a, double b)
	{
		return !GreaterThanOrEquals(a, b);
	}

	inline bool GreaterThan(double a, double b)
	{
		return !LessThanOrEquals(a, b);
	}

	/// <summary>
	/// -DOUBLE_ROUNDING_ERROR ~ DOUBLE_ROUNDING_ERROR������ ���̸� zero�� �Ǵ�
	/// </summary>
	/// <param name="a">zero������ ��</param>
	/// <returns>���� ���</returns>
	inline bool IsZero(double a)
	{
		return fabs ( a ) < DOUBLE_ROUNDING_ERROR;
	}

	/// <summary>
	/// 1 / sqrt(x)
	/// </summary>
	/// <param name="x">�Է°�</param>
	/// <returns>�����</returns>
	inline float ReciprocalSquareRoot(float x)
	{
		unsigned int tmp = ((IEEE_1_0 << 1) + IEEE_1_0 - *(unsigned int*)&x) >> 1;   
		float y = *(float*)&tmp;                                             
		return y * (1.47f - 0.47f * x * y * y);
	}

	/// <summary>
	/// 1 / sqrt(x)
	/// </summary>
	/// <param name="x">�Է°�</param>
	/// <returns>�����</returns>
	inline double ReciprocalSquareRoot(double x)
	{
		return 1.0 / sqrt(x);
	}

	/// <summary>
	/// Round �Լ�
	/// </summary>
	/// <param name="value">�Է°�</param>
	/// <param name="precision">�ݿø�������� �Ҽ��� �ڸ���</param>
	/// <returns>�����</returns>
	inline double Round(double value, int precision)
	{
		int p = Clamp(precision, 0, 15);

		// Lookup table for pwr(10.0,i)
		const static double pwr[] = { 
			1e0,  1e1,  1e2,  1e3,  1e4, 
			1e5,  1e6,  1e7,  1e8,  1e9, 
			1e10, 1e11, 1e12, 1e13, 1e14 };

		// Table of inverses
		const static double invpwr[] = {
			1e0,   1e-1,  1e-2,  1e-3,  1e-4, 
			1e-5,  1e-6,  1e-7,  1e-8,  1e-9, 
			1e-10, 1e-11, 1e-12, 1e-13, 1e-14 };

		if (value<0.0)
			value = ceil(value*pwr[p]-0.5);

		if (value>0.0)
			value = floor(value*pwr[p]+0.5);

		return value*invpwr[p];
	}

	inline double RoundDown(double value, int precision)
	{
		int p = Clamp(precision, 0, 15);

		// Lookup table for pwr(10.0,i)
		const static double pwr[] = { 
			1e0,  1e1,  1e2,  1e3,  1e4, 
			1e5,  1e6,  1e7,  1e8,  1e9, 
			1e10, 1e11, 1e12, 1e13, 1e14 };

		// Table of inverses
		const static double invpwr[] = {
			1e0,   1e-1,  1e-2,  1e-3,  1e-4, 
			1e-5,  1e-6,  1e-7,  1e-8,  1e-9, 
			1e-10, 1e-11, 1e-12, 1e-13, 1e-14 };

		if (value<0.0)
			value = ceil(value*pwr[p]);

		if (value>0.0)
			value = floor(value*pwr[p]);

		return value*invpwr[p];
	}

	inline double RoundUp(double value, int precision = 0)
	{
		int p = Clamp(precision, 0, 15);

		// Lookup table for pwr(10.0,i)
		const static double pwr[] = { 
			1e0,  1e1,  1e2,  1e3,  1e4, 
			1e5,  1e6,  1e7,  1e8,  1e9, 
			1e10, 1e11, 1e12, 1e13, 1e14 };

		// Table of inverses
		const static double invpwr[] = {
			1e0,   1e-1,  1e-2,  1e-3,  1e-4, 
			1e-5,  1e-6,  1e-7,  1e-8,  1e-9, 
			1e-10, 1e-11, 1e-12, 1e-13, 1e-14 };

		if (value<0.0)
			value = ceil(value*pwr[p]-1.0);

		if (value>0.0)
			value = floor(value*pwr[p]+1.0);

		return value*invpwr[p];
	}

	/// <summary>
	/// Round �Լ�
	/// </summary>
	/// <param name="value">�Է°�</param>
	/// <param name="precision">�ݿø�������� �Ҽ��� �ڸ���</param>
	/// <returns>�����</returns>
	inline float Round(float value, int precision = 0)
	{
		return (float) Round(double(value),precision);
	}
	
	inline void RandomSeed(unsigned int seed)
	{
		srand(seed);
	}
	/// <summary>
	/// uniform ���� �Լ�
	/// </summary>
	/// <returns>�����</returns>
	inline double UniformRandom()
	{
		return rand() / ((double) RAND_MAX + 1);
	}

	/// <summary>
	/// 구간 랜덤값을 만드는 함수
	/// 
	/// @code
	/// // 예제 - 랜덤값 생성
	/// 
	/// float v = RangeRandom(0.0, 1.0); // 0 ~ 1 사이 랜덤값 생성
	///
	/// @endcode
	/// </summary>
	/// <param name="fMin">최소값</param>
	/// <param name="fMax">최대값</param>
	/// <returns>랜덤값</returns>
	inline float RangeRandom(float fMin, float fMax)
	{
		float fUnit = float(rand( )) / float(RAND_MAX);
		float fDiff = fMax - fMin;

		return fMin + fUnit * fDiff;
	}

	Vector3 SphericalToCartesian(double latitude, double longitude, double height);
	/// <summary>
	/// Cartesian��ǥ���� Spherical��ǥ�� �ٲ��ش�.
	/// </summary>
	/// <param name="x">�Է°�</param>
	/// <param name="y">���е�</param>
	/// <param name="z">���е�</param>
	/// <returns>Spherical��ǥ ��</returns>
	Vector3 CartesianToSpherical(double x, double y, double z);

}