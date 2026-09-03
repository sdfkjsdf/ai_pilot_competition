// CoordinateConverter.cpp : DLL ���� ���α׷��� ���� ������ �Լ��� �����մϴ�.
//


#include "LadyLuck/geometry/CoordinateConverter.h"
#include "LadyLuck/geometry/Vector3.h"
#include "LadyLuck/geometry/Math.h"
#include "LadyLuck/geometry/EulerAngle.h"
#include "LadyLuck/geometry/Matrix3.h"
#include "LadyLuck/geometry/Quaternion.h"

namespace BT_Geometry
{
CoordinateConverter::CoordinateConverter() 
	: wgs84a ( 6378137 ),
	  wgs84f ( 1.0 / 298.257223563 ),
      wgs84b ( wgs84a * (1.0 - wgs84f) ),

      EARTH_A ( wgs84a ),
      EARTH_B ( wgs84b ),
      EARTH_F ( wgs84f ),
      EARTH_Esq ( 1 - wgs84b * wgs84b / (wgs84a * wgs84a) ),
	  //const double EARTH_Ecc = sqrt(EARTH_Esq);
	  EARTH_Ecc ( 0.08181919084262157 ),

      //const double dtr = Math.PI / 180.0;
	  dtr ( 3.14159265358979323846 / 180.0 ),
	  POLAR_RADIUS(6356752.3142),
	  EQUATORIAL_RADIUS(6378137.0)
{
	 
}

Vector3 CoordinateConverter::LLAtoCartesian(double latitude, double longitude, double altitude)
{
	return SphericalToCartesian(latitude*DEGTORAD, longitude*DEGTORAD, altitude+EARTH_RADIUS);
}

Vector3 CoordinateConverter::LLAtoCartesian(Vector3 const& lla)
{
	return SphericalToCartesian(lla.X*DEGTORAD, lla.Y*DEGTORAD, lla.Z+EARTH_RADIUS);
}

Vector3 CoordinateConverter::CartesianToLLA(Vector3 const& cartesian)
{
	Vector3 tmp = CartesianToSpherical(cartesian.X, cartesian.Y, cartesian.Z);
	tmp.X *= RADTODEG;
	tmp.Y *= RADTODEG;
	tmp.Z -= EARTH_RADIUS;

	return tmp;
}


Vector3 CoordinateConverter::GeodeticLLAtoCartesian(double latitude, double longitude, double altitude)
{
	//latitude, longitude, xyz(ECEF)            
	double clati = cos(dtr * latitude);
	double slati = sin(dtr * latitude);
	double clongi = cos(dtr * longitude);
	double slongi = sin(dtr * longitude);

	Vector3 rrnrm = radcur(latitude);
	double rn = rrnrm.Y;
	double re = rrnrm.X;

	double ecc = EARTH_Ecc;
	double esq = ecc * ecc;

	Vector3 ret;
	ret.X =  (rn + altitude) * clati * clongi;
	ret.Y =  (rn + altitude) * clati * slongi;
	ret.Z =  ( (1-esq)*rn + altitude) * slati;

	return ret;
}

Vector3 CoordinateConverter::CartesianToGeodeticLLA(double x, double y, double z)
{
	double esq = EARTH_Esq;
	double rp = sqrt(x * x + y * y + z * z);
	double latgc = asin(z / rp) / dtr;

	double testVal = fabs(x) + fabs(y);
	double loni, lati, alti;
	if (testVal < 1.0e-10)
	{
		loni = 0.0;
	}
	else
	{
		loni = atan2(y, x) / dtr;
	}
	if (loni < 0.0)
	{
		loni += 360.0;
	}
            
            
	double p = sqrt(x * x + y * y);
	if (p < 1.0e-10)
	{
		lati = 90.0;
		if (z < 0.0)
		{
			lati = -90.0;
		}
		alti = rp - rearth(lati);
		Vector3 llh;
		llh.X=lati;
		llh.Y=loni;
		llh.Z=alti;
		return llh;
	}

	double rnow = rearth(latgc);
	alti = rp - rnow;
	lati = gc2gd(latgc, alti);

	Vector3 rrnrm = radcur(lati);
	double rn = rrnrm.Y;
	for (int count = 0; count < 5; count++)
	{
		double slati = sin(dtr * lati);
		double tangd = (z + rn * esq * slati) / p;
		double latin = atan(tangd) / dtr;

		double dlati = latin - lati;
		double flati = latin;
		double clati = cos(dtr * lati);
		Vector3 rrnrmList = radcur(lati);
		rn = rrnrmList.Y;
		alti = (p / clati) - rn;
		if (fabs(dlati) < 1.0e-12)
		{
			break;
		}
	}

	Vector3 retLlh;
	retLlh.X = lati;
	retLlh.Y = loni;
	retLlh.Z = alti;
	return retLlh;
}




double CoordinateConverter::rearth(double lati)
{
	Vector3 rrnrm = radcur(lati);
	return rrnrm.X;
}

Vector3 CoordinateConverter::radcur(double latitude)
{	
	double asq = EARTH_A * EARTH_A;
	double bsq = EARTH_B * EARTH_B;
	double eccsq =  1 - bsq/asq;
	double ecc = sqrt(eccsq);
	double clati = cos(dtr*latitude);
	double slati = sin(dtr*latitude);
	double dsq = 1.0 - eccsq * slati * slati;
	double d = sqrt(dsq);

	double rn = EARTH_A/d;
	double rm = rn *(1.0-eccsq) /dsq;
	double rho = rn *clati;
	double z = (1.0-eccsq) * rn *slati;
	double rsq = rho*rho+z*z;
	double r  = sqrt(rsq);

	Vector3 ret;
	ret.X =r;
	ret.Y = rn;
	ret.Z=rm;

	return ret;
}

double CoordinateConverter::gc2gd(double latigc,double alti)
{
    double rtd = 1/dtr;
    double ecc = EARTH_Ecc;
    double esq = ecc * ecc;
    double altiNow = alti;
    Vector3 rrnrm = radcur(latigc);
	double rn = rrnrm.Y;
    double ratio = 1 - esq * rn / (rn + altiNow);
    double tlati = tan(dtr * latigc) / ratio;
    double latigd = rtd * atan(tlati);
    return latigd;
}


EulerAngle CoordinateConverter::CounterAngleDirection(double psi, double theta, double phi)
{
	double yaw = 360 - psi;
	double pitch = theta;
	double roll = -phi;

	if(yaw == 360)
		yaw = 0;

	EulerAngle angle(yaw * DEGTORAD, pitch * DEGTORAD, roll * DEGTORAD);

	return angle;
}

EulerAngle CoordinateConverter::CounterAngleDirection(EulerAngle const& angle)
{
	double yaw = 360*DEGTORAD - angle.Yaw;
	double pitch = angle.Pitch;
	double roll = -angle.Roll;

	if (yaw == 360*DEGTORAD)
		yaw = 0;

	EulerAngle ang(yaw, pitch, roll);

	return ang;
}

Quaternion CoordinateConverter::CounterAngleDirection(Quaternion const& angle)
{
	return CounterAngleDirection(angle.toEulerAngle()).toQuaternion();
}

Matrix3 CoordinateConverter::LocalNEDMatrix(Vector3 const& position)
{
	Vector3 north = Vector3(0, 0, EARTH_RADIUS) - position;
	north.normalize();

	Vector3 up = position;
	up.normalize();

	Vector3 right = north.cross(up);
	right.normalize();

	north = up.cross(right);
	north.normalize();

	Matrix3 adjustM;
	adjustM.setXAxis(right);
	adjustM.setYAxis(up);
	adjustM.setZAxis(-north);

	return adjustM;
}

Matrix3 CoordinateConverter::LocalNEDMatrixInverse(Vector3 const& position)
{
	return LocalNEDMatrix(position).inverse();
}

Quaternion CoordinateConverter::NEDtoRTFOrientation(Vector3 const& position, Quaternion const& attitude)
{
	Matrix3 adjustM = LocalNEDMatrix(position);
	Matrix3 tmpM = adjustM * CounterAngleDirection(attitude).toMatrix();
	return tmpM.toQuaternion();
}

Quaternion CoordinateConverter::NEDtoRTFOrientation(Vector3 const& position, EulerAngle const& attitude)
{
	Matrix3 adjustM = LocalNEDMatrix(position);
	Matrix3 tmpM = adjustM * CounterAngleDirection(attitude).toMatrix();
	return tmpM.toQuaternion();
}

Matrix3 CoordinateConverter::NEDtoRTFOrientation(Vector3 const& position, Matrix3 const& attitude)
{
	Matrix3 adjustM = LocalNEDMatrix(position);
	Matrix3 tmpM = adjustM * CounterAngleDirection(attitude.toEulerAngle()).toMatrix();
	return tmpM;
}

Quaternion CoordinateConverter::RTFtoNEDOrientation(Vector3 const& position, Quaternion const& attitude)
{
	Matrix3 adjustM = LocalNEDMatrixInverse(position);
	Matrix3 tmpM = adjustM * attitude.toMatrix();
	EulerAngle e = tmpM.toEulerAngle();
	return CounterAngleDirection(tmpM.toQuaternion());
}

Quaternion CoordinateConverter::RTFtoNEDOrientation(Vector3 const& position, EulerAngle const& attitude)
{
	Matrix3 adjustM = LocalNEDMatrixInverse(position);
	Matrix3 tmpM = adjustM * attitude.toMatrix();
	return CounterAngleDirection(tmpM.toQuaternion());
}

Matrix3 CoordinateConverter::RTFtoNEDOrientation(Vector3 const& position, Matrix3 const& attitude)
{
	Matrix3 adjustM = LocalNEDMatrixInverse(position);
	Matrix3 tmpM = adjustM * attitude;
	return CounterAngleDirection(tmpM.toQuaternion()).toMatrix();
}

Vector3 CoordinateConverter::NEDtoRTFDirection(Vector3 const& position, Vector3 const& direction)
{
	Vector3 tmp = LocalNEDMatrix(position) * direction;
	return tmp;
}

Vector3 CoordinateConverter::RTFtoNEDDirection(Vector3 const& position, Vector3 const& direction)
{
	Vector3 tmp = LocalNEDMatrixInverse(position) * direction;
	return tmp;
}

double CoordinateConverter::RTFtoAzimuth(Vector3 const& position, Quaternion const& attitude)
{
	Vector3 tmpDir = attitude * Vector3(0,0,-1);
	return RTFtoAzimuth(position, tmpDir); 
}

double CoordinateConverter::RTFtoAzimuth(Vector3 const& position, Vector3 const& direction)
{
	Vector3 north = Vector3(0, 0, EARTH_RADIUS) - position;
	Vector3 up = position;
	up.normalize();
	Vector3 right = north.cross(up);
	north = up.cross(right);
	north.normalize();

	Vector3 tmpDown = -position;
	Vector3 tmpRight = tmpDown.cross(direction);
	Vector3 tmpDir = tmpRight.cross(tmpDown);
	tmpDir.normalize();
	double tmpAzi = north.angleBetween(tmpDir);

	Vector3 tmpCross = tmpDir.cross(north);
	tmpCross.normalize();

	if (up.dot(tmpCross) < 0.9)
		tmpAzi = 2*PI - tmpAzi;

	return tmpAzi;
}

double CoordinateConverter::RTFtoPitch(Vector3 const& position, Quaternion const& attitude)
{
	return RTFtoNEDOrientation(position, attitude).toEulerAngle().Pitch; 
}

double CoordinateConverter::RTFtoPitch(Vector3 const& position, Vector3 const& direction)
{
	Vector3 north = Vector3(0, 0, EARTH_RADIUS) - position;
	Vector3 up = position;
	up.normalize();
	Vector3 right = north.cross(up);
	north = up.cross(right);
	north.normalize();

	Vector3 tmpDown = -position;
	Vector3 tmpRight = tmpDown.cross(direction);
	Vector3 tmpDir = tmpRight.cross(tmpDown);
	tmpDir.normalize();
	double tmpPitch = direction.angleBetween(tmpDir);

	Vector3 tmpCross = tmpDir.cross(direction);
	tmpCross.normalize();

	if (right.dot(tmpCross) < 0.9)
		tmpPitch = 2*PI - tmpPitch;

	return tmpPitch;
}

Quaternion CoordinateConverter::AzimuthToRTFOrientation(Vector3 const& position, double azimuth)
{
	EulerAngle tmpAngle(azimuth, 0, 0);
	return NEDtoRTFOrientation(position, tmpAngle);
}

Vector3 CoordinateConverter::NorthDirection(Vector3 const& CartesianPos)
{
	Vector3 north = Vector3(0, 0, EARTH_RADIUS) - CartesianPos;
	Vector3 up = CartesianPos;
	Vector3 right = north.cross(up);
	north = up.cross(right);
	north.normalize();

	return north;
}

Vector3 CoordinateConverter::AzimuthDirection(Vector3 const& position, Quaternion const& attitude)
{
	Quaternion tmpOrientation = AzimuthToRTFOrientation(position, RTFtoAzimuth(position, attitude));
	return tmpOrientation * Vector3(0,0,-1);
}

Vector3 CoordinateConverter::LLA2FLATNED(double lat, double lon, double alt, double lat_o, double lon_o, double alt_Ref)
{
	double eccentricitysquare, N, M;
	eccentricitysquare = 1.0 - pow(POLAR_RADIUS, 2) / pow(EQUATORIAL_RADIUS, 2);
	N = EQUATORIAL_RADIUS / sqrt(1.0 - eccentricitysquare * pow(sin(lat_o * PI / 180.0), 2)); // prime vertical radius of curvature
	M = EQUATORIAL_RADIUS * (1.0 - eccentricitysquare) / pow(1 - eccentricitysquare * pow(sin(lat_o * PI / 180.0), 2), 3 / 2);

	double dlat, dlon;
	dlat = lat - lat_o;
	dlon = lon - lon_o;

	double dN, dE, dD;
	dN = (M + alt_Ref) * dlat * PI / 180.0;
	dE = (N + alt_Ref) * cos(lat_o * PI / 180.0) * dlon * PI / 180.0;
	dD = -(alt - alt_Ref);
	Vector3 res(dN, dE, dD);
	return res;
}

Vector3 CoordinateConverter::ToECEFCartesianPosition(Vector3 LocalCartesianPos)
{
	Vector3 convertedSphericalLLA;
	convertedSphericalLLA = CartesianToSpherical(LocalCartesianPos.X, LocalCartesianPos.Y, LocalCartesianPos.Z);
	convertedSphericalLLA.X *= RADTODEG;
	convertedSphericalLLA.Y *= RADTODEG;
	convertedSphericalLLA.Z -= EARTH_RADIUS;
	Vector3 convertedCartesianPosition = GeodeticLLAtoCartesian(convertedSphericalLLA.X, convertedSphericalLLA.Y, convertedSphericalLLA.Z);
	return convertedCartesianPosition;
}

EulerAngle CoordinateConverter::ToECEFOrientation(EulerAngle LocalOri, Vector3 ECEFCartesianPosition)
{
	EulerAngle ea = LocalOri;//mCoordinateConverter.RTFtoNEDOrientation(AddSIM_Cartesian, AddSIM_EulerAngle).toEulerAngle();
	Vector3 north(0, 0, EARTH_RADIUS);
	north -= ECEFCartesianPosition;
	north.normalize();
	
	Vector3 up = ECEFCartesianPosition;
	up.normalize();

	Vector3 right = north.cross(up);
	right.normalize();

	north = up.cross(right);
	north.normalize();
	
	Vector3 pe = north * cos(ea.Yaw) + right * sin(ea.Yaw);
	Vector3 y_b = -north * sin(ea.Yaw) + right * cos(ea.Yaw);

	Vector3 xb = pe * cos(ea.Pitch) + up * sin(ea.Pitch);
	Vector3 z_b = pe * sin(ea.Pitch) - up * cos(ea.Pitch);

	Vector3 yb = y_b * cos(ea.Roll) + z_b * sin(ea.Roll);
	//REALTIMEVISUAL.Vector3 zb = -y_b * System.Math.Sin(ea.Roll) + z_b * System.Math.Cos(ea.Roll);

	Vector3 xu(1, 0, 0);
	Vector3 yu(0, 1, 0);
	Vector3 zu(0, 0, 1);

	EulerAngle cea;
	Vector3 pd = xb - (xb.dot(zu)) * zu;
	pd.normalize();

	if (pd.length() < DOUBLE_ROUNDING_ERROR)
	{
		cea.Yaw = atan2(-yb.dot(xu), yb.dot(yu));
		cea.Roll = 0.0f;
		cea.Pitch =  asin(-xb.dot(zu));
	}
	else
	{
		cea.Yaw = atan2(pd.dot(yu), pd.dot(xu));
		cea.Pitch = asin(-xb.dot(zu));

		Vector3 y_d = -xu * sin(cea.Yaw) + yu * cos(cea.Yaw);
		Vector3 z_d = pd * sin(cea.Pitch) + zu * cos(cea.Pitch);

		cea.Roll = atan2(yb.dot(z_d), yb.dot(y_d));
	}

	return cea;
}


}
