#pragma once


#include <math.h>

namespace BT_Geometry
{
	class Vector3;
	class EulerAngle;
	class Matrix3;
	class Quaternion;

	class CoordinateConverter 
	{
	public :
		CoordinateConverter();
		virtual Vector3 LLAtoCartesian(double latitude, double longitude, double altitude);
		virtual Vector3 LLAtoCartesian(Vector3 const& lla);
		virtual Vector3 CartesianToLLA(Vector3 const& cartesian);

		virtual Vector3 GeodeticLLAtoCartesian(double latitude, double longitude, double altitude);
		virtual Vector3 CartesianToGeodeticLLA(double x, double y, double z);
		
		virtual EulerAngle CounterAngleDirection(double psi, double theta, double phi);
		virtual EulerAngle CounterAngleDirection(EulerAngle const& angle);

		virtual Matrix3 LocalNEDMatrix(Vector3 const& position);
		virtual Matrix3 LocalNEDMatrixInverse(Vector3 const& position);

		virtual Quaternion NEDtoRTFOrientation(Vector3 const& position, Quaternion const& attitude);
		virtual Quaternion NEDtoRTFOrientation(Vector3 const& position, EulerAngle const& attitude);
		virtual Matrix3 NEDtoRTFOrientation(Vector3 const& position, Matrix3 const& attitude);

		virtual Quaternion RTFtoNEDOrientation(Vector3 const& position, Quaternion const& attitude);
		virtual Quaternion RTFtoNEDOrientation(Vector3 const& position, EulerAngle const& attitude);
		virtual Matrix3 RTFtoNEDOrientation(Vector3 const& position, Matrix3 const& attitude);

		virtual Vector3 NEDtoRTFDirection(Vector3 const& position, Vector3 const& direction);
		virtual Vector3 RTFtoNEDDirection(Vector3 const& position, Vector3 const& direction);

		virtual double RTFtoAzimuth(Vector3 const& position, Quaternion const& attitude);
		virtual double RTFtoAzimuth(Vector3 const& position, Vector3 const& direction);

		virtual double RTFtoPitch(Vector3 const& position, Quaternion const& attitude);
		virtual double RTFtoPitch(Vector3 const& position, Vector3 const& direction);

		virtual Quaternion AzimuthToRTFOrientation(Vector3 const& position, double azimuth);

		virtual Vector3 NorthDirection(Vector3 const& CartesianPos);
		virtual Vector3 AzimuthDirection(Vector3 const& position, Quaternion const& attitude);

		Vector3 LLA2FLATNED(double lat, double lon, double alt, double lat_o, double lon_o, double alt_Ref);

		Vector3 ToECEFCartesianPosition(Vector3 LocalCartesianPos);
		EulerAngle ToECEFOrientation(EulerAngle LocalOri, Vector3 ECEFCartesianPosition);

	private :
		
		Quaternion CounterAngleDirection(Quaternion const& angleangle);

		//WGS84 Earth Constants
		const double wgs84a ,wgs84f, wgs84b;
		const double EARTH_A, EARTH_B, EARTH_F, EARTH_Esq, EARTH_Ecc;
		const double dtr;
		const double POLAR_RADIUS, EQUATORIAL_RADIUS;
		
		double rearth(double lati);
		Vector3 radcur(double lati);
		double gc2gd(double latigc,double alti);

	};
}

