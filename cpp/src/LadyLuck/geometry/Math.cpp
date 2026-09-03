#include "LadyLuck/geometry/Math.h"
#include "LadyLuck/geometry/Vector3.h"
#include "LadyLuck/geometry/Matrix3.h"
#include "LadyLuck/geometry/Matrix4.h"
#include "LadyLuck/geometry/Quaternion.h"

namespace BT_Geometry
{
Vector3 SphericalToCartesian(double latitude, double longitude, double radius)
{
	double radCosLat = radius * cos(latitude);

	return Vector3(
		radCosLat * cos(longitude),
		radCosLat * sin(longitude),
		radius * sin(latitude));
}

Vector3 CartesianToSpherical(double x, double y, double z)
{
	double radius = sqrt(x * x + y * y + z * z);
	double longitude = atan2(y, x);
	double latitude = asin(z / radius);

	return Vector3(latitude, longitude, radius);
}
}
