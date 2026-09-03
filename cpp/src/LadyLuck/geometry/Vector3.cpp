#include "LadyLuck/geometry/Vector3.h"
#include "LadyLuck/geometry/Matrix4.h"
#include "LadyLuck/geometry/Vector4.h"
#include "LadyLuck/geometry/Quaternion.h"
namespace BT_Geometry
{
    Vector3 Vector3::_Zero = Vector3(0.0, 0.0, 0.0);
    Vector3 Vector3::_One = Vector3(1.0, 1.0, 1.0);
    Vector3 Vector3::_Right = Vector3(1.0, 0.0, 0.0);
    Vector3 Vector3::_Up = Vector3(0.0, 1.0, 0.0);
    Vector3 Vector3::_Forward = Vector3(0.0, 0.0, -1.0);
}
