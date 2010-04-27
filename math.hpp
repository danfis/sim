#ifndef _SIM_MATH_HPP_
#define _SIM_MATH_HPP_

#include <LinearMath/btVector3.h>
#include <LinearMath/btQuaternion.h>

/***
 * sim uses right handed coordinate system: Y+ up, X+ right and Z+ from
 * screen.
 * This coordinate system uses Bullet and thus Bullet's geom primitives can
 * be used.
 */
namespace sim {

typedef btScalar Scalar;

typedef btVector3 Vec3;
typedef btQuaternion Quat;

} /* namespace sim */

#endif /* _SIM_MATH_HPP_ */
