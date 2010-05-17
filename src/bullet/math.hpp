#ifndef _SIM_BULLET_MATH_HPP_
#define _SIM_BULLET_MATH_HPP_

#include <LinearMath/btVector3.h>
#include <LinearMath/btQuaternion.h>
#include <sim/math.hpp>

namespace sim {

namespace bullet {

typedef double Scalar;

inline btVector3 vToBt(Scalar x, Scalar y, Scalar z)
    { return btVector3(x, z, y); }
inline btVector3 vToBt(const Vec3 &v)
    { return btVector3(v.x(), v.z(), v.y()); }
inline Vec3 vFromBt(const btVector3 &v)
    { return Vec3(v.x(), v.z(), v.y()); }

inline btQuaternion qToBt(const Quat &q)
    { return btQuaternion(q.x(), q.z(), q.y(), -q.w()); }
inline Quat qFromBt(const btQuaternion &q)
    { return Quat(q.x(), q.z(), q.y(), -q.w()); }


} /* namespace bullet */

} /* namespace sim */

#endif /* _SIM_BULLET_MATH_HPP_ */
