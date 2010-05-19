#ifndef _SIM_ODE_MATH_HPP_
#define _SIM_ODE_MATH_HPP_

#include <ode/ode.h>
#include <sim/math.hpp>

namespace sim {

namespace ode {

inline Vec3 vFromODE(double x, double y, double z)
    { return Vec3(x, y, z); }

inline Vec3 vFromODE(const dReal *off)
    { return Vec3(off[0], off[1], off[2]); }

inline void qToODE(const sim::Quat &quat, dQuaternion q)
    { q[0] = quat.w(); q[1] = quat.x(); q[2] = quat.y(); q[3] = quat.z(); }

inline Quat qFromODE(const dQuaternion q)
    { return Quat(q[1], q[2], q[3], q[0]); }

} /* namespace ode */

} /* namespace sim */

#endif /* _SIM_ODE_MATH_HPP_ */
