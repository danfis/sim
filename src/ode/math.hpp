#ifndef _SIM_ODE_MATH_HPP_
#define _SIM_ODE_MATH_HPP_

#include <ode/ode.h>
#include <sim/math.hpp>

namespace sim {

namespace ode {

inline void qToODE(const sim::Quat &quat, dQuaternion q)
        { q[0] = quat.w(); q[1] = quat.x(); q[2] = quat.y(); q[3] = quat.z(); }

} /* namespace ode */

} /* namespace sim */

#endif /* _SIM_ODE_MATH_HPP_ */
