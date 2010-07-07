/***
 * sim
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
 *
 *  This file is part of sim.
 *
 *  sim is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation; either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  sim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

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
