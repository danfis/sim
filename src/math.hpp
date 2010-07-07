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

#ifndef _SIM_MATH_HPP_
#define _SIM_MATH_HPP_

#include <osg/Vec3>
#include <osg/Quat>

#define SIM_CUBE(x) ((x) * (x))

/***
 * sim uses right handed coordinate system: Y+ up, X+ right and Z+ from
 * screen.
 * This coordinate system uses Bullet and thus Bullet's geom primitives can
 * be used.
 */
/**
 * sim uses left handed coordinate system: X+ right, Y+ into screen, Z+ up.
 * This coordinate system is default for Openscenegraph so we can use
 * geom primitives from there.
 */
namespace sim {

typedef double Scalar;

class Vec3 : public osg::Vec3d {
  public:
    Vec3() : osg::Vec3d() {}
    Vec3(Scalar x, Scalar y, Scalar z)
        : osg::Vec3d(x, y, z){}
    Vec3(const osg::Vec3d &v) : osg::Vec3d(v){}

    osg::Vec3d &toOsg() { return (osg::Vec3d &)*this; }
    const osg::Vec3d &toOsg() const { return (const osg::Vec3d &)*this; }
};

class Quat : public osg::Quat {
  public:
    Quat() : osg::Quat() {}
    Quat(Scalar x, Scalar y, Scalar z, Scalar w)
        : osg::Quat(x, y, z, w) {}
    Quat(const Vec3 &axis, Scalar angle)
        : osg::Quat(angle, axis) {}
    Quat(const osg::Quat &q) : osg::Quat(q){}
};


// TODO: parametrize epsilon
inline bool isZero(float f) { return fabs(f) < 1E-6; }
inline bool isZero(double f) { return fabs(f) < 1E-10; }
inline bool eq(float a, float b) { return isZero(fabsf(a - b)); }
inline bool eq(Scalar a, Scalar b) { return isZero(fabs(a - b)); }

} /* namespace sim */

#endif /* _SIM_MATH_HPP_ */
