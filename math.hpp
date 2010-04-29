#ifndef _SIM_MATH_HPP_
#define _SIM_MATH_HPP_

#include <LinearMath/btVector3.h>
#include <LinearMath/btQuaternion.h>

#include <osg/Vec3>
#include <osg/Quat>

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

    btVector3 toBullet() const { return btVector3(x(), z(), y()); }

    static btVector3 toBullet(Scalar x, Scalar y, Scalar z)
        { return btVector3(x, z, y); }
    static Vec3 fromBullet(const btVector3 &v)
        { return Vec3(v.x(), v.z(), v.y()); }
    

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

    btQuaternion toBullet() const { return btQuaternion(x(), z(), y(), -w()); }

    static Quat fromBullet(const btQuaternion &q)
        { return Quat(q.x(), q.z(), q.y(), -q.w()); }
};

} /* namespace sim */

#endif /* _SIM_MATH_HPP_ */
