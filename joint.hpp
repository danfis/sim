#ifndef _SIM_JOINT_HPP_
#define _SIM_JOINT_HPP_

#include <BulletDynamics/ConstraintSolver/btTypedConstraint.h>

#include "body.hpp"

namespace sim {

/** Forward declaration */
class World;

class Joint {
  protected:
    World *_world;

    Body *_o_a, *_o_b;
    btTypedConstraint *_joint;

  public:
    Joint(World *w, Body *oA, Body *oB);
    ~Joint();

    Body *objA() { return _o_a; }
    const Body *objA() const { return _o_a; }
    Body *objB() { return _o_b; }
    const Body *objB() const { return _o_b; }
    btTypedConstraint *joint() { return _joint; }
    const btTypedConstraint *joint() const { return _joint; }

    void activate();
    void deactivate();

  protected:
    void _setJoint(btTypedConstraint *c) { _joint = c; }
};

class JointFixed : public Joint {
  public:
    JointFixed(World *w, Body *oA, Body *oB);
};

class JointPoint2Point : public Joint {
  public:
    JointPoint2Point(World *w, Body *oA, Body *oB, const Vec3 &pivotA, const Vec3 &pivotB);
};

class JointHinge : public Joint {
  public:
    JointHinge(World *w, Body *oA, Body *oB, const Vec3 &anchor, const Vec3 &axis);
};

class JointHinge2 : public Joint {
  public:
    JointHinge2(World *w, Body *oA, Body *oB, const Vec3 &anchor, const Vec3 &axis1, const Vec3 &axis2);
};

} /* namespace sim */

#endif /* _SIM_JOINT_HPP_ */
