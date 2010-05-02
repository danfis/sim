#ifndef _SIM_JOINT_HPP_
#define _SIM_JOINT_HPP_

#include <BulletDynamics/ConstraintSolver/btTypedConstraint.h>

#include "body.hpp"

namespace sim {

class Joint {
  protected:
    Body *_o_a, *_o_b;
    btTypedConstraint *_joint;

  public:
    Joint(Body *oA, Body *oB);
    ~Joint();

    Body *objA() { return _o_a; }
    const Body *objA() const { return _o_a; }
    Body *objB() { return _o_b; }
    const Body *objB() const { return _o_b; }
    btTypedConstraint *joint() { return _joint; }
    const btTypedConstraint *joint() const { return _joint; }

  protected:
    void _setJoint(btTypedConstraint *c) { _joint = c; }
};

class JointFixed : public Joint {
  public:
    JointFixed(Body *oA, Body *oB);
};

class JointPoint2Point : public Joint {
  public:
    JointPoint2Point(Body *oA, Body *oB, const Vec3 &pivotA, const Vec3 &pivotB);
};

class JointHinge : public Joint {
  public:
    JointHinge(Body *oA, Body *oB, const Vec3 &anchor, const Vec3 &axis);
};

class JointHinge2 : public Joint {
  public:
    JointHinge2(Body *oA, Body *oB, const Vec3 &anchor, const Vec3 &axis1, const Vec3 &axis2);
};

} /* namespace sim */

#endif /* _SIM_JOINT_HPP_ */
