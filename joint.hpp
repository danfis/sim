#ifndef _SIM_JOINT_HPP_
#define _SIM_JOINT_HPP_

#include <BulletDynamics/ConstraintSolver/btTypedConstraint.h>

#include "obj.hpp"

namespace sim {

class Joint {
  protected:
    Obj *_o_a, *_o_b;
    btTypedConstraint *_joint;

  public:
    Joint(Obj *oA, Obj *oB);
    ~Joint();

    Obj *objA() { return _o_a; }
    const Obj *objA() const { return _o_a; }
    Obj *objB() { return _o_b; }
    const Obj *objB() const { return _o_b; }
    btTypedConstraint *joint() { return _joint; }
    const btTypedConstraint *joint() const { return _joint; }

  protected:
    void _setJoint(btTypedConstraint *c) { _joint = c; }
};

class JointHinge2 : public Joint {
  public:
    JointHinge2(Obj *oA, Obj *oB, const Vec3 &anchor, const Vec3 &axis1, const Vec3 &axis2);
};

} /* namespace sim */

#endif /* _SIM_JOINT_HPP_ */
