#ifndef _SIM_JOINT_HPP_
#define _SIM_JOINT_HPP_

#include <BulletDynamics/ConstraintSolver/btTypedConstraint.h>

#include "body.hpp"

namespace sim {

// Forward declaration
class World;

/**
 * Class representing joint (ODE-like).
 */
class Joint {
  protected:
    World *_world;
    Body *_o_a, *_o_b; //< First and second body connect by joint
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

    /**
     * Activates joint in world.
     */
    virtual void activate();

    /**
     * Deactivates (removes) joint in world.
     */
    void deactivate();

  protected:
    void _setJoint(btTypedConstraint *c) { _joint = c; }
};

/**
 * ODE's fixed joint.
 * For more info see:
 *  http://opende.sourceforge.net/wiki/index.php/Manual_(All)#Fixed
 */
class JointFixed : public Joint {
  public:
    JointFixed(World *w, Body *oA, Body *oB);
};

/**
 * ODE's hinge2 joint.
 * For more info see:
 *  http://opende.sourceforge.net/wiki/index.php/Manual_(All)#Hinge-2
 */
class JointHinge2 : public Joint {
  public:
    JointHinge2(World *w, Body *oA, Body *oB, const Vec3 &anchor, const Vec3 &axis1, const Vec3 &axis2);

    void setLimitLinAxis1(Scalar from, Scalar to);
    void setLimitLinAxis2(Scalar from, Scalar to);
    void setLimitAngAxis1(Scalar from, Scalar to);
    void setLimitAngAxis2(Scalar from, Scalar to);
};

/**
 * ODE's hinge joint.
 * For more info see:
 *  http://opende.sourceforge.net/wiki/index.php/Manual_(All)#Hinge
 */
class JointHinge : public Joint {
  public:
    JointHinge(World *w, Body *oA, Body *oB, const Vec3 &anchor, const Vec3 &axis);
};

class JointPoint2Point : public Joint {
  public:
    JointPoint2Point(World *w, Body *oA, Body *oB, const Vec3 &pivotA, const Vec3 &pivotB);
};


} /* namespace sim */

#endif /* _SIM_JOINT_HPP_ */
