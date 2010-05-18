#ifndef _SIM_BULLET_JOINT_HPP_
#define _SIM_BULLET_JOINT_HPP_

#include <BulletDynamics/ConstraintSolver/btTypedConstraint.h>

#include "sim/joint.hpp"
#include "sim/bullet/body.hpp"

namespace sim {

namespace bullet {

// Forward declaration
class World;

/**
 * Class representing joint (ODE-like).
 */
class Joint : public sim::Joint {
  protected:
    World *_world;
    btTypedConstraint *_joint;

  public:
    Joint(World *w, Body *oA, Body *oB);
    ~Joint();

    btTypedConstraint *joint() { return _joint; }
    const btTypedConstraint *joint() const { return _joint; }

    /**
     * Activates joint in world.
     */
    void activate();

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

    virtual bool setParamLimitLoHi(double lo, double hi);
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

} /* namespace bullet */

} /* namespace sim */

#endif /* _SIM_BULLET_JOINT_HPP_ */
