#ifndef _SIM_ODE_JOINT_HPP_
#define _SIM_ODE_JOINT_HPP_

#include "sim/joint.hpp"
#include "sim/ode/body.hpp"

namespace sim {

namespace ode {

// Forward declaration
class World;

/**
 * Class representing joint (ODE-like).
 */
class Joint : public sim::Joint {
  protected:
    World *_world;
    dJointID _joint;

  public:
    Joint(World *w, Body *oA, Body *oB);
    ~Joint();

    dJointID joint() { return _joint; }
    const dJointID joint() const { return _joint; }

    /**
     * Activates joint in world.
     */
    virtual void activate();

    /**
     * Deactivates (removes) joint in world.
     */
    virtual void deactivate();

    // TODO: param vel, bounce, ...
  protected:
    void _setJoint(dJointID j);
};

/**
 * ODE's fixed joint.
 * For more info see:
 *  http://opende.sourceforge.net/wiki/index.php/Manual_(All)#Fixed
 */
class JointFixed : public Joint {
  public:
    JointFixed(World *w, Body *oA, Body *oB);
    void activate();
};

/**
 * ODE's hinge2 joint.
 * For more info see:
 *  http://opende.sourceforge.net/wiki/index.php/Manual_(All)#Hinge-2
 */
class JointHinge2 : public Joint {
  public:
    JointHinge2(World *w, Body *oA, Body *oB, const Vec3 &anchor, const Vec3 &axis1, const Vec3 &axis2);

    void setLimitAngAxis1(Scalar from, Scalar to);
};

/**
 * ODE's hinge joint.
 * For more info see:
 *  http://opende.sourceforge.net/wiki/index.php/Manual_(All)#Hinge
 */
class JointHinge : public Joint {
  public:
    JointHinge(World *w, Body *oA, Body *oB, const Vec3 &anchor, const Vec3 &axis);

    void setLimitAngAxis(Scalar from, Scalar to);
};

} /* namespace ode */

} /* namespace sim */

#endif /* _SIM_ODE_JOINT_HPP_ */
