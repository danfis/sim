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
    Vec3 _anchor;
    Vec3 _axis1;
    Vec3 _axis2;

  public:
    JointHinge2(World *w, Body *oA, Body *oB, const Vec3 &anchor, const Vec3 &axis1, const Vec3 &axis2);

    void activate();

    virtual bool setParamLimitLoHi(double lo, double hi);
    virtual void paramLimitLoHi(double *lo, double *hi) const;
    virtual bool setParamVel2(double vel);
    virtual double paramVel2() const;
    virtual bool setParamFMax2(double fmax);
    virtual double paramFMax2() const;
    virtual bool setParamBounce(double restitution);
    virtual double paramBounce() const;
};

/**
 * ODE's hinge joint.
 * For more info see:
 *  http://opende.sourceforge.net/wiki/index.php/Manual_(All)#Hinge
 */
class JointHinge : public Joint {
    Vec3 _anchor;
    Vec3 _axis;

  public:
    JointHinge(World *w, Body *oA, Body *oB, const Vec3 &anchor, const Vec3 &axis);

    void activate();

    virtual bool setParamLimitLoHi(double lo, double hi);
    virtual void paramLimitLoHi(double *lo, double *hi) const;
    virtual bool setParamVel(double vel);
    virtual double paramVel() const;
    virtual bool setParamFMax(double fmax);
    virtual double paramFMax() const;
    virtual bool setParamBounce(double restitution);
    virtual double paramBounce() const;
};

} /* namespace ode */

} /* namespace sim */

#endif /* _SIM_ODE_JOINT_HPP_ */
