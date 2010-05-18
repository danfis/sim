#ifndef _SIM_JOINT_HPP_
#define _SIM_JOINT_HPP_

#include "body.hpp"

namespace sim {

/**
 * Class representing joint (ODE-like).
 */
class Joint {
  protected:
    Body *_o_a, *_o_b; //< First and second body connect by joint

  public:
    Joint(Body *oA, Body *oB);
    virtual ~Joint();

    /* \{ */
    Body *objA() { return _o_a; }
    const Body *objA() const { return _o_a; }
    Body *objB() { return _o_b; }
    const Body *objB() const { return _o_b; }
    /* \} */

    /* \{ */
    /**
     * Activates joint in world.
     */
    virtual void activate() = 0;

    /**
     * Deactivates (removes) joint in world.
     */
    virtual void deactivate() = 0;
    /* \} */

    /* \{ */
    /**
     * Low and high stop angle or position.
     * Default values are -dInfinity, dInfinity
     */
    virtual bool setParamLimitLoHi(double lo, double hi) { return false; }
    virtual bool setParamLimitLoHi2(double lo, double hi) { return false; }
    virtual void paramLimitLoHi(double *lo, double *hi) const
        { *lo = *hi = -1.; }
    virtual void paramLimitLoHi2(double *lo, double *hi) const
        { *lo = *hi = -1.; }

    /**
     * Set desired motor velocity (angular or linear).
     */
    virtual bool setParamVel(double vel) { return false; }
    virtual double paramVel() const { return -1.; }
    virtual bool setParamVel2(double vel) { return false; }
    virtual double paramVel2() const { return -1.; }

    /**
     * The maximum force or torque that the motor will use to achieve the
     * desired velocity. This must always be greater than or equal to zero.
     * Setting this to zero (the default value) turns off the motor.
     */
    virtual bool setParamFMax(double fmax) { return false; }
    virtual double paramFMax() const { return -1.; }
    virtual bool setParamFMax2(double fmax) { return false; }
    virtual double paramFMax2() const { return -1.; }

    /**
     * The bouncyness of the stops. This is a restitution parameter in the
     * range 0..1. 0 means the stops are not bouncy at all, 1 means maximum
     * bouncyness.
     */
    virtual bool setParamBounce(double restitution) { return false; }
    virtual double paramBounce() const { return -1.; }
    /* \} */
};

/**
 * ODE's hinge2 joint interface.
 * For more info see:
 *  http://opende.sourceforge.net/wiki/index.php/Manual_(All)#Hinge-2
 */
class JointHinge2 {
  public:
    virtual void setLimitLinAxis1(Scalar from, Scalar to) = 0;
    virtual void setLimitLinAxis2(Scalar from, Scalar to) = 0;
    virtual void setLimitAngAxis1(Scalar from, Scalar to) = 0;
    virtual void setLimitAngAxis2(Scalar from, Scalar to) = 0;
};

} /* namespace sim */

#endif /* _SIM_JOINT_HPP_ */
