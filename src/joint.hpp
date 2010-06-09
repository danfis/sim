/***
 * sim
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
 *
 *  This file is part of sim.
 *
 *  sim is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation; either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  sim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

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
     * Default values are -infinity, infinity
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

} /* namespace sim */

#endif /* _SIM_JOINT_HPP_ */
