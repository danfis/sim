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

    /**
     * Enables joint and connected bodies.
     */
    void _enable();

    /**
     * Returns true if joint is enabled.
     */
    bool _enabled() const;
};

/**
 * ODE's fixed joint.
 * For more info see:
 *  http://opende.sourceforge.net/wiki/index.php/Manual_(All)#Fixed
 */
class JointFixed : public Joint {
  public:
    JointFixed(World *w, Body *oA, Body *oB);
    bool isFixed() const { return true; }
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

    bool isHinge2() const { return true; }

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
    Scalar _axis_offset;

  public:
    JointHinge(World *w, Body *oA, Body *oB, const Vec3 &anchor,
               const Vec3 &axis, Scalar axis_offset = 0.);

    bool isHinge() const { return true; }

    void activate();

    /**
     * Returns angle position of hinge (-pi..pi)
     */
    Scalar angle() const;

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
