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
    void activate();
};

/**
 * ODE's hinge2 joint.
 * For more info see:
 *  http://opende.sourceforge.net/wiki/index.php/Manual_(All)#Hinge-2
 */
class JointHinge2 : public Joint {
    Vec3 _anchor, _axis1, _axis2;
    Scalar _lim[2]; //!< Limit lo, hi

  public:
    JointHinge2(World *w, Body *oA, Body *oB, const Vec3 &anchor, const Vec3 &axis1, const Vec3 &axis2);
    void activate();

    virtual bool setParamLimitLoHi(double lo, double hi);
    void paramLimitLoHi(double *lo, double *hi) const;
};

/**
 * ODE's hinge joint.
 * For more info see:
 *  http://opende.sourceforge.net/wiki/index.php/Manual_(All)#Hinge
 */
class JointHinge : public Joint {
    Vec3 _anchor, _axis;
    Scalar _lim[2]; //!< Limit lo, hi
    Scalar _vel; //!< Target velocity of angular motor
    Scalar _fmax; //!< Max force on angular motor

  public:
    JointHinge(World *w, Body *oA, Body *oB, const Vec3 &anchor, const Vec3 &axis);
    void activate();

    bool setParamLimitLoHi(double lo, double hi);
    void paramLimitLoHi(double *lo, double *hi) const;

    bool setParamVel(double vel);
    double paramVel() const { return _vel; }

    bool setParamFMax(double fmax);
    double paramFMax() const { return _fmax; }

  protected:
    void _applyVelFMax();
};

} /* namespace bullet */

} /* namespace sim */

#endif /* _SIM_BULLET_JOINT_HPP_ */
