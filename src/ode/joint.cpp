/***
 * sim
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
 *
 *  This file is part of sim.
 *
 *  sim is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation; either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  sim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "sim/ode/joint.hpp"
#include "sim/ode/world.hpp"
#include "sim/msg.hpp"

namespace sim {

namespace ode {

Joint::Joint(World *w, Body *oA, Body *oB)
    : sim::Joint(oA, oB), _world(w), _joint(0)
{
}

Joint::~Joint()
{
    if (_joint)
        dJointDestroy(_joint);
}


void Joint::activate()
{
    dBodyID a = 0, b = 0;

    if (objA()){
        a = ((Body *)objA())->body();
    }

    if (objB()){
        b = ((Body *)objB())->body();
    }

    dJointAttach(_joint, a, b);

    _enable();
}

void Joint::deactivate()
{
    dJointDisable(_joint);
}

void Joint::_enable()
{
    dBodyID b;

    dJointEnable(_joint);

    for (size_t i = 0; i < 2; i++){
        b = dJointGetBody(_joint, 0);
        if (b)
            dBodyEnable(b);
    }
}

bool Joint::_enabled() const
{
    return dJointIsEnabled(_joint);
}


void Joint::_setJoint(dJointID j)
{
    _joint = j;

    dJointDisable(_joint);
    dJointSetData(_joint, this);
}

JointFixed::JointFixed(World *w, Body *oA, Body *oB)
    : Joint(w, oA, oB)
{
    dJointID joint;

    joint = dJointCreateFixed(w->world(), 0);

    _setJoint(joint);
}

void JointFixed::activate()
{
    Joint::activate();
    dJointSetFixed(_joint);
}


JointHinge2::JointHinge2(World *w, Body *oA, Body *oB, const Vec3 &anchor, const Vec3 &axis1, const Vec3 &axis2)
    : Joint(w, oA, oB), _anchor(anchor), _axis1(axis1), _axis2(axis2)
{
    dJointID joint;

    joint = dJointCreateHinge2(w->world(), 0);

    _setJoint(joint);
}

void JointHinge2::activate()
{
    Joint::activate();

    dJointSetHinge2Anchor(_joint, _anchor.x(), _anchor.y(), _anchor.z());
    dJointSetHinge2Axis1(_joint, _axis1.x(), _axis1.y(), _axis1.z());
    dJointSetHinge2Axis2(_joint, _axis2.x(), _axis2.y(), _axis2.z());
}

bool JointHinge2::setParamLimitLoHi(double lo, double hi)
{
    dJointSetHinge2Param(_joint, dParamLoStop1, lo);
    dJointSetHinge2Param(_joint, dParamHiStop1, hi);
    return true;
}

void JointHinge2::paramLimitLoHi(double *lo, double *hi) const
{
    *lo = dJointGetHinge2Param(_joint, dParamLoStop1);
    *hi = dJointGetHinge2Param(_joint, dParamHiStop1);
}

bool JointHinge2::setParamVel2(double vel)
{
    dJointSetHinge2Param(_joint, dParamVel2, vel);

    // if velocity is non-zero enable connected bodies
    if (_enabled() && !isZero(vel)){
        _enable();
    }

    return true;
}

double JointHinge2::paramVel2() const
{
    return dJointGetHinge2Param(_joint, dParamVel2);
}

bool JointHinge2::setParamFMax2(double fmax)
{
    dJointSetHinge2Param(_joint, dParamFMax2, fmax);
    return true;
}

double JointHinge2::paramFMax2() const
{
    return dJointGetHinge2Param(_joint, dParamFMax2);
}

bool JointHinge2::setParamBounce(double restitution)
{
    dJointSetHinge2Param(_joint, dParamBounce, restitution);
    return true;
}

double JointHinge2::paramBounce() const
{
    return dJointGetHinge2Param(_joint, dParamBounce);
}



JointHinge::JointHinge(World *w, Body *oA, Body *oB, const Vec3 &anchor,
                       const Vec3 &axis, Scalar axis_offset)
    : Joint(w, oA, oB), _anchor(anchor),
      _axis(axis), _axis_offset(axis_offset)
{
    dJointID joint;

    joint = dJointCreateHinge(w->world(), 0);

    _setJoint(joint);
}

void JointHinge::activate()
{
    Joint::activate();

    dJointSetHingeAnchor(_joint, _anchor.x(), _anchor.y(), _anchor.z());
    dJointSetHingeAxis(_joint, _axis.x(), _axis.y(), _axis.z());
}

Scalar JointHinge::angle() const
{
    return dJointGetHingeAngle(_joint);
}

bool JointHinge::setParamLimitLoHi(double lo, double hi)
{
    dJointSetHingeParam(_joint, dParamLoStop1, lo);
    dJointSetHingeParam(_joint, dParamHiStop1, hi);
    return true;
}

void JointHinge::paramLimitLoHi(double *lo, double *hi) const
{
    *lo = dJointGetHingeParam(_joint, dParamLoStop1);
    *hi = dJointGetHingeParam(_joint, dParamHiStop1);
}

bool JointHinge::setParamVel(double vel)
{
    dJointSetHingeParam(_joint, dParamVel, vel);

    // if velocity is non-zero enable connected bodies
    if (_enabled() && !isZero(vel)){
        _enable();
    }

    return true;
}

double JointHinge::paramVel() const
{
    return dJointGetHingeParam(_joint, dParamVel);
}

bool JointHinge::setParamFMax(double fmax)
{
    dJointSetHingeParam(_joint, dParamFMax, fmax);
    return true;
}

double JointHinge::paramFMax() const
{
    return dJointGetHingeParam(_joint, dParamFMax);
}

bool JointHinge::setParamBounce(double restitution)
{
    dJointSetHingeParam(_joint, dParamBounce, restitution);
    return true;
}

double JointHinge::paramBounce() const
{
    return dJointGetHingeParam(_joint, dParamBounce);
}


} /* namespace ode */

} /* namespace sim */
