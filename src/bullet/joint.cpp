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

#include <BulletDynamics/ConstraintSolver/btHingeConstraint.h>
#include <BulletDynamics/ConstraintSolver/btHinge2Constraint.h>
#include <BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h>

#include "sim/bullet/joint.hpp"
#include "sim/bullet/world.hpp"
#include "sim/bullet/math.hpp"
#include "sim/msg.hpp"

namespace sim {

namespace bullet {

Joint::Joint(World *w, Body *oA, Body *oB)
    : sim::Joint(oA, oB), _world(w), _joint(0)
{
}

Joint::~Joint()
{
    if (_joint)
        delete _joint;
}


void Joint::activate()
{
    _world->world()->addConstraint(_joint);
}

void Joint::deactivate()
{
    _world->world()->removeConstraint(_joint);
}

void Joint::_enable()
{
    Body *a = 0, *b = 0;
    a = (Body *)objA();
    b = (Body *)objB();

    if (a)
        a->body()->activate(true);
    if (b)
        b->body()->activate(true);
}

JointFixed::JointFixed(World *w, Body *oA, Body *oB)
    : Joint(w, oA, oB)
{
}

void JointFixed::activate()
{
   btGeneric6DofConstraint * joint6DOF;
   btTransform localA, localB;
   btRigidBody *bA = ((Body *)objA())->body();
   btRigidBody *bB = ((Body *)objB())->body();

   localA.setIdentity();
   localB = bB->getCenterOfMassTransform().inverse() * bA->getCenterOfMassTransform();

   // TODO: what if objB is NULL?!
   joint6DOF = new btGeneric6DofConstraint(*bA, *bB, localA, localB, true);

   joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
   joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON));

   _setJoint(joint6DOF);

   Joint::activate();
}

JointHinge2::JointHinge2(World *w, Body *oA, Body *oB, const Vec3 &anchor, const Vec3 &axis1, const Vec3 &axis2)
    : Joint(w, oA, oB),
      _anchor(anchor), _axis1(axis1), _axis2(axis2)
{
    _lim[0] = -10000;
    _lim[1] = 10000;
}

void JointHinge2::activate()
{
    btHinge2Constraint *c;
    btRigidBody *b1, *b2;
    btVector3 anch = vToBt(_anchor);
    btVector3 a1 = vToBt(_axis1);
    btVector3 a2 = vToBt(_axis2);

    b1 = ((Body *)objA())->body();
    b2 = ((Body *)objB())->body();
   
    c = new btHinge2Constraint(*b1, *b2, anch, a1, a2);
    _setJoint(c);

    Joint::activate();

    setParamLimitLoHi(_lim[0], _lim[1]);
}

Scalar JointHinge::angle() const
{
    if (_joint)
        return ((btHingeConstraint *)_joint)->getHingeAngle();
    return 0.;
}

bool JointHinge2::setParamLimitLoHi(double lo, double hi)
{
    _lim[0] = lo;
    _lim[1] = hi;

    if (_joint){
        btHinge2Constraint *j = (btHinge2Constraint *)_joint;
        j->setLimit(5, lo, hi);
    }

    return true;
}

void JointHinge2::paramLimitLoHi(double *lo, double *hi) const
{
    *lo = _lim[0];
    *hi = _lim[1];
}



JointHinge::JointHinge(World *w, Body *oA, Body *oB, const Vec3 &anchor, const Vec3 &axis)
    : Joint(w, oA, oB),
      _anchor(anchor), _axis(axis),
      _vel(0.), _fmax(0.)
{
    _lim[0] = -10000;
    _lim[1] = 10000;
}

void JointHinge::activate()
{
    btHingeConstraint *c;
    btTransform frameInA, frameInB;
    btRigidBody *bA, *bB;
    btVector3 anch = vToBt(_anchor);
    btVector3 ax = vToBt(_axis);


    btVector3 zAxis = ax;
    btVector3 xAxis = btVector3(0,1,0).cross(zAxis);
    assert (xAxis.normalized());
    btVector3 yAxis = zAxis.cross(xAxis);
    btMatrix3x3 basis(xAxis.x(), xAxis.y(), xAxis.z(), yAxis.x(), yAxis.y(), yAxis.z(), zAxis.x(), zAxis.y(), zAxis.z());

    bA = ((Body *)objA())->body();
    frameInA.setIdentity();
    frameInA.setOrigin(bA->getCenterOfMassTransform().inverse() * anch);
    frameInA.setBasis(bA->getCenterOfMassTransform().getBasis().inverse() * basis);

    if (objB()){
        bB = ((Body *)objB())->body();
        frameInB.setIdentity();
        frameInB.setOrigin(bB->getCenterOfMassTransform().inverse() * anch);
        frameInB.setBasis(bB->getCenterOfMassTransform().getBasis().inverse() * basis);

        c = new btHingeConstraint (*bA, *bB, frameInA, frameInB);
    }else{
        c = new btHingeConstraint (*bA, frameInA);
    }

    _setJoint(c);

    Joint::activate();

    setParamLimitLoHi(_lim[0], _lim[1]);
}

bool JointHinge::setParamLimitLoHi(double lo, double hi)
{
    _lim[0] = lo;
    _lim[1] = hi;

    if (_joint){
        ((btHingeConstraint *)_joint)->setLimit(_lim[0], _lim[1]);
    }

    return true;
}

void JointHinge::paramLimitLoHi(double *lo, double *hi) const
{
    *lo = _lim[0];
    *hi = _lim[1];
}

bool JointHinge::setParamVel(double vel)
{
    _vel = vel;
    return true;
}

bool JointHinge::setParamFMax(double fmax)
{
    _fmax = fmax;
    return true;
}

void JointHinge::_applyVelFMax(const sim::Time &time)
{
    Scalar imp;

    if (_joint){
        //DBG(this << " " << _vel << " " << _fmax);
        if (!isZero(_fmax) && !isZero(_vel)){
            imp = _fmax * time.inSF();
            //DBG(imp << " " << _fmax << " " << time.inSF());
            ((btHingeConstraint *)_joint)->enableAngularMotor(true, -_vel, imp);
            _enable();
        }else{
            ((btHingeConstraint *)_joint)->enableAngularMotor(false, 0., 0.);
        }
    }
}

} /* namespace bullet */

} /* namespace sim */
