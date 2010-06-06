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
    _world->addJoint(this);
}

void Joint::deactivate()
{
    /*TODO: _world->rmJoint(this); */
}

JointFixed::JointFixed(World *w, Body *oA, Body *oB)
    : Joint(w, oA, oB)
{
   btGeneric6DofConstraint * joint6DOF;
   btTransform localA, localB;
   btRigidBody *bA = oA->body();
   btRigidBody *bB = oB->body();

   localA.setIdentity();
   localB = bB->getCenterOfMassTransform().inverse() * bA->getCenterOfMassTransform();

   joint6DOF = new btGeneric6DofConstraint(*bA, *bB, localA, localB, true);

   joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
   joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON));

   _setJoint(joint6DOF);
}

JointHinge2::JointHinge2(World *w, Body *oA, Body *oB, const Vec3 &anchor, const Vec3 &axis1, const Vec3 &axis2)
    : Joint(w, oA, oB)
{
    btHinge2Constraint *c;
    btRigidBody *b1, *b2;
    btVector3 anch = vToBt(anchor);
    btVector3 a1 = vToBt(axis1);
    btVector3 a2 = vToBt(axis2);

    b1 = oA->body();
    b2 = oB->body();
   
    c = new btHinge2Constraint(*b1, *b2, anch, a1, a2);
    _setJoint(c);
}

bool JointHinge2::setParamLimitLoHi(double lo, double hi)
{
    btHinge2Constraint *j = (btHinge2Constraint *)_joint;
    j->setLimit(5, lo, hi);
    return true;
}

/*
void JointHinge2::setLimitLinAxis1(Scalar from, Scalar to)
{
    btHinge2Constraint *j = (btHinge2Constraint *)_joint;
    j->setLimit(2, from, to);
}

void JointHinge2::setLimitLinAxis2(Scalar from, Scalar to)
{
    btHinge2Constraint *j = (btHinge2Constraint *)_joint;
    j->setLimit(0, from, to);
}

void JointHinge2::setLimitAngAxis1(Scalar from, Scalar to)
{
}

void JointHinge2::setLimitAngAxis2(Scalar from, Scalar to)
{
    btHinge2Constraint *j = (btHinge2Constraint *)_joint;
    j->setLimit(3, from, to);
}
*/



JointHinge::JointHinge(World *w, Body *oA, Body *oB, const Vec3 &anchor, const Vec3 &axis)
    : Joint(w, oA, oB)
{
    btHingeConstraint *c;
    btTransform frameInA, frameInB;
    btRigidBody *bA, *bB;
    btVector3 anch = vToBt(anchor);
    btVector3 ax = vToBt(axis);

    bA = oA->body();
    bB = oB->body();

    frameInA.setIdentity();
    frameInB.setIdentity();

    frameInA.setOrigin(bA->getCenterOfMassTransform().inverse() * anch);
    frameInB.setOrigin(bB->getCenterOfMassTransform().inverse() * anch);

    btVector3 zAxis = ax;
    btVector3 xAxis = btVector3(0,1,0).cross(zAxis);
    assert (xAxis.normalized());
    btVector3 yAxis = zAxis.cross(xAxis);
    btMatrix3x3 basis(xAxis.x(), xAxis.y(), xAxis.z(), yAxis.x(), yAxis.y(), yAxis.z(), zAxis.x(), zAxis.y(), zAxis.z());
    frameInA.setBasis(bA->getCenterOfMassTransform().getBasis().inverse() * basis);
    frameInB.setBasis(bB->getCenterOfMassTransform().getBasis().inverse() * basis);
   
    c = new btHingeConstraint (*bA, *bB, frameInA, frameInB);
    _setJoint(c);
}

} /* namespace bullet */

} /* namespace sim */
