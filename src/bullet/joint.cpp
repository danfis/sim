#include <BulletDynamics/ConstraintSolver/btHingeConstraint.h>
#include <BulletDynamics/ConstraintSolver/btHinge2Constraint.h>
#include <BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h>

#include "sim/bullet/joint.hpp"
#include "sim/bullet/world.hpp"
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
    btVector3 anch = anchor.toBullet();
    btVector3 a1 = axis1.toBullet();
    btVector3 a2 = axis2.toBullet();

    b1 = oA->body();
    b2 = oB->body();
   
    DBG("anchor: " << anchor.x() << " " << anchor.y() << " " << anchor.z());
    c = new btHinge2Constraint(*b1, *b2, anch, a1, a2);
    _setJoint(c);
}

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
    btHinge2Constraint *j = (btHinge2Constraint *)_joint;
    j->setLimit(5, from, to);
}

void JointHinge2::setLimitAngAxis2(Scalar from, Scalar to)
{
    btHinge2Constraint *j = (btHinge2Constraint *)_joint;
    j->setLimit(3, from, to);
}



JointHinge::JointHinge(World *w, Body *oA, Body *oB, const Vec3 &anchor, const Vec3 &axis)
    : Joint(w, oA, oB)
{
    btHingeConstraint *c;
    btTransform frameInA, frameInB;
    btRigidBody *bA, *bB;
    btVector3 anch = anchor.toBullet();
    btVector3 ax = axis.toBullet();

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


JointPoint2Point::JointPoint2Point(World *w, Body *oA, Body *oB,
                                   const Vec3 &pivotA, const Vec3 &pivotB)
    : Joint(w, oA, oB)
{
    btPoint2PointConstraint *c;
    btVector3 pA = pivotA.toBullet();
    btVector3 pB = pivotB.toBullet();

    c = new btPoint2PointConstraint(*oA->body(), *oB->body(), pA, pB);
    _setJoint(c);
}

} /* namespace bullet */

} /* namespace sim */
