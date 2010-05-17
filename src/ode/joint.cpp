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
    dJointEnable(_joint);
}

void Joint::deactivate()
{
    dJointDisable(_joint);
}

void Joint::_setJoint(dJointID j)
{
    Body *a, *b;

    a = (Body *)objA();
    b = (Body *)objB();

    DBG(_joint << " " << a->body() << " " << b->body());

    _joint = j;
    dJointAttach(_joint, a->body(), b->body());

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
    : Joint(w, oA, oB)
{
    dJointID joint;

    joint = dJointCreateHinge2(w->world(), 0);

    _setJoint(joint);

    dJointSetHinge2Anchor(joint, anchor.x(), anchor.y(), anchor.z());
    dJointSetHinge2Axis1(joint, axis1.x(), axis1.y(), axis1.z());
    dJointSetHinge2Axis2(joint, axis2.x(), axis2.y(), axis2.z());
}

void JointHinge2::setLimitAngAxis1(Scalar from, Scalar to)
{
    dJointSetHinge2Param(_joint, dParamLoStop1, from);
    dJointSetHinge2Param(_joint, dParamHiStop1, to);
}


JointHinge::JointHinge(World *w, Body *oA, Body *oB, const Vec3 &anchor, const Vec3 &axis)
    : Joint(w, oA, oB)
{
    dJointID joint;

    joint = dJointCreateHinge(w->world(), 0);

    _setJoint(joint);

    dJointSetHingeAnchor(joint, anchor.x(), anchor.y(), anchor.z());
    dJointSetHingeAxis(joint, axis.x(), axis.y(), axis.z());
}

void JointHinge::setLimitAngAxis(Scalar from, Scalar to)
{
    dJointSetHingeParam(_joint, dParamLoStop, from);
    dJointSetHingeParam(_joint, dParamHiStop, to);
}

} /* namespace ode */

} /* namespace sim */
