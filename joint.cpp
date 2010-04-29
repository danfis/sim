#include <BulletDynamics/ConstraintSolver/btHinge2Constraint.h>

#include "joint.hpp"
#include "msg.hpp"

namespace sim {

Joint::Joint(Obj *oA, Obj *oB)
    : _o_a(oA), _o_b(oB), _joint(0)
{
}

Joint::~Joint()
{
    if (_joint)
        delete _joint;
}

JointHinge2::JointHinge2(Obj *oA, Obj *oB, const Vec3 &anchor, const Vec3 &axis1, const Vec3 &axis2)
    : Joint(oA, oB)
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

    c->enableSpring(0, true);
    c->setStiffness(0, 1000.);
    c->setStiffness(1, 1000.);
    c->setStiffness(2, 1000.);
    c->setLinearLowerLimit(Vec3::toBullet(0., 0., 0.));
    c->setLinearUpperLimit(Vec3::toBullet(0., 0., 0.));

    DBG(c);
    _setJoint(c);
}

}
