#include <BulletCollision/CollisionShapes/btBoxShape.h>

#include "obj.hpp"
#include "msg.hpp"

namespace sim {

void ObjMotionState::getWorldTransform(btTransform &world) const
{
    world = _offset.inverse() * _world;
}

void ObjMotionState::setWorldTransform(const btTransform &world)
{
    btVector3 pos;
    btQuaternion rot;

    _world = world * _offset;

    pos = _world.getOrigin();
    rot = _world.getRotation();
    _vis->setPosRot(pos, rot);
}

Obj::~Obj()
{
    if (_body)
        delete _body;
    if (_shape)
        delete _shape;
    if (_motion_state)
        delete _motion_state;
    if (_vis)
        delete _vis;
}

void Obj::setPos(const Vec3 &v)
{
    btTransform trans;
    _motion_state->getWorldTransform(trans);

    trans.setOrigin(v);
    _motion_state->setWorldTransform(trans);
    _body->setWorldTransform(trans);
}

void Obj::setRot(const Quat &q)
{
    btTransform trans;
    _motion_state->getWorldTransform(trans);

    trans.setRotation(q);
    _motion_state->setWorldTransform(trans);
    _body->setWorldTransform(trans);
}

void Obj::getPos(Scalar *x, Scalar *y, Scalar *z) const
{
    btTransform trans;
    _motion_state->getWorldTransform(trans);
    *x = trans.getOrigin().getX();
    *y = trans.getOrigin().getY();
    *z = trans.getOrigin().getZ();
}
void Obj::getPos(Vec3 *v) const
{
    btTransform trans;
    _motion_state->getWorldTransform(trans);
    *v = trans.getOrigin();
}

void Obj::getRot(Scalar *x, Scalar *y, Scalar *z, Scalar *w) const
{
    btTransform trans;
    _motion_state->getWorldTransform(trans);
    btQuaternion q = trans.getRotation();
    *x = q.x();
    *y = q.y();
    *z = q.z();
    *w = q.w();
}

void Obj::getRot(Quat *q) const
{
    btTransform trans;
    _motion_state->getWorldTransform(trans);
    *q = trans.getRotation();
}

void Obj::setPosRot(const Vec3 &v, const Quat &q)
{
    btTransform trans;
    _motion_state->getWorldTransform(trans);
    trans.setOrigin(v);
    trans.setRotation(q);
    _motion_state->setWorldTransform(trans);
    _body->setWorldTransform(trans);
}



void Obj::_set(VisObj *o, btCollisionShape *shape, Scalar mass)
{
    btVector3 local_inertia(0,0,0);

    _vis = o;
    _shape = shape;
    _motion_state = new ObjMotionState(o);

    if (!btFuzzyZero(mass))
        _shape->calculateLocalInertia(mass, local_inertia);

    _body = new btRigidBody(mass, _motion_state, _shape, local_inertia);
}



ObjCube::ObjCube(Scalar w, Scalar mass)
{
    btCollisionShape *shape = new btBoxShape(btVector3(w / 2., w / 2., w / 2.));
    _set(new VisObjCube(w), shape, mass);
}

ObjBox::ObjBox(Scalar x, Scalar y, Scalar z, Scalar mass)
{
    btCollisionShape *shape = new btBoxShape(btVector3(x / 2., y / 2., z / 2.));
    _set(new VisObjBox(x, y, z), shape, mass);
}



}
