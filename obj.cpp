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
    _vis->setPos(pos.x(), pos.y(), pos.z());
    _vis->setRot(rot.x(), rot.y(), rot.z(), rot.w());
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

void Obj::setPos(float x, float y, float z)
{
    btTransform trans;
    _motion_state->getWorldTransform(trans);

    trans.setOrigin(btVector3(x, y, z));
    _motion_state->setWorldTransform(trans);
    _body->setWorldTransform(trans);
}

void Obj::setRot(float x, float y, float z, float w)
{
    btTransform trans;
    _motion_state->getWorldTransform(trans);

    trans.setRotation(btQuaternion(x, y, z, w));
    _motion_state->setWorldTransform(trans);
    _body->setWorldTransform(trans);
}

void Obj::getPos(float *x, float *y, float *z)
{
    btTransform trans;
    _motion_state->getWorldTransform(trans);
    *x = trans.getOrigin().getX();
    *y = trans.getOrigin().getY();
    *z = trans.getOrigin().getZ();
}

void Obj::getRot(float *x, float *y, float *z, float *w)
{
    btTransform trans;
    _motion_state->getWorldTransform(trans);
    btQuaternion q = trans.getRotation();
    *x = q.x();
    *y = q.y();
    *z = q.z();
    *w = q.w();
}

void Obj::setPosRot(float x, float y, float z,
                    float rx, float ry, float rz, float rw)
{
    btTransform trans;
    _motion_state->getWorldTransform(trans);
    trans.setOrigin(btVector3(x, y, z));
    trans.setRotation(btQuaternion(rx, ry, rz, rw));
    _motion_state->setWorldTransform(trans);
    _body->setWorldTransform(trans);
}



void Obj::_set(VisObj *o, btCollisionShape *shape, float mass)
{
    btVector3 local_inertia(0,0,0);

    _vis = o;
    _shape = shape;
    _motion_state = new ObjMotionState(o);

    if (!btFuzzyZero(mass))
        _shape->calculateLocalInertia(mass, local_inertia);

    _body = new btRigidBody(mass, _motion_state, _shape, local_inertia);
}



ObjCube::ObjCube(float w, float mass)
{
    btCollisionShape *shape = new btBoxShape(btVector3(w / 2., w / 2., w / 2.));
    _set(new VisObjCube(w), shape, mass);
}

ObjBox::ObjBox(float x, float y, float z, float mass)
{
    btCollisionShape *shape = new btBoxShape(btVector3(x / 2., y / 2., z / 2.));
    _set(new VisObjBox(x, y, z), shape, mass);
}



}
