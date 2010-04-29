#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>

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
    _vis->setPosRot(Vec3::fromBullet(pos), Quat::fromBullet(rot));
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

    trans.setOrigin(v.toBullet());
    _motion_state->setWorldTransform(trans);
    _body->setWorldTransform(trans);
}

void Obj::setRot(const Quat &q)
{
    btTransform trans;
    _motion_state->getWorldTransform(trans);

    trans.setRotation(q.toBullet());
    _motion_state->setWorldTransform(trans);
    _body->setWorldTransform(trans);
}

void Obj::pos(Scalar *x, Scalar *y, Scalar *z) const
{
    Vec3 v = pos();
    *x = v.x();
    *y = v.y();
    *z = v.z();
}
Vec3 Obj::pos() const
{
    btTransform trans;
    _motion_state->getWorldTransform(trans);
    return Vec3::fromBullet(trans.getOrigin());
}

void Obj::rot(Scalar *x, Scalar *y, Scalar *z, Scalar *w) const
{
    Quat q = rot();
    *x = q.x();
    *y = q.y();
    *z = q.z();
    *w = q.w();
}

Quat Obj::rot() const
{
    btTransform trans;
    _motion_state->getWorldTransform(trans);
    return Quat::fromBullet(trans.getRotation());
}

void Obj::setPosRot(const Vec3 &v, const Quat &q)
{
    btTransform trans;
    _motion_state->getWorldTransform(trans);
    trans.setOrigin(v.toBullet());
    trans.setRotation(q.toBullet());
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

    // TODO
    _body->setDamping(0.3, 0.2);
}



ObjCube::ObjCube(Scalar w, Scalar mass)
    : Obj()
{
    btCollisionShape *shape = new btBoxShape(btVector3(w / 2., w / 2., w / 2.));
    _set(new VisObjCube(w), shape, mass);
}

ObjBox::ObjBox(Scalar x, Scalar y, Scalar z, Scalar mass)
    : Obj()
{
    btCollisionShape *shape = new btBoxShape(btVector3(x / 2., y / 2., z / 2.));
    _set(new VisObjBox(x, y, z), shape, mass);
}

ObjSphere::ObjSphere(Scalar radius, Scalar mass)
    : Obj()
{
    btCollisionShape *shape = new btSphereShape(radius);
    _set(new VisObjSphere(radius), shape, mass);
}

ObjCylinder::ObjCylinder(Scalar radius, Scalar height, Scalar mass)
    : Obj()
{
    Vec3 v(radius, radius, height / 2.);
    btCollisionShape *shape = new btCylinderShape(v.toBullet());
    _set(new VisObjCylinder(radius, height), shape, mass);
}

ObjCylinderX::ObjCylinderX(Scalar radius, Scalar height, Scalar mass)
    : ObjCylinder(radius, height, mass)
{
    // parent constructor already created cylinder
    // now it must be correctly rotated
    Quat q(Vec3(0., 1., 0.), M_PI * .5);
    setRot(q);
}

ObjCylinderY::ObjCylinderY(Scalar radius, Scalar height, Scalar mass)
    : ObjCylinder(radius, height, mass)
{
    // parent constructor already created cylinder
    // now it must be correctly rotated
    Quat q(Vec3(1., 0., 0.), M_PI * .5);
    setRot(q);
}

}
