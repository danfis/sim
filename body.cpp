#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>

#include "body.hpp"
#include "msg.hpp"

namespace sim {

void BodyMotionState::getWorldTransform(btTransform &world) const
{
    world = _offset.inverse() * _world;
}

void BodyMotionState::setWorldTransform(const btTransform &world)
{
    btVector3 pos;
    btQuaternion rot;

    _world = world * _offset;

    pos = _world.getOrigin();
    rot = _world.getRotation();
    _vis->setPosRot(Vec3::fromBullet(pos), Quat::fromBullet(rot));
}

Body::~Body()
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

void Body::setPos(const Vec3 &v)
{
    btTransform trans;
    _motion_state->getWorldTransform(trans);

    trans.setOrigin(v.toBullet());
    _motion_state->setWorldTransform(trans);
    _body->setWorldTransform(trans);
}

void Body::setRot(const Quat &q)
{
    btTransform trans;
    _motion_state->getWorldTransform(trans);

    trans.setRotation(q.toBullet());
    _motion_state->setWorldTransform(trans);
    _body->setWorldTransform(trans);
}

void Body::pos(Scalar *x, Scalar *y, Scalar *z) const
{
    Vec3 v = pos();
    *x = v.x();
    *y = v.y();
    *z = v.z();
}
Vec3 Body::pos() const
{
    btTransform trans;
    _motion_state->getWorldTransform(trans);
    return Vec3::fromBullet(trans.getOrigin());
}

void Body::rot(Scalar *x, Scalar *y, Scalar *z, Scalar *w) const
{
    Quat q = rot();
    *x = q.x();
    *y = q.y();
    *z = q.z();
    *w = q.w();
}

Quat Body::rot() const
{
    btTransform trans;
    _motion_state->getWorldTransform(trans);
    return Quat::fromBullet(trans.getRotation());
}

void Body::setPosRot(const Vec3 &v, const Quat &q)
{
    btTransform trans;
    _motion_state->getWorldTransform(trans);
    trans.setOrigin(v.toBullet());
    trans.setRotation(q.toBullet());
    _motion_state->setWorldTransform(trans);
    _body->setWorldTransform(trans);
}



void Body::_set(VisBody *o, btCollisionShape *shape, Scalar mass)
{
    btVector3 local_inertia(0,0,0);

    _vis = o;
    _shape = shape;
    _motion_state = new BodyMotionState(o);

    if (!btFuzzyZero(mass))
        _shape->calculateLocalInertia(mass, local_inertia);

    _body = new btRigidBody(mass, _motion_state, _shape, local_inertia);

    // TODO
    _body->setDamping(0.3, 0.2);
}



BodyCube::BodyCube(Scalar w, Scalar mass)
    : Body()
{
    btCollisionShape *shape = new btBoxShape(btVector3(w / 2., w / 2., w / 2.));
    _set(new VisBodyCube(w), shape, mass);
}

BodyBox::BodyBox(Vec3 dim, Scalar mass)
    : Body()
{
    btCollisionShape *shape = new btBoxShape(Vec3::toBullet(dim / 2.));
    _set(new VisBodyBox(dim), shape, mass);
}

BodySphere::BodySphere(Scalar radius, Scalar mass)
    : Body()
{
    btCollisionShape *shape = new btSphereShape(radius);
    _set(new VisBodySphere(radius), shape, mass);
}

BodyCylinder::BodyCylinder(Scalar radius, Scalar height, Scalar mass)
    : Body()
{
    Vec3 v(radius, radius, height / 2.);
    btCollisionShape *shape = new btCylinderShape(v.toBullet());
    _set(new VisBodyCylinder(radius, height), shape, mass);
}

BodyCylinderX::BodyCylinderX(Scalar radius, Scalar height, Scalar mass)
    : BodyCylinder(radius, height, mass)
{
    // parent constructor already created cylinder
    // now it must be correctly rotated
    Quat q(Vec3(0., 1., 0.), M_PI * .5);
    setRot(q);
}

BodyCylinderY::BodyCylinderY(Scalar radius, Scalar height, Scalar mass)
    : BodyCylinder(radius, height, mass)
{
    // parent constructor already created cylinder
    // now it must be correctly rotated
    Quat q(Vec3(1., 0., 0.), M_PI * .5);
    setRot(q);
}

}
