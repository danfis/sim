#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <LinearMath/btDefaultMotionState.h>

#include "physobj.hpp"
#include "msg.hpp"

namespace sim {

PhysObj::PhysObj()
    : _body(0), _shape(0)
{
	btTransform init_trans;
	init_trans.setIdentity();
	init_trans.setOrigin(btVector3(0., 0., 0.));
    _motion_state = new btDefaultMotionState(init_trans);
}

PhysObj::~PhysObj()
{
    if (_body)
        delete _body;
    if (_shape)
        delete _shape;
    if (_motion_state)
        delete _motion_state;
}

void PhysObj::setPosition(float x, float y, float z)
{
    btTransform trans;
    _motion_state->getWorldTransform(trans);

    trans.setOrigin(btVector3(x, y, z));
    _motion_state->setWorldTransform(trans);
    _body->setWorldTransform(trans);
}

void PhysObj::setRotation(float x, float y, float z, float w)
{
    btTransform trans;
    _motion_state->getWorldTransform(trans);

    trans.setRotation(btQuaternion(x, y, z, w));
    _motion_state->setWorldTransform(trans);
    _body->setWorldTransform(trans);
}

void PhysObj::getPosition(float *x, float *y, float *z)
{
    btTransform trans;
    _motion_state->getWorldTransform(trans);
    *x = trans.getOrigin().getX();
    *y = trans.getOrigin().getY();
    *z = trans.getOrigin().getZ();
}

void PhysObj::getRotation(float *x, float *y, float *z, float *w)
{
    btTransform trans;
    _motion_state->getWorldTransform(trans);
    btQuaternion q = trans.getRotation();
    *x = q.x();
    *y = q.y();
    *z = q.z();
    *w = q.w();
}



PhysObjCube::PhysObjCube(float width, float mass)
    : PhysObj()
{
    btCollisionShape *shape;
    btRigidBody *body;

    shape = new btBoxShape(btVector3(width / 2., width / 2., width / 2.));

    body = new btRigidBody(mass, _motion_state, shape);
    _setBody(body);
    _setShape(shape);
}

PhysObjBox::PhysObjBox(float x, float y, float z, float mass)
    : PhysObj()
{

	btCollisionShape* shape = new btBoxShape(btVector3(x / 2., y / 2., z / 2.));

    btRigidBody* body = new btRigidBody(mass, _motion_state, shape);

    _setBody(body);
    _setShape(shape);
}

}
