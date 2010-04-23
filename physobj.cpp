#include "physobj.hpp"

namespace sim {

PhysObj::PhysObj()
    : _body(0), _shape(0)
{
}

PhysObj::~PhysObj()
{
    if (_body)
        delete _body;
    if (_shape)
        delete _shape;
}

void PhysObj::setPosition(float x, float y, float z)
{
    btTransform trans;
    trans.setOrigin(btVector3(x, y, z));
    _body->setWorldTransform(trans);
}

void PhysObj::setRotation(float x, float y, float z, float w)
{
    btTransform trans;
    trans.setRotation(btQuaternion(x, y, z, w));
    _body->setWorldTransform(trans);
}

void PhysObj::getPosition(float *x, float *y, float *z)
{
    btTransform &trans = _body->getWorldTransform();
    *x = trans.getOrigin().getX();
    *y = trans.getOrigin().getY();
    *z = trans.getOrigin().getZ();
}

}
