#include "obj_base.hpp"

namespace sim {

VisObj::VisObj()
    : _node(0)
{
    _root = new osg::PositionAttitudeTransform();
}

void VisObj::setPosition(float x, float y, float z)
{
    _root->setPosition(osg::Vec3(x, y, z));
}

void VisObj::setRotation(float x, float y, float z, float w)
{
    _root->setAttitude(osg::Quat(x, y, z, w));
}


void VisObj::_setNode(osg::Node *n)
{
    _node = n;
    _root->addChild(_node);
}

}
