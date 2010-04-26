#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Geometry>

#include "visobj.hpp"

namespace sim {

VisObj::VisObj()
    : _node(0)
{
    _root = new osg::PositionAttitudeTransform();
}

void VisObj::setPosition(float x, float y, float z)
{
    //_root->setPosition(osg::Vec3(x, y, z));
    _root->setPosition(osg::Vec3(x, -z, y));
}

void VisObj::setRotation(float x, float y, float z, float w)
{
    //_root->setAttitude(osg::Quat(x, y, z, w));
    _root->setAttitude(osg::Quat(x, -z, y, w));
}

void VisObj::getPosition(float *x, float *y, float *z)
{
    const osg::Vec3d &v = _root->getPosition();
    /*
    *x = v[0];
    *y = v[1];
    *z = v[2];
    */
    *x = v[0];
    *y = v[2];
    *z = -v[1];
}

void VisObj::getRotation(float *x, float *y, float *z, float *w)
{
    const osg::Quat &q = _root->getAttitude();
    /*
    *x = q.x();
    *y = q.y();
    *z = q.z();
    *w = q.w();
    */
    *x = q.x();
    *y = q.z();
    *z = -q.y();
    *w = q.w();
}

void VisObj::_setNode(osg::Node *n)
{
    _node = n;
    _root->addChild(_node);
}


VisObjCube::VisObjCube(float width)
{
    osg::Geode *geode = new osg::Geode();
    osg::Box *box = new osg::Box(osg::Vec3(0., 0., 0.), width);

    geode->addDrawable(new osg::ShapeDrawable(box));

    _setNode(geode);
}


VisObjSphere::VisObjSphere(float radius)
{
    osg::Geode *geode = new osg::Geode();
    osg::Sphere *sp = new osg::Sphere(osg::Vec3(0., 0., 0.), radius);

    geode->addDrawable(new osg::ShapeDrawable(sp));

    _setNode(geode);
}

}
