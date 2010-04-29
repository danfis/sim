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

void VisObj::setPos(const Vec3 &v)
{
    _root->setPosition(v);
}


void VisObj::setRot(const Quat &q)
{
    _root->setAttitude(q);
}

void VisObj::setPosRot(const Vec3 &v, const Quat &q)
{
    setPos(v);
    setRot(q);
}

void VisObj::pos(Scalar *x, Scalar *y, Scalar *z) const
{
    Vec3 v = pos();
    *x = v[0];
    *y = v[1];
    *z = v[2];
}


void VisObj::rot(Scalar *x, Scalar *y, Scalar *z, Scalar *w) const
{
    Quat q = rot();
    *x = q.x();
    *y = q.z();
    *z = q.y();
    *w = -q.w();
}


void VisObj::_setNode(osg::Node *n)
{
    _node = n;
    _root->addChild(_node);
}


VisObjCube::VisObjCube(Scalar width)
{
    osg::Geode *geode = new osg::Geode();
    osg::Box *box = new osg::Box(Vec3(0., 0., 0.), width);

    geode->addDrawable(new osg::ShapeDrawable(box));

    _setNode(geode);
}

VisObjBox::VisObjBox(Vec3 dim)
{
    osg::Geode *geode = new osg::Geode();
    osg::Box *box = new osg::Box(osg::Vec3(0., 0., 0.), dim.x(), dim.y(), dim.z());

    geode->addDrawable(new osg::ShapeDrawable(box));

    _setNode(geode);
}


VisObjSphere::VisObjSphere(Scalar radius)
{
    osg::Geode *geode = new osg::Geode();
    osg::Sphere *sp = new osg::Sphere(Vec3(0., 0., 0.), radius);

    geode->addDrawable(new osg::ShapeDrawable(sp));

    _setNode(geode);
}

VisObjCylinder::VisObjCylinder(Scalar radius, Scalar height)
    : VisObj()
{
    osg::Geode *geode = new osg::Geode();
    osg::Cylinder *sp = new osg::Cylinder(Vec3(0., 0., 0.), radius, height);

    geode->addDrawable(new osg::ShapeDrawable(sp));

    _setNode(geode);
}

}
