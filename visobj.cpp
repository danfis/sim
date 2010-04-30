#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osg/Array>

#include "visobj.hpp"
#include "msg.hpp"

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


void VisObjShape::setColor(const osg::Vec4 &c)
{
    osg::ShapeDrawable *draw;
    draw = (osg::ShapeDrawable *)((osg::Geode *)_node)->getDrawable(0);
    draw->setColor(c);
}

void VisObjShape::_setShape(osg::Shape *shape)
{
    osg::Geode *geode = new osg::Geode();
    geode->addDrawable(new osg::ShapeDrawable(shape));
    _setNode(geode);
}

VisObjCube::VisObjCube(Scalar width)
    : VisObjShape()
{
    _setShape(new osg::Box(Vec3(0., 0., 0.), width));
}


VisObjBox::VisObjBox(Vec3 dim)
    : VisObjShape()
{
    _setShape(new osg::Box(osg::Vec3(0., 0., 0.), dim.x(), dim.y(), dim.z()));
}


VisObjSphere::VisObjSphere(Scalar radius)
    : VisObjShape()
{
    _setShape(new osg::Sphere(Vec3(0., 0., 0.), radius));
}

VisObjCylinder::VisObjCylinder(Scalar radius, Scalar height)
    : VisObjShape()
{
    _setShape(new osg::Cylinder(Vec3(0., 0., 0.), radius, height));
}

}
