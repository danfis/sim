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

void VisObj::getPos(Scalar *x, Scalar *y, Scalar *z) const
{
    Vec3 v;
    getPos(&v);
    *x = v[0];
    *y = v[1];
    *z = v[2];
}

void VisObj::getPos(Vec3 *o) const
{
    *o = (const Vec3 &)_root->getPosition();
}

void VisObj::getRot(Scalar *x, Scalar *y, Scalar *z, Scalar *w) const
{
    Quat q;
    getRot(&q);
    *x = q.x();
    *y = q.z();
    *z = q.y();
    *w = -q.w();
}

void VisObj::getRot(Quat *q) const
{
    *q = (const Quat &)_root->getAttitude();
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

VisObjBox::VisObjBox(Scalar x, Scalar y, Scalar z)
{
    osg::Geode *geode = new osg::Geode();
    osg::Box *box = new osg::Box(Vec3(0., 0., 0.), x, z, y);

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
