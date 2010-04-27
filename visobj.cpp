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

void VisObj::setPos(Scalar x, Scalar y, Scalar z)
{
    //_root->setPosition(osg::Vec3(x, y, z));
    _root->setPosition(osg::Vec3(x, -z, y));
}


void VisObj::setRot(Scalar x, Scalar y, Scalar z, Scalar w)
{
    //_root->setAttitude(osg::Quat(x, y, z, w));
    _root->setAttitude(osg::Quat(x, z, y, -w));
}

void VisObj::setPosRot(const Vec3 &v, const Quat &q)
{
    setPos(v);
    setRot(q);
}

void VisObj::getPos(Scalar *x, Scalar *y, Scalar *z) const
{
    const osg::Vec3d &v = _root->getPosition();
    /*
    *x = v[0];
    *y = v[1];
    *z = v[2];
    */
    *x = v[0];
    *y = -v[2];
    *z = v[1];
}

void VisObj::getPos(Vec3 *o) const
{
    const osg::Vec3d &v = _root->getPosition();

    o->setX(v[0]);
    o->setY(-v[2]);
    o->setZ(v[1]);
}

void VisObj::getRot(Scalar *x, Scalar *y, Scalar *z, Scalar *w) const
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
    *z = q.y();
    *w = -q.w();
}

void VisObj::getRot(Quat *q) const
{
    const osg::Quat &v = _root->getAttitude();

    q->setX(v.x());
    q->setY(v.z());
    q->setZ(v.y());
    q->setW(-v.w());
}

void VisObj::_setNode(osg::Node *n)
{
    _node = n;
    _root->addChild(_node);
}


VisObjCube::VisObjCube(Scalar width)
{
    osg::Geode *geode = new osg::Geode();
    osg::Box *box = new osg::Box(osg::Vec3(0., 0., 0.), width);

    geode->addDrawable(new osg::ShapeDrawable(box));

    _setNode(geode);
}

VisObjBox::VisObjBox(Scalar x, Scalar y, Scalar z)
{
    osg::Geode *geode = new osg::Geode();
    osg::Box *box = new osg::Box(osg::Vec3(0., 0., 0.), x, z, y);

    geode->addDrawable(new osg::ShapeDrawable(box));

    _setNode(geode);
}


VisObjSphere::VisObjSphere(Scalar radius)
{
    osg::Geode *geode = new osg::Geode();
    osg::Sphere *sp = new osg::Sphere(osg::Vec3(0., 0., 0.), radius);

    geode->addDrawable(new osg::ShapeDrawable(sp));

    _setNode(geode);
}

VisObjCylinder::VisObjCylinder(Scalar radius, Scalar height)
    : VisObj()
{
    osg::Geode *geode = new osg::Geode();
    osg::Cylinder *sp = new osg::Cylinder(osg::Vec3(0., 0., 0.), radius, height);

    geode->addDrawable(new osg::ShapeDrawable(sp));

    _setNode(geode);
}

}
