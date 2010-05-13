#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osg/Array>
#include <osgText/Text>

#include "visbody.hpp"
#include "msg.hpp"

namespace sim {

VisBody::VisBody()
    : _node(0)
{
    _root = new osg::PositionAttitudeTransform();
    _group = new osg::Group();
    _root->addChild(_group);

    _text = new osg::Geode();
    _group->addChild(_text);
}

void VisBody::setPos(const Vec3 &v)
{
    _root->setPosition(v);
}


void VisBody::setRot(const Quat &q)
{
    _root->setAttitude(q);
}

void VisBody::setPosRot(const Vec3 &v, const Quat &q)
{
    setPos(v);
    setRot(q);
}

void VisBody::pos(Scalar *x, Scalar *y, Scalar *z) const
{
    Vec3 v = pos();
    *x = v[0];
    *y = v[1];
    *z = v[2];
}


void VisBody::rot(Scalar *x, Scalar *y, Scalar *z, Scalar *w) const
{
    Quat q = rot();
    *x = q.x();
    *y = q.z();
    *z = q.y();
    *w = -q.w();
}

void VisBody::_setNode(osg::Node *n)
{
    _node = n;
    _group->addChild(_node);
}

void VisBody::setOsgText(osg::ref_ptr<osgText::TextBase> t)
{
    if (_text.valid()){
        _text->removeDrawables(0);
    }

    if (t.valid()){
        _text->addDrawable(t);
    }
}



void VisBodyShape::setColor(const osg::Vec4 &c)
{
    osg::ShapeDrawable *draw;
    draw = (osg::ShapeDrawable *)((osg::Geode *)_node.get())->getDrawable(0);
    draw->setColor(c);
}

void VisBodyShape::setText(const char *str, float size, const osg::Vec4 &color)
{
    osg::Drawable *draw;
    draw = (osg::Drawable *)((osg::Geode *)_node.get())->getDrawable(0);

    osg::ref_ptr<osgText::Text> text = new osgText::Text;
    text->setAxisAlignment(osgText::Text::SCREEN);
    //text->setBackdropType(osgText::Text::OUTLINE);
    //text->setDrawMode(osgText::Text::TEXT | osgText::Text::BOUNDINGBOX | osgText::Text::ALIGNMENT);
    text->setDrawMode(osgText::Text::TEXT);
    text->setAlignment(osgText::TextBase::CENTER_BOTTOM);

    text->setCharacterSize(size);
    text->setColor(color);

    // position text on top of shape
    text->setPosition(sim::Vec3(0., 0., draw->getBound().zMax()).toOsg());

    text->setText(str);

    setOsgText(text);
}

void VisBodyShape::_setShape(osg::Shape *shape)
{
    osg::Geode *geode = new osg::Geode();
    geode->addDrawable(new osg::ShapeDrawable(shape));
    _setNode(geode);
}


VisBodyBox::VisBodyBox(Vec3 dim)
    : VisBodyShape()
{
    _setShape(new osg::Box(osg::Vec3(0., 0., 0.), dim.x(), dim.y(), dim.z()));
}


VisBodySphere::VisBodySphere(Scalar radius)
    : VisBodyShape()
{
    _setShape(new osg::Sphere(Vec3(0., 0., 0.), radius));
}

VisBodyCylinder::VisBodyCylinder(Scalar radius, Scalar height)
    : VisBodyShape()
{
    _setShape(new osg::Cylinder(Vec3(0., 0., 0.), radius, height));
}

}
