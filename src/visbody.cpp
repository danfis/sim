#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osg/Array>
#include <osgText/Text>
#include <osgUtil/SmoothingVisitor>
#include <osg/Material>

#include "visbody.hpp"
#include "msg.hpp"

namespace sim {

VisBody::VisBody()
    : _node(0), _offset(0., 0., 0.)
{
    _root = new osg::PositionAttitudeTransform();
    _group = new osg::Group();
    _root->addChild(_group);

    _text = new osg::Geode();
    _group->addChild(_text);

    /* for debugging
    {
        osg::ShapeDrawable *draw = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0., 0., 0.), .1, .1, .1));
        osg::Geode *g = new osg::Geode;
        g->addDrawable(draw);
        _group->addChild(g);
    }
    */
}

void VisBody::setPos(const Vec3 &v)
{
    _root->setPosition(v + _offset);
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

    /*
    osg::StateSet *state_set;
    state_set = draw->getOrCreateStateSet();
    osg::ref_ptr<osg::Material> material = new osg::Material;
    material->setColorMode(osg::Material::AMBIENT_AND_DIFFUSE);
    //material->setAmbient(osg::Material::FRONT_AND_BACK, c);
    material->setDiffuse(osg::Material::FRONT_AND_BACK, c);
    state_set->setAttributeAndModes(material, osg::StateAttribute::ON);
    draw->setStateSet(state_set);
    */

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

    setColor(osg::Vec4(0.5, 0.5, 0.5, 1.));
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



VisBodyTriMesh::VisBodyTriMesh(const sim::Vec3 *coords, size_t coords_len,
                               const unsigned int *indices, size_t indices_len)
    : VisBody()
{
    size_t i;
    osg::ref_ptr<osg::Geode> g = new osg::Geode;
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    osg::ref_ptr<osg::Vec3Array> vert = new osg::Vec3Array;
    osg::ref_ptr<osg::DrawElementsUInt> faces;

    for (i = 0; i < coords_len; i++){
        vert->push_back(coords[i].toOsg());
    }
    geom->setVertexArray(vert);

    for (i = 0; i < indices_len; i += 3){
        faces = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
        faces->push_back(indices[i]);
        faces->push_back(indices[i + 1]);
        faces->push_back(indices[i + 2]);
        geom->addPrimitiveSet(faces);
    }

    osgUtil::SmoothingVisitor::smooth(*geom.get());

    g->addDrawable(geom);
    _setNode(g);
}

void VisBodyTriMesh::setColor(const osg::Vec4 &c)
{
    osg::Geometry *g = (osg::Geometry *)((osg::Geode *)_node.get())->getDrawable(0);
    osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
    color->push_back(c);
    g->setColorArray(color);
    g->setColorBinding(osg::Geometry::BIND_OVERALL);
}

}
