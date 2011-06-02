/***
 * sim
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
 *                   Vojta Vonasek <vonasek@labe.felk.cvut.cz>
 *
 *  This file is part of sim.
 *
 *  sim is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation; either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  sim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osg/Array>
#include <osgText/Text>
#include <osgUtil/SmoothingVisitor>
#include <osg/Material>
#include <osgDB/ReadFile>
#include <string.h>

#include "visbody.hpp"
#include "msg.hpp"

namespace sim {

void VisBody::povCoords(std::ostream &os, const osg::Vec3 &vec) const
{
    os << " <" << vec[0] << "," << vec[1] << "," << vec[2] << "> ";
}

void VisBody::povTransformation(std::ostream &ofs, const osg::Vec3 &position, const osg::Quat &rotation) const
{
    osg::Matrixd mt;
    mt.makeIdentity();
    mt.makeTranslate(position);

    osg::Matrixd mr;
    mr.makeIdentity();
    mr.makeRotate(rotation);

    osg::Matrix m = mr*mt;
//    DBG("Rotation="<<rotation[0] <<"," << rotation[1] <<","<< rotation[2] << "," << rotation[3]);
    ofs << "matrix <";
    ofs << m(0,0) << "," << m(0,1) <<","<<m(0,2) <<",\n";
    ofs << m(1,0) << "," << m(1,1) <<","<<m(1,2) <<",\n";
    ofs << m(2,0) << "," << m(2,1) <<","<<m(2,2) <<",\n";
    ofs << m(3,0) << "," << m(3,1) <<","<<m(3,2) <<">\n";
}

void VisBody::povColor(std::ostream &os, const osg::Vec4 &color) const
{
    //ofs << "color rgb <" << color[0]<<","<<color[1]<<","<<color[2]<<"> ";
    os << "color rgbf <" << color[0]<<","<<color[1]<<","<<color[2]<<"," << (1-color[3]) << "> ";
}
static void printBlenderMaterial(std::ostream &ofs, const osg::Vec4 &color, const int idx) {
    ofs << "    mat = Material.New('mat_" << idx << "')\n";
    ofs << "    mat.rgbCol = [" << color[0] << ","  << color[1] << "," << color[2] << "]\n";
    ofs << "    mat.setAlpha(" << color[3] << ")\n";
    ofs << "    n = ob.getData()\n";
    ofs << "    n.materials = [ mat ]\n";
    ofs << "    n.update()\n";
}

VisBody::VisBody()
    : _id(_getUniqueID()), _node(0), _offset(0., 0., 0.)
{
    _root = new osg::PositionAttitudeTransform();
    _group = new osg::Group();
    _root->addChild(_group.get());

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


unsigned long VisBody::_last_id = 0L;
unsigned long VisBody::_getUniqueID()
{
    return ++_last_id;
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

const osg::Vec4 &VisBody::color() const {
    osg::ShapeDrawable *draw;
    draw = (osg::ShapeDrawable *)((osg::Geode *)_node.get())->getDrawable(0);

	return draw->getColor();
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

void VisBodyShape::setTexture(const std::string &fn)
{
    osg::ShapeDrawable *draw;
    draw = (osg::ShapeDrawable *)((osg::Geode *)_node.get())->getDrawable(0);

    osg::Image *image = osgDB::readImageFile(fn);
    if (!image){
        ERR("Could not find texture in file " << fn);
        return;
    }

    osg::Texture2D *tex = new osg::Texture2D;
    tex->setDataVariance(osg::Object::DYNAMIC);
    tex->setImage(image);

    osg::StateSet *state = draw->getOrCreateStateSet();
    state->setTextureAttributeAndModes(0, tex, osg::StateAttribute::ON);
    draw->setStateSet(state);

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

void VisBodyShape::toPovrayObject(std::ostream &os) const
{
    const osg::Geode *geo;
    const osg::ShapeDrawable *draw;
    const osg::Shape *shape;

    if (!(geo = (const osg::Geode *)node()))
        return;
    if (!(draw = (const osg::ShapeDrawable *)geo->getDrawable(0)))
        return;
    if (!(shape = draw->getShape()))
        return;

    os << "#declare object_" << id() << " = object {" << std::endl;
    _toPovrayFullShape(os, shape, draw);
    os << "}" << std::endl; // object
}

void VisBodyShape::toPovrayTr(std::ostream &os) const
{
    os << "object { object_" << id() << std::endl;
    povTransformation(os, pos(), rot());
    os << "}" << std::endl; // object
}


VisBodyBox::VisBodyBox(Vec3 dim)
    : VisBodyShape()
{
    _setShape(new osg::Box(osg::Vec3(0., 0., 0.), dim.x(), dim.y(), dim.z()));
}

void VisBodyBox::toBlender(std::ostream &os) const
{
    const osg::Geode *geo;
    const osg::ShapeDrawable *draw;
    const osg::Box *shape;

    if (!(geo = (const osg::Geode *)node()))
        return;
    if (!(draw = (const osg::ShapeDrawable *)geo->getDrawable(0)))
        return;
    if (!(shape = (const osg::Box *)draw->getShape()))
        return;

    Vec3 center(shape->getCenter());
    Vec3 lengths(shape->getHalfLengths());

    os << "try:\n";
    os << "    ob = Blender.Object.Get('object_" << id() << "')\n";
    os << "except:\n";
    os << "    me = Mesh.Primitives.Cube(1.0)\n";
    os << "    sc.objects.new(me,'object_" << id() << "')\n";
    os << "    ob = Blender.Object.Get('object_" << id() << "')\n";
    os << "    ob.setSize(" << (lengths[0]*2) << "," << (lengths[1]*2) << "," << (lengths[2]*2) << ")\n";

    printBlenderMaterial(os, draw->getColor(), id());
    os << "\n";
}

void VisBodyBox::exportToPovray(std::ofstream &ofs, PovrayMode mode) {
    osg::Geode *g = _node->asGeode();

    if (!g) {
        return;
    }

    if (mode == POVRAY_GEOM || mode == POVRAY_GEOMTRANSFROM) {
        for(int i=0;i<(int)g->getNumDrawables();i++) {
            osg::Drawable *d = g->getDrawable(i);
            if (d) {
                osg::Box *b = (osg::Box *)d->getShape();
                osg::ShapeDrawable *sd = (osg::ShapeDrawable *)d;
                Vec3 center(b->getCenter());
                Vec3 lengths(b->getHalfLengths());
                
                ofs << "box { <" << center[0]-lengths[0] << "," << center[1]-lengths[1] << "," << center[2]-lengths[2] << ">,";
                ofs << " <" << center[0]+lengths[0] << "," << center[1]+lengths[1] << "," << center[2]+lengths[2] << "> ";

                ofs << " pigment {";
                povColor(ofs,sd->getColor());
                ofs << "} ";
                ofs << "}\n";
            }
        }
    } 

    if (mode == POVRAY_TRANSFORM || mode == POVRAY_GEOMTRANSFROM) {
        povTransformation(ofs,pos(),rot());
    } 
}

void VisBodyBox::exportToBlender(std::ofstream &ofs, const int idx) {
    osg::Geode *g = _node->asGeode();

    if (!g) {
        return;
    }

    for(int i=0;i<(int)g->getNumDrawables();i++) {
        osg::Drawable *d = g->getDrawable(i);
        if (d) {
            osg::Box *b = (osg::Box *)d->getShape();
            osg::ShapeDrawable *sd = (osg::ShapeDrawable *)d;
            Vec3 center(b->getCenter());
            Vec3 lengths(b->getHalfLengths());
            ofs << "try:\n";
            ofs << "    ob = Blender.Object.Get('object_" << idx << "')\n";
            ofs << "except:\n";
            ofs << "    me = Mesh.Primitives.Cube(1.0)\n";
            ofs << "    sc.objects.new(me,'object_" << idx << "')\n";
            ofs << "    ob = Blender.Object.Get('object_" << idx << "')\n";
            ofs << "    ob.setSize(" << (lengths[0]*2) << "," << (lengths[1]*2) << "," << (lengths[2]*2) << ")\n";
            printBlenderMaterial(ofs,sd->getColor(),idx);
            ofs << "\n";

        }
    }
 
}

void VisBodyBox::_toPovrayFullShape(std::ostream &os,
                                      const osg::Shape *shape,
                                      const osg::ShapeDrawable *draw) const
{
    const osg::Box *b = (const osg::Box *)shape;
    Vec3 center(b->getCenter());
    Vec3 lengths(b->getHalfLengths());

    os << "box { <" << center[0] - lengths[0] << ","
                    << center[1] - lengths[1] << ","
                    << center[2]-lengths[2] << ">,"
       << " <" << center[0]+lengths[0] << ","
               << center[1]+lengths[1] << ","
               << center[2]+lengths[2] << "> ";

    os << " pigment {";
    povColor(os, draw->getColor());
    os << " } ";
    os << "}" << std::endl;
}

void VisBodyCube::exportToPovray(std::ofstream &ofs, PovrayMode mode) {
    osg::Geode *g = _node->asGeode();

    if (!g) {
        return;
    }

    if (mode == POVRAY_GEOM || mode == POVRAY_GEOMTRANSFROM) {
        for(int i=0;i<(int)g->getNumDrawables();i++) {
            osg::Drawable *d = g->getDrawable(i);
            if (d) {
                osg::Box *b = (osg::Box *)d->getShape();
                osg::ShapeDrawable *sd = (osg::ShapeDrawable *)d;
                Vec3 center(b->getCenter());
                Vec3 lengths(b->getHalfLengths());
                ofs << "box { <" << center[0]-lengths[0] << "," << center[1]-lengths[1] << "," << center[2]-lengths[2] << ">,";
                ofs << " <" << center[0]+lengths[0] << "," << center[1]+lengths[1] << "," << center[2]+lengths[2] << "> ";
 				ofs << " pigment {";
                povColor(ofs,sd->getColor());
                ofs << "} ";
                ofs << "}\n";


            }
        }
    } 

    if (mode == POVRAY_TRANSFORM || mode == POVRAY_GEOMTRANSFROM) {
        povTransformation(ofs,pos(),rot());
    } 

}

void VisBodyCube::exportToBlender(std::ofstream &ofs, const int idx) {

    osg::Geode *g = _node->asGeode();

    if (!g) {
        return;
    }


    for(int i=0;i<(int)g->getNumDrawables();i++) {
        osg::Drawable *d = g->getDrawable(i);
        if (d) {
            osg::Box *b = (osg::Box *)d->getShape();
            osg::ShapeDrawable *sd = (osg::ShapeDrawable *)d;
            Vec3 center(b->getCenter());
            Vec3 lengths(b->getHalfLengths());

            ofs << "try:\n";
            ofs << "    ob = Blender.Object.Get('object_" << idx << "')\n";
            ofs << "except:\n";
            ofs << "    me = Mesh.Primitives.Cube(1.0)\n";
            ofs << "    ob = Blender.Object.Get('object_" << idx << "')\n";
            ofs << "    ob.setSize(" << lengths[0]*2 << "," << lengths[1]*2 << "," << lengths[2]*2 << ")\n";
            printBlenderMaterial(ofs,sd->getColor(),idx);
            ofs << "\n";
        }
    }
 
}



VisBodySphere::VisBodySphere(Scalar radius)
    : VisBodyShape()
{
    _setShape(new osg::Sphere(Vec3(0., 0., 0.), radius));
}


void VisBodySphere::toBlender(std::ostream &os) const
{
    const osg::Geode *geo;
    const osg::ShapeDrawable *draw;
    const osg::Sphere *shape;

    if (!(geo = (const osg::Geode *)node()))
        return;
    if (!(draw = (const osg::ShapeDrawable *)geo->getDrawable(0)))
        return;
    if (!(shape = (const osg::Sphere *)draw->getShape()))
        return;

    double radius = shape->getRadius();

    os << "try:\n";
    os << "    ob = Blender.Object.Get('object_" << id() << "')\n";
    os << "except:\n";
    os << "    me = Mesh.Primitives.UVsphere(32,32," << radius << ")\n";
    os << "    ob = sc.objects.new(me,'object_" << id() << "')\n";

    printBlenderMaterial(os, draw->getColor(), id());
    os << "\n";
}

void VisBodySphere::exportToPovray(std::ofstream &ofs, PovrayMode mode) {
    osg::Geode *g = _node->asGeode();

    if (!g) {
        return;
    }

    if (mode == POVRAY_GEOM || mode == POVRAY_GEOMTRANSFROM) {
        for(int i=0;i<(int)g->getNumDrawables();i++) {
            osg::Drawable *d = g->getDrawable(i);
            if (d) {
                osg::Sphere *b = (osg::Sphere *)d->getShape();
                osg::ShapeDrawable *sd = (osg::ShapeDrawable *)d;
                Vec3 center(b->getCenter());
                double radius = b->getRadius();
                ofs << "sphere { ";
                povCoords(ofs,center);
                ofs << "," << radius << "\n";

                ofs << " pigment {";
                povColor(ofs,sd->getColor());
                ofs << "} ";
                ofs << "}\n";
            }
        }
    } 

    if (mode == POVRAY_TRANSFORM || mode == POVRAY_GEOMTRANSFROM) {
        povTransformation(ofs,pos(),rot());
    } 

}

void VisBodySphere::exportToBlender(std::ofstream &ofs, const int idx) {
    osg::Geode *g = _node->asGeode();

    if (!g) {
        return;
    }

    for(int i=0;i<(int)g->getNumDrawables();i++) {
        osg::Drawable *d = g->getDrawable(i);
        if (d) {
            osg::Sphere *b = (osg::Sphere *)d->getShape();
            osg::ShapeDrawable *sd = (osg::ShapeDrawable *)d;
            Vec3 center(b->getCenter());
            double radius = b->getRadius();

            ofs << "try:\n";
            ofs << "    ob = Blender.Object.Get('object_" << idx << "')\n";
            ofs << "except:\n";
            ofs << "    me = Mesh.Primitives.UVsphere(32,32," << radius << ")\n";
            ofs << "    sc.objects.new(me,'object_" << idx << "')\n";
            printBlenderMaterial(ofs,sd->getColor(),idx);
            ofs << "\n";

        }
    }
     
}

void VisBodySphere::_toPovrayFullShape(std::ostream &os,
                                      const osg::Shape *shape,
                                      const osg::ShapeDrawable *draw) const
{

    const osg::Sphere *s = (const osg::Sphere *)shape;
    Vec3 center(s->getCenter());
    double radius = s->getRadius();

    os << "sphere { ";
    povCoords(os, center);
    os << "," << radius << "\n";

    os << " pigment {";
    povColor(os, draw->getColor());
    os << "} ";
    os << "}\n";
}



VisBodyCylinder::VisBodyCylinder(Scalar radius, Scalar height)
    : VisBodyShape()
{
    _setShape(new osg::Cylinder(Vec3(0., 0., 0.), radius, height));
}

void VisBodyCylinder::toBlender(std::ostream &os) const
{
    const osg::Geode *geo;
    const osg::ShapeDrawable *draw;
    const osg::Cylinder *shape;

    if (!(geo = (const osg::Geode *)node()))
        return;
    if (!(draw = (const osg::ShapeDrawable *)geo->getDrawable(0)))
        return;
    if (!(shape = (const osg::Cylinder *)draw->getShape()))
        return;

    double radius = shape->getRadius();
    double height = shape->getHeight();

    os << "try:\n";
    os << "    ob = Blender.Object.Get('object_" << id() << "')\n";
    os << "except:\n";
    os << "    me = Mesh.Primitives.Cylinder(32," << 2. * radius << "," << height << ")\n";
    os << "    sc.objects.new(me,'object_" << id() << "')\n";

    printBlenderMaterial(os, draw->getColor(), id());
    os << "\n";
}


void VisBodyCylinder::exportToPovray(std::ofstream &ofs, PovrayMode mode) {
    osg::Geode *g = _node->asGeode();

    if (!g) {
        return;
    }

    if (mode == POVRAY_GEOM || mode == POVRAY_GEOMTRANSFROM) {
        for(int i=0;i<(int)g->getNumDrawables();i++) {
            osg::Drawable *d = g->getDrawable(i);
            if (d) {
                osg::Cylinder *b = (osg::Cylinder *)d->getShape();
                osg::ShapeDrawable *sd = (osg::ShapeDrawable *)d;
                Vec3 center(b->getCenter());
                double radius = b->getRadius();
                double height = b->getHeight();
                ofs << "cylinder { ";
                ofs << "<" << center[0] << "," << center[1] <<"," << center[2]-height/2.0 << ">,";
                ofs << "<" << center[0] << "," << center[1] <<"," << center[2]+height/2.0 << ">,";
                ofs << radius << "\n";

                ofs << " pigment {";
                povColor(ofs,sd->getColor());
                ofs << "} ";
                ofs << "}\n";
            }
        }
    } 

    if (mode == POVRAY_TRANSFORM || mode == POVRAY_GEOMTRANSFROM) {
        povTransformation(ofs,pos(),rot());
    } 

}


void VisBodyCylinder::exportToBlender(std::ofstream &ofs, const int idx) {
    osg::Geode *g = _node->asGeode();

    if (!g) {
        return;
    }

    for(int i=0;i<(int)g->getNumDrawables();i++) {
        osg::Drawable *d = g->getDrawable(i);
        if (d) {
            osg::Cylinder *b = (osg::Cylinder *)d->getShape();
            osg::ShapeDrawable *sd = (osg::ShapeDrawable *)d;
            Vec3 center(b->getCenter());
            double radius = b->getRadius();
            double height = b->getHeight();

            ofs << "try:\n";
            ofs << "    ob = Blender.Object.Get('object_" << idx << "')\n";
            ofs << "except:\n";
            ofs << "    me = Mesh.Primitives.Cylinder(32," << 2*radius <<","<< height << ")\n";
            ofs << "    sc.objects.new(me,'object_" << idx << "')\n";
            printBlenderMaterial(ofs,sd->getColor(),idx);
            ofs << "\n";
        }
    }
    ofs << "\n";

}

void VisBodyCylinder::_toPovrayFullShape(std::ostream &os,
                                      const osg::Shape *shape,
                                      const osg::ShapeDrawable *draw) const
{

    const osg::Cylinder *s = (const osg::Cylinder *)shape;
    Vec3 center(s->getCenter());
    double radius = s->getRadius();
    double height = s->getHeight();

    os << "cylinder { ";
    os << "<" << center[0] << "," << center[1] <<"," << center[2]-height/2.0 << ">,";
    os << "<" << center[0] << "," << center[1] <<"," << center[2]+height/2.0 << ">,";
    os << radius << "\n";

    os << " pigment {";
    povColor(os, draw->getColor());
    os << "} ";
    os << "}\n";
}



VisBodyCone::VisBodyCone(Scalar radius, Scalar height)
    : VisBodyShape()
{
    _setShape(new osg::Cone(Vec3(0., 0., 0.), radius, height));
}


void VisBodyCone::exportToPovray(std::ofstream &ofs, PovrayMode mode) {
    /*
    osg::Geode *g = _node->asGeode();

    if (!g) {
        return;
    }

    if (mode == POVRAY_GEOM || mode == POVRAY_GEOMTRANSFROM) {
        for(int i=0;i<(int)g->getNumDrawables();i++) {
            osg::Drawable *d = g->getDrawable(i);
            if (d) {
                osg::Cylinder *b = (osg::Cylinder *)d->getShape();
                osg::ShapeDrawable *sd = (osg::ShapeDrawable *)d;
                Vec3 center(b->getCenter());
                double radius = b->getRadius();
                double height = b->getHeight();
                ofs << "cylinder { ";
                ofs << "<" << center[0] << "," << center[1] <<"," << center[2]-height/2.0 << ">,";
                ofs << "<" << center[0] << "," << center[1] <<"," << center[2]+height/2.0 << ">,";
                ofs << radius << "\n";

                ofs << " pigment {";
                povColor(ofs,sd->getColor());
                ofs << "} ";
                ofs << "}\n";
            }
        }
    } 

    if (mode == POVRAY_TRANSFORM || mode == POVRAY_GEOMTRANSFROM) {
        ofs << "translate ";
        povCoords(ofs,pos());
        ofs << "\n";
    } 
    */
}

void VisBodyCone::exportToBlender(std::ofstream &ofs, const int idx) {
}

void VisBodyCone::_toPovrayFullShape(std::ostream &os,
                                       const osg::Shape *shape,
                                       const osg::ShapeDrawable *draw) const
{
    // TODO
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

    VisBodyTriMesh::setColor(osg::Vec4(0.5, 0.5, 0.5, 1.));
}

void VisBodyTriMesh::setColor(const osg::Vec4 &c)
{
    osg::Geometry *g = (osg::Geometry *)((osg::Geode *)_node.get())->getDrawable(0);
    osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
    color->push_back(c);
    g->setColorArray(color);
    g->setColorBinding(osg::Geometry::BIND_OVERALL);
}

void VisBodyTriMesh::exportToPovray(std::ofstream &ofs, PovrayMode mode) {
    
    osg::Geode *g = _node->asGeode();

    if (!g) {
        return;
    }

    if (mode == POVRAY_GEOM || mode == POVRAY_GEOMTRANSFROM) {
        ofs << "mesh {";
        for(int i=0;i<(int)g->getNumDrawables();i++) {
            osg::Drawable *d = g->getDrawable(i);
            if (d) {
                osg::Geometry *gm = d->asGeometry();
                if (gm) {
                    osg::Vec3Array *points = (osg::Vec3Array *)gm->getVertexArray();
                    osg::Geometry::DrawElementsList l;
                    gm->getDrawElementsList(l);
                    for(int j=0;j<(int)l.size();j++) {
                        ofs << "triangle{";
                        osg::DrawElementsUInt *dui = (osg::DrawElementsUInt *)l[j];
                        for(int k=0;k<(int)dui->getNumIndices();k++) {
                            osg::Vec3 v = (*points)[dui->index(k)];
                            ofs << "<" << v[0] <<","<<v[1]<<","<<v[2] << ">";
                            if (k < 2) {
                                ofs << ",";
                            }
                            
                        }    
                        ofs << "}\n";
                    }
                    osg::Vec4Array *colorArray = (osg::Vec4Array *)gm->getColorArray();
                    
                    ofs << "pigment {";
                    povColor(ofs,(*colorArray)[0]);
                    ofs << "}\n";
                }
            }
        }
        ofs << "}\n";
    } 

    if (mode == POVRAY_TRANSFORM || mode == POVRAY_GEOMTRANSFROM) {
        povTransformation(ofs,pos(),rot());
    } 
}

void VisBodyTriMesh::toBlender(std::ostream &os) const
{
    const osg::Geode *geo;
    const osg::Geometry *geom;
    const osg::Vec3Array *points;
    const osg::DrawElementsUInt *face;
    size_t i, j, len, len2, idx;
   
    if (!(geo = (const osg::Geode *)node()))
        return;
    if (!(geom = (const osg::Geometry *)geo->getDrawable(0)))
        return;
    if (!(points = (const osg::Vec3Array *)geom->getVertexArray()))
        return;

    len = geom->getNumPrimitiveSets();
    if (len == 0)
        return;

    os << "try:\n";
    os << "    ob = Blender.Object.Get('object_" << id() << "')\n";
    os << "except:\n";
    os << "    me = Mesh.New('mesh_" << id() << "')\n";

    idx = 0;
    for (i = 0; i < len; i++){
        face = (const osg::DrawElementsUInt *)geom->getPrimitiveSet(i);
        if (!face)
            continue;

        len2 = face->getNumIndices();
        for (j = 0; j < len2; j++){
            const osg::Vec3 &v = (*points)[face->index(j)];
            os << "    me.verts.extend([[" << v[0] << ", " << v[1] << ", " << v[2] << "]])\n";
        }

        os << "    me.faces.extend([[";
        for (j = 0; j < len2; j++){
            os << idx;
            if (j < len2 - 1)
                os << ", ";
            idx++;
        }
        os << "]])\n";
    }

    os << "    ob = sc.objects.new(me,'object_" << id() << "')\n";

    const osg::Vec4Array *colorArray = (const osg::Vec4Array *)geom->getColorArray();
    if (colorArray){
        printBlenderMaterial(os, (*colorArray)[0], id());
    }

    os << "\n";
}


void VisBodyTriMesh::exportToBlender(std::ofstream &ofs, const int idx) {
    
    osg::Geode *g = _node->asGeode();

    if (!g) {
        return;
    }

    ofs << "try:\n";
    ofs << "    ob = Blender.Object.Get('object_" << idx << "')\n";
    ofs << "except:\n";
    ofs << "    me = Mesh.New('mesh_" << idx << "')\n";

    for(int i=0;i<(int)g->getNumDrawables();i++) {
        osg::Drawable *d = g->getDrawable(i);
        if (d) {
            osg::Geometry *gm = d->asGeometry();
            if (gm) {
                int indices = 0;
                osg::Vec3Array *points = (osg::Vec3Array *)gm->getVertexArray();
                osg::Geometry::DrawElementsList l;
                gm->getDrawElementsList(l);
                for(int j=0;j<(int)l.size();j++) {
                    ofs << "    me.verts.extend([";
                    osg::DrawElementsUInt *dui = (osg::DrawElementsUInt *)l[j];
                    for(int k=0;k<(int)dui->getNumIndices();k++) {
                        osg::Vec3 v = (*points)[dui->index(k)];
                        ofs << "[" << v[0] <<","<<v[1]<<","<<v[2] << "]";
                        if (k < 2) {
                            ofs << ",";
                        }
                    }    
                    ofs << "])\n";
                }
                for(int j=0;j<(int)l.size();j++) {
                    ofs << "    me.faces.extend([[" << (indices*3+0) << "," << (indices*3+1) << "," << (indices*3+2) << "]])\n";
                    indices++;
                }
                ofs << "    ob = sc.objects.new(me,'object_" << idx << "')\n";
                osg::Vec4Array *colorArray = (osg::Vec4Array *)gm->getColorArray();
                printBlenderMaterial(ofs,(*colorArray)[0],idx);

            }
        }
    }
    ofs << "\n";
}


void VisBodyTriMesh::toPovrayObject(std::ostream &os) const
{
    os << "#declare object_" << id() << " = object {" << std::endl;
    _toPovrayMesh(os);
    os << "}" << std::endl; // object
}

void VisBodyTriMesh::toPovrayTr(std::ostream &os) const
{
    os << "object { object_" << id() << std::endl;
    povTransformation(os, pos(), rot());
    os << "}" << std::endl; // object
}

void VisBodyTriMesh::_toPovrayMesh(std::ostream &os) const
{
    const osg::Geode *geo;
    const osg::Geometry *geom;
    const osg::Vec3Array *points;
    const osg::DrawElementsUInt *face;
    size_t i, j, len, len2;
   
    if (!(geo = (const osg::Geode *)node()))
        return;
    if (!(geom = (const osg::Geometry *)geo->getDrawable(0)))
        return;
    if (!(points = (const osg::Vec3Array *)geom->getVertexArray()))
        return;


    os << "mesh {";
    len = geom->getNumPrimitiveSets();
    for (i = 0; i < len; i++){
        face = (const osg::DrawElementsUInt *)geom->getPrimitiveSet(i);
        if (!face)
            continue;

        os << "triangle {";

        len2 = face->getNumIndices();
        for (j = 0; j < len2; j++){
            const osg::Vec3 &v = (*points)[face->index(j)];
            os << "<" << v[0] << "," << v[1] << "," << v[2] << ">";
            if (j < len2 - 1)
                os << ", ";
        }

        os << "}" << std::endl; // triangle
    }

    const osg::Vec4Array *colorArray = (const osg::Vec4Array *)geom->getColorArray();
    if (colorArray){
        os << "pigment {";
        povColor(os, (*colorArray)[0]);
        os << "}\n";
    }

    os << "}" << std::endl; // mesh
}


}


