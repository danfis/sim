#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osg/Array>
#include <osgText/Text>
#include <osgUtil/SmoothingVisitor>
#include <osg/Material>
#include <osgDB/ReadFile>

#include "visbody.hpp"
#include "msg.hpp"

namespace sim {

void povCoords(std::ofstream &ofs, const osg::Vec3 &vec) {
	ofs << " <" << vec[0] << "," << vec[1] << "," << vec[2] << "> ";
}

void povTransformation(std::ofstream &ofs, const osg::Vec3 &position, const osg::Quat &rotation) {
	osg::Matrixd m;
	m.makeIdentity();
	m.makeRotate(rotation);
	m.makeTranslate(position);
	ofs << "matrix <";
	ofs << m(0,0) << "," << m(0,1) <<","<<m(0,2) <<",\n";
	ofs << m(1,0) << "," << m(1,1) <<","<<m(1,2) <<",\n";
	ofs << m(2,0) << "," << m(2,1) <<","<<m(2,2) <<",\n";
	ofs << m(3,0) << "," << m(3,1) <<","<<m(3,2) <<">\n";
}

void povColor(std::ofstream &ofs, const osg::Vec4 &color) {
	//ofs << "color rgb <" << color[0]<<","<<color[1]<<","<<color[2]<<"> ";
	ofs << "color rgbf <" << color[0]<<","<<color[1]<<","<<color[2]<<"," << (1-color[3]) << "> ";
}


VisBody::VisBody()
    : _node(0), _offset(0., 0., 0.)
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

void VisBody::setOsgText(osg::ref_ptr<osgText::TextBase> t)
{
    if (_text.valid()){
        _text->removeDrawables(0);
    }

    if (t.valid()){
        _text->addDrawable(t);
    }
}

void VisBody::exportToPovray(std::ofstream &ofs, const TPovrayMode mode) {
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

void VisBodyShape::exportToPovray(std::ofstream &ofs, const TPovrayMode mode) {

}


VisBodyBox::VisBodyBox(Vec3 dim)
    : VisBodyShape()
{
    _setShape(new osg::Box(osg::Vec3(0., 0., 0.), dim.x(), dim.y(), dim.z()));
}

void VisBodyBox::exportToPovray(std::ofstream &ofs, const TPovrayMode mode) {
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

void VisBodyCube::exportToPovray(std::ofstream &ofs, const TPovrayMode mode) {
	osg::Geode *g = _node->asGeode();

	if (!g) {
		return;
	}

	if (mode == POVRAY_GEOM || mode == POVRAY_GEOMTRANSFROM) {
		for(int i=0;i<(int)g->getNumDrawables();i++) {
			osg::Drawable *d = g->getDrawable(i);
			if (d) {
				osg::Box *b = (osg::Box *)d->getShape();
				Vec3 center(b->getCenter());
				Vec3 lengths(b->getHalfLengths());
				ofs << "box { <" << center[0]-lengths[0] << "," << center[1]-lengths[1] << "," << center[2]-lengths[2] << ">,";
				ofs << " <" << center[0]+lengths[0] << "," << center[1]+lengths[1] << "," << center[2]+lengths[2] << "> ";
				ofs << "}\n";
			}
		}
	} 

	if (mode == POVRAY_TRANSFORM || mode == POVRAY_GEOMTRANSFROM) {
		ofs << "translate ";
		povCoords(ofs,pos());
		ofs << "\n";
	} 

}




VisBodySphere::VisBodySphere(Scalar radius)
    : VisBodyShape()
{
    _setShape(new osg::Sphere(Vec3(0., 0., 0.), radius));
}


void VisBodySphere::exportToPovray(std::ofstream &ofs, const TPovrayMode mode) {
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
		ofs << "translate ";
		povCoords(ofs,pos());
		ofs << "\n";
	} 

}


VisBodyCylinder::VisBodyCylinder(Scalar radius, Scalar height)
    : VisBodyShape()
{
    _setShape(new osg::Cylinder(Vec3(0., 0., 0.), radius, height));
}

void VisBodyCylinder::exportToPovray(std::ofstream &ofs, const TPovrayMode mode) {
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

}


VisBodyCone::VisBodyCone(Scalar radius, Scalar height)
    : VisBodyShape()
{
    _setShape(new osg::Cone(Vec3(0., 0., 0.), radius, height));
}


void VisBodyCone::exportToPovray(std::ofstream &ofs, const TPovrayMode mode) {
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

void VisBodyTriMesh::exportToPovray(std::ofstream &ofs, const TPovrayMode mode) {
	
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
		ofs << "translate ";
		povCoords(ofs,pos());
		ofs << "\n";
	} 
}





}
