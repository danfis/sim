/***
 * sim
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
 *                   Vojta Vonasek <vonasek@labe.felk.cvut.cz>
 *
 *  This file is part of sim.
 *
 *  sim is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation; either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  sim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _SIM_VIS_BODY_HPP_
#define _SIM_VIS_BODY_HPP_

#include <osg/PositionAttitudeTransform>
#include <osg/Geode>
#include <osgText/TextBase>
#include <osg/Vec4f>
#include <osg/ShapeDrawable>
#include "math.hpp"
#include <fstream>

namespace sim {


/**
 * Visual representation of body.
 */
class VisBody {
  protected:
    unsigned long _id; //!< Unique ID of object
    osg::ref_ptr<osg::PositionAttitudeTransform> _root;
    osg::ref_ptr<osg::Group> _group;
    osg::ref_ptr<osg::Geode> _text;
    osg::ref_ptr<osg::Node> _node;
    Vec3 _offset;

  public:
    /**
     * Used by exportToPovray() method. Defines which property of body
     * should be exported.
     */
    enum PovrayMode {
        POVRAY_GEOM = 0, //!< export only geometry of object 
        POVRAY_TRANSFORM, //!< export only transformation matrix (rotation+translation) of the object
        POVRAY_GEOMTRANSFROM //!< export both geometry and transformation
    };

  public:
    VisBody();
    virtual ~VisBody() {}

    /**
     * Returns unique ID assigned to body.
     */
    unsigned long id() const { return _id; }

    /**
     * Returns root node of osg scene graph.
     */
    osg::Node *rootNode() { return _root; }
    const osg::Node *rootNode() const { return _root; }

    osg::Node *node() { return _node; }
    const osg::Node *node() const { return _node; }

    /**
     * Returns position of root node.
     */
    Vec3 pos() const { return _root->getPosition(); }
    void pos(Scalar *x, Scalar *y, Scalar *z) const;

    /**
     * Returns rotation of root node.
     */
    Quat rot() const { return _root->getAttitude(); }
    void rot(Scalar *x, Scalar *y, Scalar *z, Scalar *w) const;

    Vec3 &offset() { return _offset; }
    const Vec3 &offset() const { return _offset; }
    void setOffset(const Vec3 &o) { _offset = o; }

    /**
     * Set position of body.
     */
    void setPos(const Vec3 *v) { setPos(*v); }
    void setPos(const Vec3 &v);
    void setPos(const Scalar x, const Scalar y, const Scalar z)
        { setPos(Vec3(x, y, z)); }

    /**
     * Set rotation of body.
     */
    void setRot(const Quat &v);
    void setRot(const Quat *v) { setRot(*v); }
    void setRot(const Scalar x, const Scalar y, const Scalar z, const Scalar w)
        { setRot(Quat(x, y, z, w)); }

    /**
     * Set position and rotation at once.
     */
    void setPosRot(const Vec3 &v, const Quat &q);
    void setPosRot(const Vec3 *v, const Quat *q) { setPosRot(*v, *q); }

    /**
     * Set color of body.
     */
    virtual void setColor(const osg::Vec4 &c) {}
    virtual void setColor(float r, float g, float b, float a)
        { setColor(osg::Vec4(r, g, b, a)); }

	const osg::Vec4& color() const;

    virtual void setTexture(const std::string &fn) {}

    virtual void setOsgText(osg::ref_ptr<osgText::TextBase> t);
    virtual void setText(const char *text, float size = 1.,
                         const osg::Vec4 &color = osg::Vec4(0., 0., 0., 1.)) {}


	virtual void exportToPovray(std::ofstream &ofs, PovrayMode mode) {}
	virtual void exportToBlender(std::ofstream &ofs, const int idx) {}

    /**
     * Exports VisBody as full povray object (with correct transformation
     * etc...).
     */
    virtual void toPovrayFull(std::ostream &os) const
    { toPovrayObject(os);
      toPovrayTr(os); }

    /**
     * Exports VisBody as povray object without any transformations and as
     * declaration of object:
     *      #declare object_[id] = object { ... }
     */
    virtual void toPovrayObject(std::ostream &os) const {}

    /**
     * Exports VisBody as transformation of already declared object:
     *      object { object_[id] [transformation] }
     */
    virtual void toPovrayTr(std::ostream &os) const {}

    /**
     * Exports VisBody to blender format (used by Blender component).
     */
    virtual void toBlender(std::ostream &os) const {}

  protected:
    /**
     * Set top node of scene graph.
     *
     * This node will be connected to _root node which is
     * osg::PositionAttitudeTransform node.
     */
    void _setNode(osg::Node *n);


    /**
     * Returns unique ID of body.
     */
    static unsigned long _getUniqueID();
    static unsigned long _last_id;

    void povCoords(std::ostream &os, const osg::Vec3 &vec) const;
    void povTransformation(std::ostream &ofs, const osg::Vec3 &position,
                           const osg::Quat &rotation) const;
    void povColor(std::ostream &os, const osg::Vec4 &color) const;

  public:
    static unsigned long lastId() { return _last_id; }
};


/**
 * Base class for visual bodies consisting only from single shape.
 */
class VisBodyShape : public VisBody {
  public:
    VisBodyShape() : VisBody() {}
    void setColor(const osg::Vec4 &c);
    void setTexture(const std::string &fn);
    void setText(const char *text, float size = 1.,
                 const osg::Vec4 &color = osg::Vec4(0., 0., 0., 1.));


    void toPovrayObject(std::ostream &os) const;
    void toPovrayTr(std::ostream &os) const;

  protected:
    /**
     * Set up shape. This is easier way to set up node then via _setNode().
     */
    void _setShape(osg::Shape *shape);

    virtual void _toPovrayFullShape(std::ostream &os,
                                    const osg::Shape *shape,
                                    const osg::ShapeDrawable *draw) const = 0;
};


/**
 * VisBody with general Box shape.
 * Takes vector of three lengths of edges.
 */
class VisBodyBox : public VisBodyShape {
  public:
    VisBodyBox(Vec3 dim);
	void exportToPovray(std::ofstream &ofs, PovrayMode mode);
	void exportToBlender(std::ofstream &ofs, const int idx);

    void toBlender(std::ostream &os) const;

  protected:
    void _toPovrayFullShape(std::ostream &os,
                            const osg::Shape *shape,
                            const osg::ShapeDrawable *draw) const;
};

/**
 * VisBody with cube shape.
 * Takes width of edge as only argument.
 */
class VisBodyCube : public VisBodyBox {
  public:
    VisBodyCube(Scalar width) : VisBodyBox(Vec3(width, width, width)) {}
	void exportToPovray(std::ofstream &ofs, PovrayMode mode);
	void exportToBlender(std::ofstream &ofs, const int idx);
};

/**
 * Sphere shape. Takes radius as only argument.
 */
class VisBodySphere : public VisBodyShape {
  public:
    VisBodySphere(Scalar radius);
	void exportToPovray(std::ofstream &ofs, PovrayMode mode);
	void exportToBlender(std::ofstream &ofs, const int idx);

    void toBlender(std::ostream &os) const;

  protected:
    void _toPovrayFullShape(std::ostream &os,
                            const osg::Shape *shape,
                            const osg::ShapeDrawable *draw) const;
};

/**
 * Cylinder shape. Takes radius and height as arguments.
 */
class VisBodyCylinder : public VisBodyShape {
  public:
    VisBodyCylinder(Scalar radius, Scalar height);
	void exportToPovray(std::ofstream &ofs, PovrayMode mode);
	void exportToBlender(std::ofstream &ofs, const int idx);

    void toBlender(std::ostream &os) const;

  protected:
    void _toPovrayFullShape(std::ostream &os,
                            const osg::Shape *shape,
                            const osg::ShapeDrawable *draw) const;
};

/**
 * Cone shape. Takes radius and height.
 */
class VisBodyCone : public VisBodyShape {
  public:
    VisBodyCone(Scalar radius, Scalar height);
	void exportToPovray(std::ofstream &ofs, PovrayMode mode);
	void exportToBlender(std::ofstream &ofs, const int idx);

    void toBlender(std::ostream &os) const { /* TODO */ }

  protected:
    void _toPovrayFullShape(std::ostream &os,
                            const osg::Shape *shape,
                            const osg::ShapeDrawable *draw) const;
};



class VisBodyTriMesh : public VisBody {
  public:
    VisBodyTriMesh(const sim::Vec3 *coords, size_t coords_len,
                   const unsigned int *indices, size_t indices_len);
    void setColor(const osg::Vec4 &c);
	void exportToPovray(std::ofstream &ofs, PovrayMode mode);
	void exportToBlender(std::ofstream &ofs, const int idx);

    void toBlender(std::ostream &os) const;

    void toPovrayObject(std::ostream &os) const;
    void toPovrayTr(std::ostream &os) const;

  protected:
    void _toPovrayMesh(std::ostream &os) const;
};

} /* namespace sim */

#endif /* _SIM_VIS_BODY_HPP_ */
