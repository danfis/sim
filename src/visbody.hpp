#ifndef _VIS_OBJ_HPP_
#define _VIS_OBJ_HPP_

#include <osg/PositionAttitudeTransform>
#include <osg/Geode>
#include <osgText/TextBase>

#include "math.hpp"

namespace sim {

/**
 * Visual representation of body.
 */
class VisBody {
  protected:
    osg::ref_ptr<osg::PositionAttitudeTransform> _root;
    osg::ref_ptr<osg::Group> _group;
    osg::ref_ptr<osg::Geode> _text;
    osg::ref_ptr<osg::Node> _node;

  public:
    VisBody();
    virtual ~VisBody() {}

    /**
     * Returns root node of osg scene graph.
     */
    osg::Node *rootNode() { return _root; }
    const osg::Node *rootNode() const { return _root; }

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

    virtual void setOsgText(osg::ref_ptr<osgText::TextBase> t);
    virtual void setText(const char *text, float size = 1.,
                         const osg::Vec4 &color = osg::Vec4(0., 0., 0., 1.)) = 0;

  protected:
    /**
     * Set top node of scene graph.
     *
     * This node will be connected to _root node which is
     * osg::PositionAttitudeTransform node.
     */
    void _setNode(osg::Node *n);
};


/**
 * Base class for visual bodies consisting only from single shape.
 */
class VisBodyShape : public VisBody {
  public:
    VisBodyShape() : VisBody() {}
    void setColor(const osg::Vec4 &c);
    void setText(const char *text, float size = 1.,
                 const osg::Vec4 &color = osg::Vec4(0., 0., 0., 1.));

  protected:
    /**
     * Set up shape. This is easier way to set up node then via _setNode().
     */
    void _setShape(osg::Shape *shape);
};


/**
 * VisBody with general Box shape.
 * Takes vector of three lengths of edges.
 */
class VisBodyBox : public VisBodyShape {
  public:
    VisBodyBox(Vec3 dim);
};

/**
 * VisBody with cube shape.
 * Takes width of edge as only argument.
 */
class VisBodyCube : public VisBodyBox {
  public:
    VisBodyCube(Scalar width) : VisBodyBox(Vec3(width, width, width)) {}
};

/**
 * Sphere shape. Takes radius as only argument.
 */
class VisBodySphere : public VisBodyShape {
  public:
    VisBodySphere(Scalar radius);
};

/**
 * Cylinder shape. Takes radius and height as arguments.
 */
class VisBodyCylinder : public VisBodyShape {
  public:
    VisBodyCylinder(Scalar radius, Scalar height);
};

} /* namespace sim */

#endif
