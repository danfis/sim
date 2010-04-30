#ifndef _VIS_OBJ_HPP_
#define _VIS_OBJ_HPP_

#include <osg/PositionAttitudeTransform>

#include "math.hpp"

namespace sim {

/**
 * Visual reprezentation of object.
 */
class VisObj {
  protected:
    osg::PositionAttitudeTransform *_root;
    osg::Node *_node;

  public:
    VisObj();
    virtual ~VisObj() {}

    /**
     * Returns root node of osg scene graph.
     */
    osg::Node *rootNode() { return _root; }
    const osg::Node *rootNode() const { return _root; }

    Vec3 pos() const { return _root->getPosition(); }
    void pos(Scalar *x, Scalar *y, Scalar *z) const;
    Quat rot() const { return _root->getAttitude(); }
    void rot(Scalar *x, Scalar *y, Scalar *z, Scalar *w) const;

    /**
     * Tranfsforms object in 3D space.
     */
    void setPos(const Scalar x, const Scalar y, const Scalar z)
        { setPos(Vec3(x, y, z)); }
    void setPos(const Vec3 *v) { setPos(*v); }
    void setPos(const Vec3 &v);

    void setRot(const Scalar x, const Scalar y, const Scalar z, const Scalar w)
        { setRot(Quat(x, y, z, w)); }
    void setRot(const Quat *v) { setRot(*v); }
    void setRot(const Quat &v);

    void setPosRot(const Vec3 *v, const Quat *q) { setPosRot(*v, *q); }
    void setPosRot(const Vec3 &v, const Quat &q);

    virtual void setColor(const osg::Vec4 &c) {}

  protected:
    /**
     * Set top node of scene graph.
     */
    void _setNode(osg::Node *n);
};


class VisObjShape : public VisObj {
  public:
    VisObjShape() : VisObj() {}
    void setColor(const osg::Vec4 &c);

  protected:
    void _setShape(osg::Shape *shape);
};


class VisObjCube : public VisObjShape {
  public:
    VisObjCube(Scalar width);
};

class VisObjBox : public VisObjShape {
  public:
    VisObjBox(Vec3 dim);
};

class VisObjSphere : public VisObjShape {
  public:
    VisObjSphere(Scalar radius);
};

class VisObjCylinder : public VisObjShape {
  public:
    VisObjCylinder(Scalar radius, Scalar height);
};

} /* namespace sim */

#endif
