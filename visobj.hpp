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

    void getPos(Scalar *x, Scalar *y, Scalar *z) const;
    void getPos(Scalar *v) const { getPos(v, v + 1, v + 2); }
    void getPos(Vec3 *v) const;
    void getRot(Scalar *x, Scalar *y, Scalar *z, Scalar *w) const;
    void getRot(Scalar *v) const { getRot(v, v + 1, v + 2, v + 3); }
    void getRot(Quat *q) const;

  protected:
    /**
     * Set top node of scene graph.
     */
    void _setNode(osg::Node *n);
};



class VisObjCube : public VisObj {
  public:
    VisObjCube(Scalar width);
};

class VisObjBox : public VisObj {
  public:
    VisObjBox(Scalar x, Scalar y, Scalar z);
};

class VisObjSphere : public VisObj {
  public:
    VisObjSphere(Scalar radius);
};

class VisObjCylinder : public VisObj {
  public:
    VisObjCylinder(Scalar radius, Scalar height);
};

} /* namespace sim */

#endif
