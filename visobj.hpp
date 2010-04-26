#ifndef _VIS_OBJ_HPP_
#define _VIS_OBJ_HPP_

#include <osg/PositionAttitudeTransform>

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
    void setPos(float x, float y, float z);
    void setPos(float *v) { setPos(v[0], v[1], v[2]); }
    void setRot(float x, float y, float z, float w);
    void setRot(float *v) { setRot(v[0], v[1], v[2], v[3]); }

    void getPos(float *x, float *y, float *z);
    void getPos(float *v) { getPos(v, v + 1, v + 2); }
    void getRot(float *x, float *y, float *z, float *w);
    void getRot(float *v) { getRot(v, v + 1, v + 2, v + 3); }

  protected:
    /**
     * Set top node of scene graph.
     */
    void _setNode(osg::Node *n);
};



class VisObjCube : public VisObj {
  public:
    VisObjCube(float width);
};

class VisObjSphere : public VisObj {
  public:
    VisObjSphere(float radius);
};

} /* namespace sim */

#endif
