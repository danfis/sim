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
    void setPosition(float x, float y, float z);
    void setPosition(float *v) { setPosition(v[0], v[1], v[2]); }
    void setRotation(float x, float y, float z, float w);
    void setRotation(float *v) { setRotation(v[0], v[1], v[2], v[3]); }

    void getPosition(float *x, float *y, float *z);
    void getPosition(float *v) { getPosition(v, v + 1, v + 2); }
    void getRotation(float *x, float *y, float *z, float *w);
    void getRotation(float *v) { getRotation(v, v + 1, v + 2, v + 3); }

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
