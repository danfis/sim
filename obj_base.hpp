#ifndef _SIM_OBJ_BASE_HPP_
#define _SIM_OBJ_BASE_HPP_

#include <osg/PositionAttitudeTransform>
#include <BulletDynamics/Dynamics/btRigidBody.h>

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
     * Pure virtual function which builds content of object.
     */
    virtual void build() = 0;

    /**
     * Tranfsforms object in 3D space.
     */
    void setPosition(float x, float y, float z);
    void setRotation(float x, float y, float z, float w);

  protected:
    /**
     * Set top node of scene graph.
     */
    void _setNode(osg::Node *n);
};


/**
 * Physical representation of object.
 */
class PhysObj {
    btRigidBody *_body;
    btCollisionShape *_shape;

  public:
    PhysObj() {}
    virtual ~PhysObj() {}

    /**
     * Pure virtual function which should build object.
     */
    virtual void build() = 0;
};


}

#endif /* _SIM_OBJ_BASE_HPP_ */
