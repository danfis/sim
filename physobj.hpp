#ifndef _SIM_PHYS_OBJ_HPP_
#define _SIM_PHYS_OBJ_HPP_

#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <LinearMath/btMotionState.h>

namespace sim {

/**
 * Physical representation of object.
 */
class PhysObj {
  protected:
    btRigidBody *_body;
    btCollisionShape *_shape;
    btMotionState *_motion_state;

  public:
    PhysObj();
    virtual ~PhysObj();

    btRigidBody *body() { return _body; }
    const btRigidBody *body() const { return _body; }
    btCollisionShape *shape() { return _shape; }
    const btCollisionShape *shape() const { return _shape; }
    btMotionState *motionState() { return _motion_state; }
    const btMotionState *motionState() const { return _motion_state; }

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
    void _setBody(btRigidBody *body) { _body = body; }
    void _setShape(btCollisionShape *shape) { _shape = shape; }
};


class PhysObjCube : public PhysObj {
  public:
    PhysObjCube(float width, float mass);
};

class PhysObjBox : public PhysObj {
  public:
    PhysObjBox(float x, float y, float z, float mass);
};


}

#endif /* _SIM_PHYS_OBJ_HPP_ */
