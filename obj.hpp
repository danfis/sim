#ifndef _SIM_OBJ_HPP_
#define _SIM_OBJ_HPP_

#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <LinearMath/btMotionState.h>

#include "visobj.hpp"

namespace sim {

class ObjMotionState : public btMotionState {
    VisObj *_vis;
    btTransform _world, _offset;

  public:
    ObjMotionState(VisObj *vis,
                   const btTransform &start = btTransform::getIdentity(),
                   const btTransform &offset = btTransform::getIdentity())
        : _vis(vis), _world(start), _offset(offset) {}

    void getWorldTransform(btTransform &worldTrans) const;
    void setWorldTransform(const btTransform &worldTrans);
};

class Obj {
    btRigidBody *_body;
    btCollisionShape *_shape;
    ObjMotionState *_motion_state;
    VisObj *_vis;

  public:
    Obj() : _body(0), _shape(0), _motion_state(0), _vis(0) {}
    virtual ~Obj();

    btRigidBody *body() { return _body; }
    const btRigidBody *body() const { return _body; }
    btCollisionShape *shape() { return _shape; }
    const btCollisionShape *shape() const { return _shape; }
    btMotionState *motionState() { return _motion_state; }
    const btMotionState *motionState() const { return _motion_state; }
    VisObj *visObj() { return _vis; }
    const VisObj *visObj() const { return _vis; }

    /**
     * Tranfsforms object in 3D space.
     */
    void setPos(float x, float y, float z);
    void setPos(float *v) { setPos(v[0], v[1], v[2]); }
    void setRot(float x, float y, float z, float w);
    void setRot(float *v) { setRot(v[0], v[1], v[2], v[3]); }
    void setPosRot(float x, float y, float z,
                   float rx, float ry, float rz, float rw);

    void getPos(float *x, float *y, float *z);
    void getPos(float *v) { getPos(v, v + 1, v + 2); }
    void getRot(float *x, float *y, float *z, float *w);
    void getRot(float *v) { getRot(v, v + 1, v + 2, v + 3); }

  protected:
    void _set(VisObj *o, btCollisionShape *shape, float mass);
};

}

#endif /* _SIM_OBJ_HPP_ */
