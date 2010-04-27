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
    void setPos(const Scalar x, const Scalar y, const Scalar z)
        { setPos(Vec3(x, y, z)); }
    void setPos(const Scalar *v) { setPos(Vec3(v[0], v[1], v[2])); }
    void setPos(const Vec3 *v) { setPos(*v); }
    void setPos(const Vec3 &v);

    void setRot(const Scalar x, const Scalar y, const Scalar z, const Scalar w)
        { setRot(Quat(x, y, z, w)); }
    void setRot(const Scalar *v) { setRot(Quat(v[0], v[1], v[2], v[3])); }
    void setRot(const Quat *q) { setRot(*q); }
    void setRot(const Quat &q);

    void setPosRot(const Vec3 *v, const Quat *q) { setPosRot(*v, *q); }
    void setPosRot(const Vec3 &v, const Quat &q);

    void getPos(Scalar *x, Scalar *y, Scalar *z) const;
    void getPos(Scalar *v) const { getPos(v, v + 1, v + 2); }
    void getPos(Vec3 *v) const;

    void getRot(Scalar *x, Scalar *y, Scalar *z, Scalar *w) const;
    void getRot(Scalar *v) const { getRot(v, v + 1, v + 2, v + 3); }
    void getRot(Quat *q) const;

  protected:
    void _set(VisObj *o, btCollisionShape *shape, Scalar mass);
};


class ObjCube : public Obj {
  public:
    ObjCube(Scalar width, Scalar mass);
};

class ObjBox : public Obj {
  public:
    ObjBox(Scalar x, Scalar y, Scalar z, Scalar mass);
};

}

#endif /* _SIM_OBJ_HPP_ */
