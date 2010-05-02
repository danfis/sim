#ifndef _SIM_OBJ_HPP_
#define _SIM_OBJ_HPP_

#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <LinearMath/btMotionState.h>

#include "visbody.hpp"

namespace sim {

class BodyMotionState : public btMotionState {
    VisBody *_vis;
    btTransform _world, _offset;

  public:
    BodyMotionState(VisBody *vis,
                   const btTransform &start = btTransform::getIdentity(),
                   const btTransform &offset = btTransform::getIdentity())
        : _vis(vis), _world(start), _offset(offset) {}

    void getWorldTransform(btTransform &worldTrans) const;
    void setWorldTransform(const btTransform &worldTrans);
};

class Body {
    btRigidBody *_body;
    btCollisionShape *_shape;
    BodyMotionState *_motion_state;
    VisBody *_vis;

  public:
    Body() : _body(0), _shape(0), _motion_state(0), _vis(0) {}
    virtual ~Body();

    btRigidBody *body() { return _body; }
    const btRigidBody *body() const { return _body; }
    btCollisionShape *shape() { return _shape; }
    const btCollisionShape *shape() const { return _shape; }
    btMotionState *motionState() { return _motion_state; }
    const btMotionState *motionState() const { return _motion_state; }
    VisBody *visBody() { return _vis; }
    const VisBody *visBody() const { return _vis; }

    /**
     * Tranfsforms object in 3D space.
     */
    void setPos(const Scalar x, const Scalar y, const Scalar z)
        { setPos(Vec3(x, y, z)); }
    void setPos(const Vec3 *v) { setPos(*v); }
    void setPos(const Vec3 &v);

    void setRot(const Scalar x, const Scalar y, const Scalar z, const Scalar w)
        { setRot(Quat(x, y, z, w)); }
    void setRot(const Quat *q) { setRot(*q); }
    void setRot(const Quat &q);

    void setPosRot(const Vec3 *v, const Quat *q) { setPosRot(*v, *q); }
    void setPosRot(const Vec3 &v, const Quat &q);

    Vec3 pos() const;
    void pos(Scalar *x, Scalar *y, Scalar *z) const;

    Quat rot() const;
    void rot(Scalar *x, Scalar *y, Scalar *z, Scalar *w) const;

  protected:
    void _set(VisBody *o, btCollisionShape *shape, Scalar mass);
};


class BodyCube : public Body {
  public:
    BodyCube(Scalar width, Scalar mass);
};

class BodyBox : public Body {
  public:
    BodyBox(Vec3 dim, Scalar mass);
};

class BodySphere : public Body {
  public:
    BodySphere(Scalar radius, Scalar mass);
};

/**
 * Cylinder along Z axis.
 */
class BodyCylinder : public Body {
  public:
    BodyCylinder(Scalar radius, Scalar height, Scalar mass);
};

/**
 * Cylinder along X axis.
 */
class BodyCylinderX : public BodyCylinder {
  public:
    BodyCylinderX(Scalar radius, Scalar height, Scalar mass);
};

/**
 * Cylinder along Y axis.
 */
class BodyCylinderY : public BodyCylinder {
  public:
    BodyCylinderY(Scalar radius, Scalar height, Scalar mass);
};


}

#endif /* _SIM_OBJ_HPP_ */
