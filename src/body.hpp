#ifndef _SIM_OBJ_HPP_
#define _SIM_OBJ_HPP_

#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <LinearMath/btMotionState.h>

#include "visbody.hpp"

namespace sim {

// Forward declaration
class World;

/**
 * Class using which is possible to change VisBody's position according to
 * Body's position effectively. See Bullet's btMotionState documentation
 * for more info.
 */
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

/**
 * Struct with collision info.
 * This struct is used internally by collicion detector and should be set
 * carefully.
 *
 * This will be probably changed in future - for now it is fastest solution
 * how to solve some collision detection problems.
 */
struct BodyCollisionInfo {
    /**
     * If two bodies have same value of .dont_collide_id and this value
     * isn't zero collision is not performed.
     */
    unsigned long dont_collide_id;

    BodyCollisionInfo() : dont_collide_id(0) {}
};

/**
 * Class representing physical body.
 */
class Body {
  protected:
    World *_world;

    btRigidBody *_body;
    btCollisionShape *_shape;
    BodyMotionState *_motion_state;
    VisBody *_vis;

    BodyCollisionInfo _collision_info;

  public:
    Body(World *w);
    virtual ~Body();

    btRigidBody *body() { return _body; }
    const btRigidBody *body() const { return _body; }
    btCollisionShape *shape() { return _shape; }
    const btCollisionShape *shape() const { return _shape; }
    btMotionState *motionState() { return _motion_state; }
    const btMotionState *motionState() const { return _motion_state; }
    VisBody *visBody() { return _vis; }
    const VisBody *visBody() const { return _vis; }
    World *world() { return _world; }
    const World *world() const { return _world; }

    /**
     * Set position of body in 3D space.
     */
    virtual void setPos(const Vec3 &v);
    virtual void setPos(const Vec3 *v) { setPos(*v); }
    virtual void setPos(const Scalar x, const Scalar y, const Scalar z)
        { setPos(Vec3(x, y, z)); }

    /**
     * Set rotation of body in 3D space.
     */
    virtual void setRot(const Quat &q);
    virtual void setRot(const Quat *q) { setRot(*q); }
    virtual void setRot(const Scalar x, const Scalar y, const Scalar z, const Scalar w)
        { setRot(Quat(x, y, z, w)); }

    /**
     * Set position and rotation of body at once.
     */
    virtual void setPosRot(const Vec3 &v, const Quat &q);
    virtual void setPosRot(const Vec3 *v, const Quat *q) { setPosRot(*v, *q); }

    /**
     * Returns position of body.
     */
    virtual Vec3 pos() const;
    virtual void pos(Scalar *x, Scalar *y, Scalar *z) const;

    /**
     * Returns rotation of body.
     */
    virtual Quat rot() const;
    virtual void rot(Scalar *x, Scalar *y, Scalar *z, Scalar *w) const;

    /**
     * Activates body in world - world will realize this body.
     */
    virtual void activate();

    /**
     * Deactives body - body will be removed from a world.
     */
    virtual void deactivate();


    const BodyCollisionInfo &collInfo() { return _collision_info; }
    void collSetDontCollideId(unsigned long id)
        { _collision_info.dont_collide_id = id; }

  protected:
    /**
     * Sets internals of Body.
     *
     * btRigidBody is created according to shape and VisBody is connected
     * with BodyMotionState to update its position when needed.
     */
    virtual void _set(VisBody *o, btCollisionShape *shape, Scalar mass);
};


/**
 * Body with shape of cube.
 * Constructor takes width of edge and amount of mass.
 */
class BodyCube : public Body {
  public:
    BodyCube(World *w, Scalar width, Scalar mass);
};

/**
 * Body with general box shape.
 * Constructot takes vector with lengths of three box's edges and amount of
 * mass.
 */
class BodyBox : public Body {
  public:
    BodyBox(World *w, Vec3 dim, Scalar mass);
};

/**
 * Body with sphere shape.
 * Contructor takes radius of sphere and mass.
 */
class BodySphere : public Body {
  public:
    BodySphere(World *w, Scalar radius, Scalar mass);
};

/**
 * Body with cylinder shape placed along Z axis.
 * Constructor takes radius and height of cylinder.
 */
class BodyCylinder : public Body {
  public:
    BodyCylinder(World *w, Scalar radius, Scalar height, Scalar mass);
};

/**
 * Body with shape of cylinder placed along X axis.
 */
class BodyCylinderX : public BodyCylinder {
  public:
    BodyCylinderX(World *w, Scalar radius, Scalar height, Scalar mass);
};

/**
 * Body with shape of cylinder placed along Y axis.
 */
class BodyCylinderY : public BodyCylinder {
  public:
    BodyCylinderY(World *w, Scalar radius, Scalar height, Scalar mass);
};


}

#endif /* _SIM_OBJ_HPP_ */
