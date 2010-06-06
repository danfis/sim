/***
 * sim
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
 *
 *  This file is part of sim.
 *
 *  sim is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation; either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  sim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _SIM_BULLET_OBJ_HPP_
#define _SIM_BULLET_OBJ_HPP_

#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <LinearMath/btMotionState.h>

#include "sim/body.hpp"
#include "sim/visbody.hpp"

namespace sim {

namespace bullet {

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
 * Class representing physical body.
 */
class Body : public sim::Body {
  protected:
    World *_world;

    btRigidBody *_body;
    btCollisionShape *_shape;
    BodyMotionState *_motion_state;
    VisBody *_vis;

    Scalar _damping_lin, _damping_ang;

  public:
    Body(World *w);
    virtual ~Body();

    /* \{ */
    btRigidBody *body() { return _body; }
    const btRigidBody *body() const { return _body; }
    btCollisionShape *shape() { return _shape; }
    const btCollisionShape *shape() const { return _shape; }
    btMotionState *motionState() { return _motion_state; }
    const btMotionState *motionState() const { return _motion_state; }
    World *world() { return _world; }
    const World *world() const { return _world; }
    /* \} */

    const Scalar &dampingLin() const { return _damping_lin; }
    const Scalar &dampingAng() const { return _damping_ang; }
    void setDampingLin(Scalar v) { _damping_lin = v; }
    void setDampingAng(Scalar v) { _damping_ang = v; }

    /* \{ */
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
    /* \} */

    /* \{ */
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
    /* \} */

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
    BodyCube(World *w, Scalar width, Scalar mass, VisBody *vis = SIM_BODY_DEFAULT_VIS);
};

/**
 * Body with general box shape.
 * Constructot takes vector with lengths of three box's edges and amount of
 * mass.
 */
class BodyBox : public Body {
  public:
    BodyBox(World *w, Vec3 dim, Scalar mass, VisBody *vis = SIM_BODY_DEFAULT_VIS);
};

/**
 * Body with sphere shape.
 * Contructor takes radius of sphere and mass.
 */
class BodySphere : public Body {
  public:
    BodySphere(World *w, Scalar radius, Scalar mass, VisBody *vis = SIM_BODY_DEFAULT_VIS);
};

/**
 * Body with cylinder shape placed along Z axis.
 * Constructor takes radius and height of cylinder.
 */
class BodyCylinder : public Body {
  public:
    BodyCylinder(World *w, Scalar radius, Scalar height, Scalar mass,
                 VisBody *vis = SIM_BODY_DEFAULT_VIS);
};

/**
 * Body with shape of cylinder placed along X axis.
 */
class BodyCylinderX : public BodyCylinder {
  public:
    BodyCylinderX(World *w, Scalar radius, Scalar height, Scalar mass,
                 VisBody *vis = SIM_BODY_DEFAULT_VIS);
};

/**
 * Body with shape of cylinder placed along Y axis.
 */
class BodyCylinderY : public BodyCylinder {
  public:
    BodyCylinderY(World *w, Scalar radius, Scalar height, Scalar mass,
                 VisBody *vis = SIM_BODY_DEFAULT_VIS);
};

/**
 * Body represented by triangular mesh. This Body is static and thus can't
 * have any mass.
 */
class BodyTriMesh : public Body {
  public:
    BodyTriMesh(World *w, const sim::Vec3 *coords, size_t coords_len,
                const unsigned int *indices, size_t indices_len,
                VisBody *vis = SIM_BODY_DEFAULT_VIS);
};


class BodyConvexHull : public Body {
  public:
    BodyConvexHull(World *w, const sim::Vec3 *points, size_t points_len,
                   Scalar mass, VisBody *vis = SIM_BODY_DEFAULT_VIS);
};
} /* namespace bullet */

} /* namespace sim */

#endif /* _SIM_BULLET_OBJ_HPP_ */
