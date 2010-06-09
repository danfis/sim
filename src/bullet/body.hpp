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
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <LinearMath/btMotionState.h>

#include "sim/body.hpp"
#include "sim/visbody.hpp"

namespace sim {

namespace bullet {

// Forward declaration
class World;
class Body;

/**
 * Class using which is possible to change VisBody's position according to
 * Body's position effectively. See Bullet's btMotionState documentation
 * for more info.
 */

class BodyMotionState : public btMotionState {
    Body *_body;
    btTransform _world;

  public:
    BodyMotionState(Body *body,
                    const btTransform &start = btTransform::getIdentity())
        : _body(body), _world(start) {}

    void getWorldTransform(btTransform &worldTrans) const;
    void setWorldTransform(const btTransform &worldTrans);
};

class Body : public sim::Body {
  protected:
    struct shape_t {
        btCollisionShape *shape;
        btTransform tr;
        VisBody *vis;
        shape_t(btCollisionShape *s, VisBody *v)
            : shape(s), vis(v) {}
    };
    typedef std::map<int, shape_t *> _shapes_t;
    typedef _shapes_t::iterator _shapes_it_t;
    typedef _shapes_t::const_iterator _shapes_cit_t;

    World *_world;

    btRigidBody *_body;
    btCompoundShape *_shape;
    btMotionState *_motion_state;

    _shapes_t _shapes;
    int _next_id;

    Scalar _mass;
    Vec3 _local_inertia;

    Scalar _damping_lin, _damping_ang;

    friend class BodyMotionState;

  public:
    Body(World *w);
    virtual ~Body();

    /* \{ */
    btRigidBody *body() { return _body; }
    const btRigidBody *body() const { return _body; }
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
     * Adds cube to compound shape.
     * Visual representation can be provided.
     * Parameters pos_offset and rot_offset define relative offset of shape
     * from origin (0., 0., 0.).
     * Returns ID of shape that can be used lately for modifications.
     */
    int addCube(Scalar width,
                VisBody *vis = SIM_BODY_DEFAULT_VIS,
                const Vec3 &pos_offset = Vec3(0., 0., 0.),
                const Quat &rot_offset = Quat(0., 0., 0., 1.));
    int addBox(const Vec3 &dim,
               VisBody *vis = SIM_BODY_DEFAULT_VIS,
               const Vec3 &pos_offset = Vec3(0., 0., 0.),
               const Quat &rot_offset = Quat(0., 0., 0., 1.));
    int addSphere(Scalar radius,
                  VisBody *vis = SIM_BODY_DEFAULT_VIS,
                  const Vec3 &pos_offset = Vec3(0., 0., 0.),
                  const Quat &rot_offset = Quat(0., 0., 0., 1.));
    int addCylinderZ(Scalar radius, Scalar height,
                     VisBody *vis = SIM_BODY_DEFAULT_VIS,
                     const Vec3 &pos_offset = Vec3(0., 0., 0.),
                     const Quat &rot_offset = Quat(0., 0., 0., 1.));
    int addCylinderY(Scalar radius, Scalar height,
                     VisBody *vis = SIM_BODY_DEFAULT_VIS,
                     const Vec3 &pos_offset = Vec3(0., 0., 0.),
                     const Quat &rot_offset = Quat(0., 0., 0., 1.));
    int addCylinderX(Scalar radius, Scalar height,
                     VisBody *vis = SIM_BODY_DEFAULT_VIS,
                     const Vec3 &pos_offset = Vec3(0., 0., 0.),
                     const Quat &rot_offset = Quat(0., 0., 0., 1.));
    int addTriMesh(const sim::Vec3 *coords, size_t coords_len,
                   const unsigned int *indices, size_t indices_len,
                   VisBody *vis = SIM_BODY_DEFAULT_VIS,
                   const Vec3 &pos_offset = Vec3(0., 0., 0.),
                   const Quat &rot_offset = Quat(0., 0., 0., 1.));

    /**
     * Removes shape with ID that was returned by add.. method.
     */
    void rmShape(int ID);
    /* \} */

    /* \{ */
    void setMassCube(Scalar width, Scalar mass);
    void setMassBox(const Vec3 &dim, Scalar mass);
    void setMassSphere(Scalar radius, Scalar mass);
    void setMassCylinder(Scalar radius, Scalar height, Scalar mass);
    /* \} */

    /**
     * Returns pointer to VisBody of shape with specified ID.
     */
    VisBody *visBody(int ID);
    const VisBody *visBody(int ID) const;
    void visBodyAll(std::list<const VisBody *> *list) const;
    void visBodyAll(std::list<VisBody *> *list);

    void activate();
    void deactivate();

  protected:
    int _addShape(btCollisionShape *shape, VisBody *vis,
                  const Vec3 &pos, const Quat &rot);

    shape_t *shape(int ID);
    const shape_t *shape(int ID) const;

    void _applyShapesToVis();
    void _applyPosRotToBt();
};

class BodySimple : public Body {
  public:
    BodySimple(World *w) : Body(w) {}
    virtual ~BodySimple() {}

    VisBody *visBody() { return shape(1)->vis; }
    const VisBody *visBody() const { return shape(1)->vis; }
    void setVisBody(VisBody *vis) { shape(1)->vis = vis; }
};


/**
 * Body with general box shape.
 * Constructot takes vector with lengths of three box's edges and amount of
 * mass.
 */
class BodyBox : public BodySimple {
  public:
    BodyBox(World *w, Vec3 dim, Scalar mass, VisBody *vis = SIM_BODY_DEFAULT_VIS);
};

/**
 * Body with shape of cube.
 * Constructor takes width of edge and amount of mass.
 */
class BodyCube : public BodyBox {
  public:
    BodyCube(World *w, Scalar width, Scalar mass, VisBody *vis = SIM_BODY_DEFAULT_VIS);
};


/**
 * Body with sphere shape.
 * Contructor takes radius of sphere and mass.
 */
class BodySphere : public BodySimple {
  public:
    BodySphere(World *w, Scalar radius, Scalar mass, VisBody *vis = SIM_BODY_DEFAULT_VIS);
};

/**
 * Body with cylinder shape placed along Z axis.
 * Constructor takes radius and height of cylinder.
 */
class BodyCylinder : public BodySimple {
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
class BodyTriMesh : public BodySimple {
  public:
    BodyTriMesh(World *w, const sim::Vec3 *coords, size_t coords_len,
                const unsigned int *indices, size_t indices_len,
                VisBody *vis = SIM_BODY_DEFAULT_VIS);
};

} /* namespace bullet */

} /* namespace sim */

#endif /* _SIM_BULLET_OBJ_HPP_ */
