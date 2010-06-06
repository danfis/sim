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

#ifndef _SIM_BULLET_WORLD_HPP_
#define _SIM_BULLET_WORLD_HPP_

#include <BulletCollision/CollisionDispatch/btCollisionConfiguration.h>
#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>
#include <BulletCollision/BroadphaseCollision/btBroadphaseInterface.h>
#include <BulletDynamics/ConstraintSolver/btConstraintSolver.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>

#include "sim/visworld.hpp"
#include "sim/world.hpp"
#include "sim/bullet/body.hpp"
#include "sim/bullet/joint.hpp"
#include "sim/bullet/actuator.hpp"
#include "sim/bullet/collision_detection.hpp"

namespace sim {

namespace bullet {


/**
 * Physical representation world.
 */
class World : public sim::World {
  protected:
    btCollisionConfiguration *_coll_conf;
    CollisionDispatcher *_dispatch;
    btBroadphaseInterface *_broadphase;
    btConstraintSolver *_solver;
    btDynamicsWorld *_world;

  public:
    World();
    virtual ~World();

    /**
     * Initializes world.
     */
    void init();

    /**
     * Finalize world.
     */
    void finish();

    /**
     * Performes one step.
     */
    void step(const sim::Time &time, unsigned int substeps = 1);

    bool done();

    sim::Body *createBodyCube(Scalar width, Scalar mass, VisBody *vis = SIM_BODY_DEFAULT_VIS);
    sim::Body *createBodyBox(Vec3 dim, Scalar mass, VisBody *vis = SIM_BODY_DEFAULT_VIS);
    sim::Body *createBodySphere(Scalar radius, Scalar mass, VisBody *vis = SIM_BODY_DEFAULT_VIS);
    sim::Body *createBodyCylinderX(Scalar radius, Scalar height, Scalar mass,
                                   VisBody *vis = SIM_BODY_DEFAULT_VIS);
    sim::Body *createBodyCylinderY(Scalar radius, Scalar height, Scalar mass,
                                   VisBody *vis = SIM_BODY_DEFAULT_VIS);
    sim::Body *createBodyCylinderZ(Scalar radius, Scalar height, Scalar mass,
                                   VisBody *vis = SIM_BODY_DEFAULT_VIS);
    sim::Body *createBodyTriMesh(const Vec3 *coords, size_t coords_len,
                                 const unsigned int *indices, size_t indices_len,
                                 VisBody *vis = SIM_BODY_DEFAULT_VIS);

    sim::Joint *createJointFixed(sim::Body *oA, sim::Body *oB);
    sim::Joint *createJointHinge(sim::Body *A, sim::Body *oB,
                                 const Vec3 &anchor, const Vec3 &axis);
    sim::Joint *createJointHinge2(sim::Body *A, sim::Body *oB, const Vec3 &anchor,
                                  const Vec3 &axis1, const Vec3 &axis2);

    sim::ActuatorWheelCylinderX *createActuatorWheelCylinderX
                                    (Scalar radius, Scalar height, Scalar mass);


    /**
     * Adds joint to world. This function is used by .activate() method of
     * Joint and should NOT be used directly.
     */
    void addJoint(Joint *j);

    /**
     * Adds body to world. This function is used by .activate() method of
     * Body and should NOT be used directly.
     */
    void addBody(Body *obj);
};

} /* namespace bullet */

} /* namespace sim */

#endif /* _SIM_BULLET_WORLD_HPP_ */

