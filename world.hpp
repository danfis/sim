#ifndef _SIM_WORLD_HPP_
#define _SIM_WORLD_HPP_

#include <BulletCollision/CollisionDispatch/btCollisionConfiguration.h>
#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>
#include <BulletCollision/BroadphaseCollision/btBroadphaseInterface.h>
#include <BulletDynamics/ConstraintSolver/btConstraintSolver.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>

#include "body.hpp"
#include "joint.hpp"
#include "actor.hpp"
#include "visworld.hpp"

namespace sim {


/**
 * Physical world.
 */
class World {
    btCollisionConfiguration *_coll_conf;
    btCollisionDispatcher *_dispatch;
    btBroadphaseInterface *_broadphase;
    btConstraintSolver *_solver;
    btDynamicsWorld *_world;
    VisWorld *_vis;

  public:
    World();
    virtual ~World();

    void addBody(Body *obj);
    void addJoint(Joint *j);
    void addActor(Actor *a);

    /**
     * Initializes world.
     */
    void init();

    /**
     * Destroys world.
     */
    void destroy();

    /**
     * Performes one step.
     */
    void step(bool phys = true, bool vis = true);

    bool done();
};

}

#endif /* _SIM_WORLD_HPP_ */

