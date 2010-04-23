#ifndef _SIM_PHYS_WORLD_HPP_
#define _SIM_PHYS_WORLD_HPP_

#include <BulletCollision/CollisionDispatch/btCollisionConfiguration.h>
#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>
#include <BulletCollision/BroadphaseCollision/btBroadphaseInterface.h>
#include <BulletDynamics/ConstraintSolver/btConstraintSolver.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>

#include "physobj.hpp"

namespace sim {


/**
 * Physical world.
 */
class PhysWorld {
    btCollisionConfiguration *_coll_conf;
    btCollisionDispatcher *_dispatch;
    btBroadphaseInterface *_broadphase;
    btConstraintSolver *_solver;
    btDynamicsWorld *_world;

  public:
    PhysWorld();
    virtual ~PhysWorld();
};

}

#endif /* _SIM_PHYS_WORLD_HPP_ */
