#ifndef _SIM_WORLD_HPP_
#define _SIM_WORLD_HPP_

#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>
#include <BulletCollision/CollisionDispatch/btCollisionConfiguration.h>
#include <btBroadphaseInterface>

namespace sim {

class World {
    btCollisionDispatcher *_dispatch;
    btCollisionConfiguration *_coll_conf;
    btBroadphaseInterface *_broadphase;
    btConstraitSolver *_solver;
    btDynamicsWorld *_world;

  public:
    World();
    virtual ~World();
};

}

#endif /* _SIM_WORLD_HPP_ */
