#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>

#include "physworld.hpp"

namespace sim {

PhysWorld::PhysWorld()
    : _coll_conf(0),
      _dispatch(0),
      _broadphase(0),
      _solver(0),
      _world(0)
{
    _coll_conf = dynamic_cast<btCollisionConfiguration *>(new btDefaultCollisionConfiguration());
    _dispatch = new btCollisionDispatcher(_coll_conf);
    _broadphase = dynamic_cast<btBroadphaseInterface *>(new btDbvtBroadphase());
    _solver = dynamic_cast<btConstraintSolver *>(new btSequentialImpulseConstraintSolver());
    _world = dynamic_cast<btDynamicsWorld *>(new btDiscreteDynamicsWorld(_dispatch, _broadphase, _solver, _coll_conf));

    _world->setGravity(btVector3(0,-10,0));
}

PhysWorld::~PhysWorld()
{
    delete _world;
    delete _solver;
    delete _broadphase;
    delete _dispatch;
    delete _coll_conf;
}

} /* namespace sim */
