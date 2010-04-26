#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>

#include "physworld.hpp"
#include "msg.hpp"

namespace sim {

PhysWorld::PhysWorld()
    : _coll_conf(0),
      _dispatch(0),
      _broadphase(0),
      _solver(0),
      _world(0)
{
    _coll_conf = new btDefaultCollisionConfiguration();
    _dispatch = new btCollisionDispatcher(_coll_conf);
    _broadphase = new btDbvtBroadphase();
    _solver = new btSequentialImpulseConstraintSolver();
    _world = new btDiscreteDynamicsWorld(_dispatch, _broadphase, _solver, _coll_conf);

    _world->setGravity(btVector3(0, -9.81, 0));
}

PhysWorld::~PhysWorld()
{
    delete _world;
    delete _solver;
    delete _broadphase;
    delete _dispatch;
    delete _coll_conf;
}


void PhysWorld::addObj(PhysObj *obj)
{
    btRigidBody *body;

    body = obj->body();
    DBG(body);
    DBG(body->isStaticObject());
    DBG(body->getCollisionShape());
    if (body)
        _world->addRigidBody(body);
}


void PhysWorld::init()
{
}

void PhysWorld::destroy()
{
}

void PhysWorld::step()
{
    // perform simulation step
    // TODO: Parametrize this
    _world->stepSimulation(1./60., 20);
}

} /* namespace sim */
