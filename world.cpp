#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>

#include "world.hpp"
#include "msg.hpp"

namespace sim {

World::World()
    : _coll_conf(0),
      _dispatch(0),
      _broadphase(0),
      _solver(0),
      _world(0),
      _vis(0)
{
    _coll_conf = new btDefaultCollisionConfiguration();
    _dispatch = new btCollisionDispatcher(_coll_conf);
    _broadphase = new btDbvtBroadphase();
    _solver = new btSequentialImpulseConstraintSolver();
    _world = new btDiscreteDynamicsWorld(_dispatch, _broadphase, _solver, _coll_conf);

    _vis = new VisWorld();

    _world->setGravity(btVector3(0, -9.81, 0));
}

World::~World()
{
    delete _world;
    delete _solver;
    delete _broadphase;
    delete _dispatch;
    delete _coll_conf;

    if (_vis)
        delete _vis;
}


void World::addObj(Obj *obj)
{
    btRigidBody *body;
    VisObj *vobj;

    body = obj->body();
    if (body)
        _world->addRigidBody(body);

    vobj = obj->visObj();
    if (vobj)
        _vis->addObj(vobj);
}

void World::addJoint(Joint *j)
{
    btTypedConstraint *c = j->joint();

    DBG(c);
    if (c)
        _world->addConstraint(c);
}


void World::init()
{
    if (_vis)
        _vis->init();
}

void World::destroy()
{
    if (_vis)
        _vis->destroy();
}

void World::step()
{
    // perform simulation step
    // TODO: Parametrize this
    _world->stepSimulation(1./60., 20);

    if (_vis)
        _vis->step();
}

bool World::done()
{
    if (_vis)
        return _vis->done();
    return false;
}

} /* namespace sim */

