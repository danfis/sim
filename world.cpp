#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>

#include <algorithm>

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
    _dispatch = new CollisionDispatcher(_coll_conf);
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


void World::addBody(Body *obj)
{
    btRigidBody *body;
    VisBody *vobj;

    body = obj->body();
    if (body)
        _world->addRigidBody(body);

    vobj = obj->visBody();
    if (vobj)
        _vis->addBody(vobj);
}

void World::addJoint(Joint *j)
{
    btTypedConstraint *c = j->joint();

    DBG(c);
    if (c)
        _world->addConstraint(c);
}


void World::addActor(Actor *a)
{
    _actors.push_back(a);
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

void World::step(bool phys, bool vis)
{
    // perform simulation step
    // TODO: Parametrize this
    if (phys){
        _actorsRunPreStep();
        _world->stepSimulation(1./60., 20);
        _actorsRunPostStep();
    }

    if (vis && _vis)
        _vis->step();
}


bool World::done()
{
    if (_vis)
        return _vis->done();
    return false;
}

void World::regActorPreStep(Actor *a)
{
    if (std::find(_pre_step.begin(), _pre_step.end(), a) == _pre_step.end()){
        _pre_step.push_back(a);
    }
}

void World::regActorPostStep(Actor *a)
{
    if (std::find(_post_step.begin(), _post_step.end(), a) == _post_step.end()){
        _post_step.push_back(a);
    }
}

void World::_actorsRunPreStep()
{
    _act_it it, it_end;

    //DBG("Pre steps");

    it = _pre_step.begin();
    it_end = _pre_step.end();
    for (; it != it_end; ++it){
        //DBG("    " << *it);
        (*it)->preStep();
    }
}

void World::_actorsRunPostStep()
{
    _act_it it, it_end;

    //DBG("Pre steps");

    it = _post_step.begin();
    it_end = _post_step.end();
    for (; it != it_end; ++it){
        //DBG("    " << *it);
        (*it)->postStep();
    }
}


} /* namespace sim */

