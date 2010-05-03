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
    std::list<Body *>::iterator bit, bit_end;
    std::list<Body *> &bodies = a->bodies();
    std::list<Joint *>::iterator jit, jit_end;
    std::list<Joint *> &joints = a->joints();

    bit = bodies.begin();
    bit_end = bodies.end();
    for (; bit != bit_end; ++bit){
        addBody(*bit);
    }

    jit = joints.begin();
    jit_end = joints.end();
    for (; jit != jit_end; ++jit){
        addJoint(*jit);
    }
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
    if (phys)
        _world->stepSimulation(1./60., 20);

    if (vis && _vis)
        _vis->step();
}

bool World::done()
{
    if (_vis)
        return _vis->done();
    return false;
}

} /* namespace sim */

