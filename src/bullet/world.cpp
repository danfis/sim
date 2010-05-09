#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>

#include <algorithm>

#include "sim/bullet/world.hpp"
#include "sim/msg.hpp"

namespace sim {

namespace bullet {

World::World()
    : sim::World(),
      _coll_conf(0),
      _dispatch(0),
      _broadphase(0),
      _solver(0),
      _world(0)
{
    _coll_conf = new btDefaultCollisionConfiguration();
    _dispatch = new CollisionDispatcher(_coll_conf);
    _broadphase = new btDbvtBroadphase();
    _solver = new btSequentialImpulseConstraintSolver();
    _world = new btDiscreteDynamicsWorld(_dispatch, _broadphase, _solver, _coll_conf);

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
    if (vobj && visWorld())
        visWorld()->addBody(vobj);
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
}

void World::finish()
{
}

void World::step()
{
    // perform simulation step
    // TODO: Parametrize this
    _world->stepSimulation(1./60., 20);
}


bool World::done()
{
    return false;
}


sim::Body *World::createBodyCube(Scalar width, Scalar mass)
{
    return new BodyCube(this, width, mass);
}

sim::Body *World::createBodyBox(Vec3 dim, Scalar mass)
{
    return new BodyBox(this, dim, mass);
}

sim::Body *World::createBodySphere(Scalar radius, Scalar mass)
{
    return new BodySphere(this, radius, mass);
}

sim::Body *World::createBodyCylinderX(Scalar radius, Scalar height, Scalar mass)
{
    return new BodyCylinderX(this, radius, height, mass);
}

sim::Body *World::createBodyCylinderY(Scalar radius, Scalar height, Scalar mass)
{
    return new BodyCylinderY(this, radius, height, mass);
}

sim::Body *World::createBodyCylinderZ(Scalar radius, Scalar height, Scalar mass)
{
    return new BodyCylinder(this, radius, height, mass);
}

sim::Joint *World::createJointFixed(sim::Body *oA, sim::Body *oB)
{
    Body *A = dynamic_cast<Body *>(oA);
    Body *B = dynamic_cast<Body *>(oB);

    if (!A || !B)
        return 0;

    return new JointFixed(this, A, B);
}

sim::Joint *World::createJointHinge(sim::Body *oA, sim::Body *oB,
                                    const Vec3 &anchor, const Vec3 &axis)
{
    Body *A = dynamic_cast<Body *>(oA);
    Body *B = dynamic_cast<Body *>(oB);

    if (!A || !B)
        return 0;

    return new JointHinge(this, A, B, anchor, axis);
}

sim::Joint *World::createJointHinge2(sim::Body *oA, sim::Body *oB, const Vec3 &anchor,
                                     const Vec3 &axis1, const Vec3 &axis2)
{
    Body *A = dynamic_cast<Body *>(oA);
    Body *B = dynamic_cast<Body *>(oB);

    if (!A || !B)
        return 0;

    return new JointHinge2(this, A, B, anchor, axis1, axis2);
}


sim::ActuatorWheelCylinderX *World::createActuatorWheelCylinderX
                                (Scalar radius, Scalar height, Scalar mass)
{
    return new ActuatorWheelCylinderX(this, radius, height, mass);
}


} /* namespace bullet */

} /* namespace sim */

