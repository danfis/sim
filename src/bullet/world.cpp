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

#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>

#include <algorithm>

#include "sim/bullet/world.hpp"
#include "sim/bullet/math.hpp"
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
    _world->setGravity(vToBt(_gravity));
}

void World::finish()
{
}

void World::step(const sim::Time &time, unsigned int substeps)
{
    btScalar fixed = time.inSF() / (double)substeps;
    _world->stepSimulation(time.inSF(), substeps, fixed);
}


bool World::done()
{
    return false;
}


sim::Body *World::createBodyCube(Scalar width, Scalar mass, VisBody *vis)
{
    return new BodyCube(this, width, mass, vis);
}

sim::Body *World::createBodyBox(Vec3 dim, Scalar mass, VisBody *vis)
{
    return new BodyBox(this, dim, mass, vis);
}

sim::Body *World::createBodySphere(Scalar radius, Scalar mass, VisBody *vis)
{
    return new BodySphere(this, radius, mass, vis);
}

sim::Body *World::createBodyCylinderX(Scalar radius, Scalar height, Scalar mass,
                                      VisBody *vis)
{
    return new BodyCylinderX(this, radius, height, mass, vis);
}

sim::Body *World::createBodyCylinderY(Scalar radius, Scalar height, Scalar mass,
                                      VisBody *vis)
{
    return new BodyCylinderY(this, radius, height, mass, vis);
}

sim::Body *World::createBodyCylinderZ(Scalar radius, Scalar height, Scalar mass,
                                      VisBody *vis)
{
    return new BodyCylinder(this, radius, height, mass, vis);
}

sim::Body *World::createBodyTriMesh(const Vec3 *coords, size_t coords_len,
                                    const unsigned int *indices, size_t indices_len,
                                    Scalar mass, VisBody *vis)
{
    return new BodyTriMesh(this, coords, coords_len, indices, indices_len, vis);
}

sim::Body *World::createBodyCompound()
{
    return 0;
}



sim::Joint *World::createJointFixed(sim::Body *oA, sim::Body *oB)
{
    return new JointFixed(this, (Body *)oA, (Body *)oB);
}

sim::Joint *World::createJointHinge(sim::Body *oA, sim::Body *oB,
                                    const Vec3 &anchor, const Vec3 &axis)
{
    DBG("");
    return new JointHinge(this, (Body *)oA, (Body *)oB, anchor, axis);
}

sim::Joint *World::createJointHinge2(sim::Body *oA, sim::Body *oB, const Vec3 &anchor,
                                     const Vec3 &axis1, const Vec3 &axis2)
{
    return new JointHinge2(this, (Body *)oA, (Body *)oB, anchor, axis1, axis2);
}



} /* namespace bullet */

} /* namespace sim */

