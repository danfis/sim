/***
 * sim
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
 *
 *  This file is part of sim.
 *
 *  sim is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation; either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  sim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
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
#include "sim/common.hpp"

namespace sim {

namespace bullet {

World::World()
    : sim::WorldBullet(),
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
    // delete all bodies
    for_each(_bodies_it_t, _bodies){
        delete *it;
    }
    _bodies.clear();

    // delete all joints
    for_each(_joints_it_t, _joints){
        delete *it;
    }
    _joints.clear();

    delete _world;
    delete _solver;
    delete _broadphase;
    delete _dispatch;
    delete _coll_conf;

    if (_vis)
        delete _vis;
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
    _setJointsForceToImpulse(time);

    btScalar fixed = time.inSF() / (double)substeps;
    _world->stepSimulation(time.inSF(), substeps, fixed);
}


bool World::done()
{
    return false;
}


sim::Body *World::createBodyCube(Scalar width, Scalar mass, VisBody *vis)
{
    return _createBody(new BodyCube(this, width, mass, vis));
}

sim::Body *World::createBodyBox(Vec3 dim, Scalar mass, VisBody *vis)
{
    return _createBody(new BodyBox(this, dim, mass, vis));
}

sim::Body *World::createBodySphere(Scalar radius, Scalar mass, VisBody *vis)
{
    return _createBody(new BodySphere(this, radius, mass, vis));
}

sim::Body *World::createBodyCylinderX(Scalar radius, Scalar height, Scalar mass,
                                      VisBody *vis)
{
    return _createBody(new BodyCylinderX(this, radius, height, mass, vis));
}

sim::Body *World::createBodyCylinderY(Scalar radius, Scalar height, Scalar mass,
                                      VisBody *vis)
{
    return _createBody(new BodyCylinderY(this, radius, height, mass, vis));
}

sim::Body *World::createBodyCylinderZ(Scalar radius, Scalar height, Scalar mass,
                                      VisBody *vis)
{
    return _createBody(new BodyCylinder(this, radius, height, mass, vis));
}

sim::Body *World::createBodyTriMesh(const Vec3 *coords, size_t coords_len,
                                    const unsigned int *indices, size_t indices_len,
                                    VisBody *vis)
{
    return _createBody(new BodyTriMesh(this, coords, coords_len, indices, indices_len, vis));
}

sim::Body *World::createBodyCompound()
{
    return _createBody(new Body(this));
}



sim::Joint *World::createJointFixed(sim::Body *oA, sim::Body *oB)
{
    return _createJoint(new JointFixed(this, (Body *)oA, (Body *)oB));
}

sim::Joint *World::createJointHinge(sim::Body *oA, sim::Body *oB,
                                    const Vec3 &anchor, const Vec3 &axis)
{
    return _createJoint(new JointHinge(this, (Body *)oA, (Body *)oB, anchor, axis));
}

sim::Joint *World::createJointHinge2(sim::Body *oA, sim::Body *oB, const Vec3 &anchor,
                                     const Vec3 &axis1, const Vec3 &axis2)
{
    return _createJoint(new JointHinge2(this, (Body *)oA, (Body *)oB, anchor, axis1, axis2));
}

sim::Body *World::_createBody(sim::Body *b)
{
    _bodies.push_back(b);
    return b;
}

sim::Joint *World::_createJoint(sim::Joint *j)
{
    _joints.push_back(j);
    return j;
}

void World::_setJointsForceToImpulse(const sim::Time &time)
{
    Joint *j;

    for_each(_joints_it_t, _joints){
        j = (Joint *)*it;

        if (j->isHinge()){
            ((JointHinge *)j)->_applyVelFMax(time);
        }
    }
}

} /* namespace bullet */


namespace WorldFactory {
WorldBullet *Bullet()
{
    return new sim::bullet::World();
}
} /* namespace WorldFactory */

} /* namespace sim */

