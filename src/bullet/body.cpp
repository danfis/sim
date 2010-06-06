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

#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletCollision/CollisionShapes/btTriangleMesh.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <osg/ShapeDrawable>

#include "sim/bullet/body.hpp"
#include "sim/bullet/world.hpp"
#include "sim/bullet/math.hpp"
#include "sim/msg.hpp"

namespace sim {

namespace bullet {

void BodyMotionState::getWorldTransform(btTransform &world) const
{
    world = _offset.inverse() * _world;
}

void BodyMotionState::setWorldTransform(const btTransform &world)
{
    btVector3 pos;
    btQuaternion rot;

    _world = world * _offset;

    pos = _world.getOrigin();
    rot = _world.getRotation();
    DBG("pos: " << DBGV(vFromBt(pos)));
    _vis->setPosRot(vFromBt(pos), qFromBt(rot));
}

Body::Body(World *w)
    : _world(w), _body(0), _shape(0), _motion_state(0), _vis(0),
      _damping_lin(0.2), _damping_ang(0.2)
{
}

Body::~Body()
{
    if (_body)
        delete _body;
    if (_shape)
        delete _shape;
    if (_motion_state)
        delete _motion_state;
    if (_vis)
        delete _vis;
}

void Body::setPos(const Vec3 &v)
{
    btTransform trans;
    Vec3 pos = v;

    _motion_state->getWorldTransform(trans);

    trans.setOrigin(vToBt(pos));
    _motion_state->setWorldTransform(trans);
    _body->setWorldTransform(trans);
}

void Body::setRot(const Quat &q)
{
    btTransform trans;
    _motion_state->getWorldTransform(trans);

    trans.setRotation(qToBt(q));
    _motion_state->setWorldTransform(trans);
    _body->setWorldTransform(trans);
}

void Body::pos(Scalar *x, Scalar *y, Scalar *z) const
{
    Vec3 v = pos();
    *x = v.x();
    *y = v.y();
    *z = v.z();
}
Vec3 Body::pos() const
{
    btTransform trans;
    _motion_state->getWorldTransform(trans);
    return vFromBt(trans.getOrigin());
}

void Body::rot(Scalar *x, Scalar *y, Scalar *z, Scalar *w) const
{
    Quat q = rot();
    *x = q.x();
    *y = q.y();
    *z = q.z();
    *w = q.w();
}

Quat Body::rot() const
{
    btTransform trans;
    _motion_state->getWorldTransform(trans);
    return qFromBt(trans.getRotation());
}

void Body::setPosRot(const Vec3 &v, const Quat &q)
{
    btTransform trans;
    _motion_state->getWorldTransform(trans);
    trans.setOrigin(vToBt(v));
    trans.setRotation(qToBt(q));
    _motion_state->setWorldTransform(trans);
    _body->setWorldTransform(trans);
}

void Body::activate()
{
    _body->setDamping(_damping_lin, _damping_ang);
    _world->addBody(this);
}

void Body::deactivate()
{
    /*TODO: _world->rmBody(this);*/
}


void Body::_set(VisBody *o, btCollisionShape *shape, Scalar mass)
{
    btVector3 local_inertia(0,0,0);

    if (o)
        setVisBody(o);

    _shape = shape;
    _motion_state = new BodyMotionState(o);

    if (!btFuzzyZero(mass))
        _shape->calculateLocalInertia(mass, local_inertia);

    _body = new btRigidBody(mass, _motion_state, _shape, local_inertia);

    _body->setUserPointer(this);
}



BodyCube::BodyCube(World *world, Scalar w, Scalar mass, VisBody *vis)
    : Body(world)
{
    btCollisionShape *shape = new btBoxShape(btVector3(w / 2., w / 2., w / 2.));

    if (vis == SIM_BODY_DEFAULT_VIS)
        vis = new VisBodyCube(w);

    _set(vis, shape, mass);
}

BodyBox::BodyBox(World *w, Vec3 dim, Scalar mass, VisBody *vis)
    : Body(w)
{
    btCollisionShape *shape = new btBoxShape(vToBt(dim / 2.));

    if (vis == SIM_BODY_DEFAULT_VIS)
        vis = new VisBodyBox(dim);

    _set(vis, shape, mass);
}

BodySphere::BodySphere(World *w, Scalar radius, Scalar mass, VisBody *vis)
    : Body(w)
{
    btCollisionShape *shape = new btSphereShape(radius);

    if (vis == SIM_BODY_DEFAULT_VIS)
        vis = new VisBodySphere(radius);

    _set(vis, shape, mass);
}

BodyCylinder::BodyCylinder(World *w, Scalar radius, Scalar height, Scalar mass,
                           VisBody *vis)
    : Body(w)
{
    Vec3 v(radius, radius, height / 2.);
    btCollisionShape *shape = new btCylinderShape(vToBt(v));

    if (vis == SIM_BODY_DEFAULT_VIS)
        vis = new VisBodyCylinder(radius, height);

    _set(vis, shape, mass);
}

BodyCylinderX::BodyCylinderX(World *w, Scalar radius, Scalar height, Scalar mass,
                             VisBody *vis)
    : BodyCylinder(w, radius, height, mass, vis)
{
    // parent constructor already created cylinder
    // now it must be correctly rotated
    Quat q(Vec3(0., 1., 0.), M_PI * .5);
    setRot(q);
}

BodyCylinderY::BodyCylinderY(World *w, Scalar radius, Scalar height, Scalar mass,
                             VisBody *vis)
    : BodyCylinder(w, radius, height, mass, vis)
{
    // parent constructor already created cylinder
    // now it must be correctly rotated
    Quat q(Vec3(1., 0., 0.), M_PI * .5);
    setRot(q);
}

BodyTriMesh::BodyTriMesh(World *w, const sim::Vec3 *coords, size_t coords_len,
                         const unsigned int *indices, size_t indices_len,
                         VisBody *vis)
    : Body(w)
{
    btTriangleMesh *tris = new btTriangleMesh();

    for (size_t i = 0; i < indices_len; i += 3){
        tris->addTriangle(vToBt(coords[indices[i]]),
                          vToBt(coords[indices[i + 1]]),
                          vToBt(coords[indices[i + 2]]));
    }

    if (vis == SIM_BODY_DEFAULT_VIS)
        vis = new sim::VisBodyTriMesh(coords, coords_len, indices, indices_len);

    btBvhTriangleMeshShape *shape = new btBvhTriangleMeshShape(tris, true);
    _set(vis, shape, 0.);
}

BodyConvexHull::BodyConvexHull(World *w, const sim::Vec3 *points, size_t points_len,
                               Scalar mass, VisBody *vis)
    : Body(w)
{
    btConvexHullShape *shape;

    shape = new btConvexHullShape();
    for (size_t i = 0; i < points_len; i++){
        shape->addPoint(vToBt(points[i]));
    }

    _set(vis, shape, mass);
}

} /* namespace bullet */

} /* namespace sim */
