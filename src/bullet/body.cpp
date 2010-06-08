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
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <osg/ShapeDrawable>

#include "sim/bullet/body.hpp"
#include "sim/bullet/world.hpp"
#include "sim/bullet/math.hpp"
#include "sim/msg.hpp"
#include "sim/common.hpp"

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
    //DBG("pos: " << DBGV(vFromBt(pos)));
    _body->setPosRot(vFromBt(pos), qFromBt(rot));
    if (_body->visBody())
        _body->visBody()->setPosRot(vFromBt(pos), qFromBt(rot));
}

void BodyMotionStateCompound::getWorldTransform(btTransform &world) const
{
    world = _world;
}

void BodyMotionStateCompound::setWorldTransform(const btTransform &world)
{
    btVector3 pos;
    btQuaternion rot;

    _world = world;

    pos = _world.getOrigin();
    rot = _world.getRotation();
    //DBG("pos: " << DBGV(vFromBt(pos)));
    _body->setPosRot(vFromBt(pos), qFromBt(rot));

    _body->_applyShapesToVis();
}

Body::Body(World *w)
    : _world(w), _body(0), _shape(0), _motion_state(0),
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


void Body::activate()
{
    _body->setDamping(_damping_lin, _damping_ang);

    _applyPosRotToBt();

    _world->world()->addRigidBody(_body);

    if (_world->visWorld() && _vis){
        _world->visWorld()->addBody(_vis);
    }
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
    _motion_state = new BodyMotionState(this);

    if (!btFuzzyZero(mass))
        _shape->calculateLocalInertia(mass, local_inertia);

    _body = new btRigidBody(mass, _motion_state, _shape, local_inertia);

    _body->setUserPointer(this);
}

void Body::_applyPosRotToBt()
{
    btTransform world;
    world.setIdentity();
    world.setOrigin(vToBt(pos()));
    world.setRotation(qToBt(rot()));
    _motion_state->setWorldTransform(world);
    _body->setWorldTransform(world);
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

BodyCompound::BodyCompound(World *w)
    : Body(w),
      _next_id(1), _mass(0), _local_inertia(0., 0., 0.)
{
    _shape = new btCompoundShape;
    _motion_state = new BodyMotionStateCompound(this);
}

BodyCompound::~BodyCompound()
{
    for_each(_shapes_it_t, _shapes){
        if (it->second->vis)
            delete it->second->vis;

        ((btCompoundShape *)_shape)->removeChildShape(it->second->shape);
        delete it->second->shape;

        delete it->second;
    }
    _shapes.clear();
}

int BodyCompound::addCube(Scalar w, VisBody *vis,
                          const Vec3 &pos, const Quat &rot)
{
    return addBox(Vec3(w, w, w), vis, pos, rot);
}

int BodyCompound::addBox(const Vec3 &dim, VisBody *vis,
                         const Vec3 &pos, const Quat &rot)
{
    btCollisionShape *shape = new btBoxShape(vToBt(dim / 2));

    if (vis == SIM_BODY_DEFAULT_VIS)
        vis = new VisBodyBox(dim);

    return _addShape(shape, vis, pos, rot);
}

int BodyCompound::addSphere(Scalar radius, VisBody *vis,
                            const Vec3 &pos, const Quat &rot)
{
    btCollisionShape *shape = new btSphereShape(radius);

    if (vis == SIM_BODY_DEFAULT_VIS)
        vis = new VisBodySphere(radius);

    return _addShape(shape, vis, pos, rot);
}

int BodyCompound::addCylinderZ(Scalar radius, Scalar height, VisBody *vis,
                            const Vec3 &pos, const Quat &rot)
{
    Vec3 v(radius, radius, height / 2.);
    btCollisionShape *shape = new btCylinderShape(vToBt(v));

    if (vis == SIM_BODY_DEFAULT_VIS)
        vis = new VisBodyCylinder(radius, height);

    return _addShape(shape, vis, pos, rot);
}

int BodyCompound::addCylinderY(Scalar radius, Scalar height, VisBody *vis,
                               const Vec3 &pos, const Quat &rot)
{
    Quat r = rot * Quat(Vec3(1., 0., 0.), M_PI * .5);
    return addCylinderZ(radius, height, vis, pos, r);
}

int BodyCompound::addCylinderX(Scalar radius, Scalar height, VisBody *vis,
                               const Vec3 &pos, const Quat &rot)
{
    Quat r = rot * Quat(Vec3(0., 1., 0.), M_PI * .5);
    return addCylinderZ(radius, height, vis, pos, r);
}

int BodyCompound::addTriMesh(const sim::Vec3 *coords, size_t coords_len,
                             const unsigned int *ids, size_t ids_len,
                             VisBody *vis, const Vec3 &pos, const Quat &rot)
{
    btTriangleMesh *tris = new btTriangleMesh();

    for (size_t i = 0; i < ids_len; i += 3){
        tris->addTriangle(vToBt(coords[ids[i]]),
                          vToBt(coords[ids[i + 1]]),
                          vToBt(coords[ids[i + 2]]));
    }

    if (vis == SIM_BODY_DEFAULT_VIS)
        vis = new sim::VisBodyTriMesh(coords, coords_len, ids, ids_len);

    btBvhTriangleMeshShape *shape = new btBvhTriangleMeshShape(tris, true);

    return _addShape(shape, vis, pos, rot);
}


void BodyCompound::setMassCube(Scalar w, Scalar mass)
{
    setMassBox(Vec3(w, w, w), mass);
}

void BodyCompound::setMassBox(const Vec3 &dim, Scalar mass)
{
    _mass = mass;

    if (!isZero(_mass)){
        btVector3 local_inertia(0,0,0);

        btCollisionShape *shape = new btBoxShape(vToBt(dim / 2.));
        shape->calculateLocalInertia(_mass, local_inertia);
        delete shape;

        _local_inertia = vFromBt(local_inertia);
    }else{
        _local_inertia.set(0., 0., 0.);
    }
}

void BodyCompound::setMassSphere(Scalar radius, Scalar mass)
{
    _mass = mass;

    if (!isZero(_mass)){
        btVector3 local_inertia(0,0,0);

        btCollisionShape *shape = new btSphereShape(radius);
        shape->calculateLocalInertia(_mass, local_inertia);
        delete shape;

        _local_inertia = vFromBt(local_inertia);
    }else{
        _local_inertia.set(0., 0., 0.);
    }
}

void BodyCompound::setMassCylinder(Scalar radius, Scalar height, Scalar mass)
{
    _mass = mass;

    if (!isZero(_mass)){
        btVector3 local_inertia(0,0,0);

        Vec3 v(radius, radius, height / 2.);
        btCollisionShape *shape = new btCylinderShape(vToBt(v));
        shape->calculateLocalInertia(_mass, local_inertia);
        delete shape;

        _local_inertia = vFromBt(local_inertia);
    }else{
        _local_inertia.set(0., 0., 0.);
    }
}

void BodyCompound::rmShape(int id)
{
    shape_t *s;

    s = shape(id);
    if (!s)
        return;

    if (s->vis)
        delete s->vis;

    ((btCompoundShape *)_shape)->removeChildShape(s->shape);
    delete s->shape;

    delete s;

    _shapes.erase(id);
}

void BodyCompound::activate()
{
    _body = new btRigidBody(_mass, _motion_state, _shape, vToBt(_local_inertia));
    _body->setUserPointer(this);

    _body->setDamping(_damping_lin, _damping_ang);

    _applyPosRotToBt();
    _applyShapesToVis();

    _world->world()->addRigidBody(_body);

    if (_world->visWorld()){
        for_each(_shapes_it_t, _shapes){
            _world->visWorld()->addBody(it->second->vis);
        }
    }
}



VisBody *BodyCompound::visBody(int id)
{
    shape_t *s = shape(id);
    if (!s)
        return 0;
    return s->vis;
}

const VisBody *BodyCompound::visBody(int id) const
{
    const shape_t *s = shape(id);
    if (!s)
        return 0;
    return s->vis;
}

void BodyCompound::visBodyAll(std::list<const VisBody *> *list) const
{
    for_each(_shapes_cit_t, _shapes){
        list->push_back(it->second->vis);
    }
}

void BodyCompound::visBodyAll(std::list<VisBody *> *list)
{
    for_each(_shapes_cit_t, _shapes){
        list->push_back(it->second->vis);
    }
}



int BodyCompound::_addShape(btCollisionShape *shape, VisBody *vis,
                            const Vec3 &pos, const Quat &rot)
{
    shape_t *s;
   
    s = new shape_t(shape, vis);

    _shapes.insert(_shapes_t::value_type(_next_id, s));

    s->tr.setIdentity();
    s->tr.setOrigin(vToBt(pos));
    s->tr.setRotation(qToBt(rot));

    ((btCompoundShape *)_shape)->addChildShape(s->tr, shape);

    return _next_id++;
}

BodyCompound::shape_t *BodyCompound::shape(int ID)
{
    _shapes_it_t it = _shapes.find(ID);

    if (it != _shapes.end())
        return it->second;
    return 0;
}

const BodyCompound::shape_t *BodyCompound::shape(int ID) const
{
    _shapes_cit_t it = _shapes.find(ID);

    if (it != _shapes.end())
        return it->second;
    return 0;
}

void BodyCompound::_applyShapesToVis()
{
    VisBody *vis;
    btTransform world(qToBt(rot()), vToBt(pos()));
    btTransform tr;

    for_each(_shapes_it_t, _shapes){
        vis = it->second->vis;

        tr = world * it->second->tr;
        vis->setPos(vFromBt(tr.getOrigin()));
        vis->setRot(qFromBt(tr.getRotation()));
    }
}

} /* namespace bullet */

} /* namespace sim */
