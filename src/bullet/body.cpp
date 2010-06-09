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
    world = _world;
}

void BodyMotionState::setWorldTransform(const btTransform &world)
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
    : sim::Body(),
      _world(w), _body(0),
      _next_id(1), _mass(0), _local_inertia(0., 0., 0.),
      _damping_lin(0.2), _damping_ang(0.2)
{
    _shape = new btCompoundShape;
    _motion_state = new BodyMotionState(this);
}

Body::~Body()
{
    for_each(_shapes_it_t, _shapes){
        if (it->second->vis)
            delete it->second->vis;

        _shape->removeChildShape(it->second->shape);
        delete it->second->shape;

        delete it->second;
    }
    _shapes.clear();

    if (_body)
        delete _body;
    if (_motion_state)
        delete _motion_state;
}

int Body::addCube(Scalar w, VisBody *vis,
                          const Vec3 &pos, const Quat &rot)
{
    return addBox(Vec3(w, w, w), vis, pos, rot);
}

int Body::addBox(const Vec3 &dim, VisBody *vis,
                         const Vec3 &pos, const Quat &rot)
{
    btCollisionShape *shape = new btBoxShape(vToBt(dim / 2));

    if (vis == SIM_BODY_DEFAULT_VIS)
        vis = new VisBodyBox(dim);

    return _addShape(shape, vis, pos, rot);
}

int Body::addSphere(Scalar radius, VisBody *vis,
                            const Vec3 &pos, const Quat &rot)
{
    btCollisionShape *shape = new btSphereShape(radius);

    if (vis == SIM_BODY_DEFAULT_VIS)
        vis = new VisBodySphere(radius);

    return _addShape(shape, vis, pos, rot);
}

int Body::addCylinderZ(Scalar radius, Scalar height, VisBody *vis,
                            const Vec3 &pos, const Quat &rot)
{
    Vec3 v(radius, radius, height / 2.);
    btCollisionShape *shape = new btCylinderShape(vToBt(v));

    if (vis == SIM_BODY_DEFAULT_VIS)
        vis = new VisBodyCylinder(radius, height);

    return _addShape(shape, vis, pos, rot);
}

int Body::addCylinderY(Scalar radius, Scalar height, VisBody *vis,
                               const Vec3 &pos, const Quat &rot)
{
    Quat r = rot * Quat(Vec3(1., 0., 0.), M_PI * .5);
    return addCylinderZ(radius, height, vis, pos, r);
}

int Body::addCylinderX(Scalar radius, Scalar height, VisBody *vis,
                               const Vec3 &pos, const Quat &rot)
{
    Quat r = rot * Quat(Vec3(0., 1., 0.), M_PI * .5);
    return addCylinderZ(radius, height, vis, pos, r);
}

int Body::addTriMesh(const sim::Vec3 *coords, size_t coords_len,
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


void Body::setMassCube(Scalar w, Scalar mass)
{
    setMassBox(Vec3(w, w, w), mass);
}

void Body::setMassBox(const Vec3 &dim, Scalar mass)
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

void Body::setMassSphere(Scalar radius, Scalar mass)
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

void Body::setMassCylinder(Scalar radius, Scalar height, Scalar mass)
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

void Body::rmShape(int id)
{
    shape_t *s;

    s = shape(id);
    if (!s)
        return;

    if (s->vis)
        delete s->vis;

    _shape->removeChildShape(s->shape);
    delete s->shape;

    delete s;

    _shapes.erase(id);
}

void Body::activate()
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

void Body::deactivate()
{
    /*TODO: _world->rmBody(this);*/
}



VisBody *Body::visBody(int id)
{
    shape_t *s = shape(id);
    if (!s)
        return 0;
    return s->vis;
}

const VisBody *Body::visBody(int id) const
{
    const shape_t *s = shape(id);
    if (!s)
        return 0;
    return s->vis;
}

void Body::visBodyAll(std::list<const VisBody *> *list) const
{
    for_each(_shapes_cit_t, _shapes){
        list->push_back(it->second->vis);
    }
}

void Body::visBodyAll(std::list<VisBody *> *list)
{
    for_each(_shapes_cit_t, _shapes){
        list->push_back(it->second->vis);
    }
}



int Body::_addShape(btCollisionShape *shape, VisBody *vis,
                            const Vec3 &pos, const Quat &rot)
{
    shape_t *s;
   
    s = new shape_t(shape, vis);

    _shapes.insert(_shapes_t::value_type(_next_id, s));

    s->tr.setIdentity();
    s->tr.setOrigin(vToBt(pos));
    s->tr.setRotation(qToBt(rot));

    _shape->addChildShape(s->tr, shape);

    return _next_id++;
}

Body::shape_t *Body::shape(int ID)
{
    _shapes_it_t it = _shapes.find(ID);

    if (it != _shapes.end())
        return it->second;
    return 0;
}

const Body::shape_t *Body::shape(int ID) const
{
    _shapes_cit_t it = _shapes.find(ID);

    if (it != _shapes.end())
        return it->second;
    return 0;
}

void Body::_applyShapesToVis()
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

void Body::_applyPosRotToBt()
{
    btTransform world;
    world.setIdentity();
    world.setOrigin(vToBt(pos()));
    world.setRotation(qToBt(rot()));
    _motion_state->setWorldTransform(world);
    _body->setWorldTransform(world);
}





BodyBox::BodyBox(World *w, Vec3 dim, Scalar mass, VisBody *vis)
    : BodySimple(w)
{
    addBox(dim, vis);
    setMassBox(dim, mass);
}

BodyCube::BodyCube(World *world, Scalar w, Scalar mass, VisBody *vis)
    : BodyBox(world, Vec3(w, w, w), mass, vis)
{
}

BodySphere::BodySphere(World *w, Scalar radius, Scalar mass, VisBody *vis)
    : BodySimple(w)
{
    addSphere(radius, vis);
    setMassSphere(radius, mass);
}

BodyCylinder::BodyCylinder(World *w, Scalar radius, Scalar height, Scalar mass,
                           VisBody *vis)
    : BodySimple(w)
{
    addCylinderZ(radius, height, vis);
    setMassCylinder(radius, height, mass);
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
    : BodySimple(w)
{
    addTriMesh(coords, coords_len, indices, indices_len, vis);
}


} /* namespace bullet */

} /* namespace sim */
