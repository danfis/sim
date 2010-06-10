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

#include "sim/ode/body.hpp"
#include "sim/ode/world.hpp"
#include "sim/ode/math.hpp"
#include "sim/msg.hpp"
#include "sim/common.hpp"

namespace sim {

namespace ode {

static dGeomID geomBuildTriMesh(const Vec3 *coords, size_t coords_len,
                                const unsigned int *ids, size_t ids_len,
                                dSpaceID space)
{
    float *vertices;
    dTriIndex *indices;
    size_t i;
    dTriMeshDataID data;
    dGeomID shape;
   
    data = dGeomTriMeshDataCreate();

    // data must be transformed to ODE understand that
    // also indices should be copied because of type of dTriIndex
    vertices = new float[3 * coords_len];
    indices = new dTriIndex[ids_len];
    for (i = 0; i < coords_len; i++){
        vertices[3 * i] = coords[i].x();
        vertices[3 * i + 1] = coords[i].y();
        vertices[3 * i + 2] = coords[i].z();
    }
    for (i = 0; i < ids_len; i++){
        indices[i] = ids[i];
    }

    dGeomTriMeshDataBuildSingle(data, vertices, 3 * sizeof(float), coords_len,
                                      indices, ids_len, 3 * sizeof(dTriIndex));
    //dGeomTriMeshDataPreprocess(data);

    shape = dCreateTriMesh(space, data, 0, 0, 0);

    // TODO: How to deal with allocated arrays?

    return shape;
}

void bodyMovedCB(dBodyID body)
{
    const dReal *pos;
    const dReal *rot;
    Body *b = (Body *)dBodyGetData(body);

    pos = dBodyGetPosition(body);
    rot = dBodyGetQuaternion(body);
    b->setPos(pos[0], pos[1], pos[2]);
    b->setRot(rot[1], rot[2], rot[3], rot[0]);

    //DBG(body << " " << DBGV(b->pos()));
    b->_applyGeomsToVis();
}




Body::Body(World *w)
    : _world(w), _body(0), _next_id(1)
{
    setVisBody(0);
    dMassSetZero(&_mass);
    _move_cb = bodyMovedCB;
}

Body::~Body()
{
    for_each(_shapes_it_t, _shapes){
        dGeomDestroy(it->second->shape);

        if (it->second->vis)
            delete it->second->vis;

        delete it->second;
    }
    _shapes.clear();

    if (_body)
        dBodyDestroy(_body);
    if (_vis)
        delete _vis;
}

Body::shape_t *Body::_aShape()
{
    if (_shapes.size() == 0)
        return 0;
    return _shapes.begin()->second;
}

const Body::shape_t *Body::_aShape() const
{
    if (_shapes.size() == 0)
        return 0;
    return _shapes.begin()->second;
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

int Body::_addShape(dGeomID shape, VisBody *vis,
                             const Vec3 &pos, const Quat &rot)
{
    shape_t *s = new shape_t(shape, vis, pos, rot);

    _shapes.insert(_shapes_t::value_type(_next_id, s));

    return _next_id++;
}

int Body::addCube(Scalar w, VisBody *vis,
                          const Vec3 &pos, const Quat &rot)
{
    dGeomID shape;
    shape = dCreateBox(world()->space(), w, w, w);

    if (vis == SIM_BODY_DEFAULT_VIS)
        vis = new VisBodyCube(w);

    return _addShape(shape, vis, pos, rot);
}

int Body::addBox(const Vec3 &dim, VisBody *vis,
                         const Vec3 &pos, const Quat &rot)
{
    dGeomID shape;
    shape = dCreateBox(world()->space(), dim.x(), dim.y(), dim.z());

    if (vis == SIM_BODY_DEFAULT_VIS)
        vis = new VisBodyBox(dim);

    return _addShape(shape, vis, pos, rot);
}

int Body::addSphere(Scalar radius, VisBody *vis,
                            const Vec3 &pos, const Quat &rot)
{
    dGeomID shape;
    shape = dCreateSphere(world()->space(), radius);

    if (vis == SIM_BODY_DEFAULT_VIS)
        vis = new VisBodySphere(radius);

    return _addShape(shape, vis, pos, rot);
}

int Body::addCylinderZ(Scalar radius, Scalar height, VisBody *vis,
                               const Vec3 &pos, const Quat &rot)
{
    dGeomID shape;
    shape = dCreateCylinder(world()->space(), radius, height);

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
    dGeomID shape;

    shape = geomBuildTriMesh(coords, coords_len, ids, ids_len, _world->space());

    if (vis == SIM_BODY_DEFAULT_VIS)
        vis = new VisBodyTriMesh(coords, coords_len, ids, ids_len);

    return _addShape(shape, vis, pos, rot);
}

void Body::setMassCube(Scalar w, Scalar mass)
{
    dMassSetBoxTotal(&_mass, mass, w, w, w);
}

void Body::setMassBox(const Vec3 &dim, Scalar mass)
{
    dMassSetBoxTotal(&_mass, mass, dim.x(), dim.y(), dim.z());
}

void Body::setMassSphere(Scalar radius, Scalar mass)
{
    dMassSetSphereTotal(&_mass, mass, radius);
}

void Body::setMassCylinder(Scalar radius, Scalar height, Scalar mass)
{
    dMassSetCylinderTotal(&_mass, mass, 3, radius, height);
}

void Body::rmShape(int id)
{
    shape_t *s;

    s = shape(id);
    if (!s)
        return;

    dGeomDestroy(s->shape);
    if (s->vis)
        delete s->vis;
    delete s;

    _shapes.erase(id);
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


void Body::_applyGeomsToVis()
{
    const dReal *p;
    dQuaternion q;

    for_each(_shapes_it_t, _shapes){
        if (it->second->vis){
            p = dGeomGetPosition(it->second->shape);
            dGeomGetQuaternion(it->second->shape, q);
            it->second->vis->setPosRot(vFromODE(p), qFromODE(q));
        }
    }
}

void Body::activate()
{
    if (_body)
        dBodyDestroy(_body);

    if (!isZero(_mass.mass)){
        DBG("");
        _body = dBodyCreate(world()->world());
        dBodySetData(_body, this);

        dBodySetMass(_body, &_mass);

        for_each(_shapes_it_t, _shapes){
            dGeomSetBody(it->second->shape, _body);
        }
        DBG(_body);
    }

    _enableShape();
    _applyPosRot();
    _enableBody();
    _enableVisBody();
}


void Body::deactivate()
{
    _disableVisBody();
    _disableBody();
    _disableShape();
}

void Body::_applyPosRot()
{
    if (!_body){
        Quat r;
        Vec3 p;
        dQuaternion q;

        for_each(_shapes_it_t, _shapes){
            r = rot() * it->second->rot;
            p = pos() + it->second->pos;
            qToODE(r, q);

            dGeomSetPosition(it->second->shape, p.x(), p.y(), p.z());
            dGeomSetQuaternion(it->second->shape, q);
        }
    }else{
        Vec3 p;
        dQuaternion q;

        // set offsets
        for_each(_shapes_it_t, _shapes){
            p = it->second->pos;
            qToODE(it->second->rot, q);

            dGeomSetOffsetPosition(it->second->shape, p.x(), p.y(), p.z());
            dGeomSetOffsetQuaternion(it->second->shape, q);
        }

        //DBG(DBGV(pos()));
        qToODE(rot(), q);
        dBodySetPosition(_body, pos().x(), pos().y(), pos().z());
        dBodySetQuaternion(_body, q);
    }
}

void Body::_enableBody()
{
    if (_body){
        dBodyEnable(_body);
        dBodySetMovedCallback(_body, _move_cb);
    }
}

void Body::_enableShape()
{
    for_each(_shapes_it_t, _shapes){
        dGeomEnable(it->second->shape);
        dGeomSetData(it->second->shape, this);
    }
}

void Body::_enableVisBody()
{
    VisWorld *vw = world()->visWorld();
    if (!vw)
        return;

    DBG(_body);
    for_each(_shapes_it_t, _shapes){
        if (it->second->vis){
            DBG("");
            vw->addBody(it->second->vis);
        }
    }

    _applyGeomsToVis();
}

void Body::_disableBody()
{
    if (_body){
        dBodyDisable(_body);
    }
}

void Body::_disableShape()
{
    for_each(_shapes_it_t, _shapes){
        dGeomDisable(it->second->shape);
    }
}

void Body::_disableVisBody()
{
    for_each(_shapes_it_t, _shapes){
        // TODO
    }
}



BodyBox::BodyBox(World *w, const Vec3 &dim, Scalar mass, VisBody *vis)
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
    setRot(Quat(Vec3(0., 1., 0.), M_PI * .5));
}

BodyCylinderY::BodyCylinderY(World *w, Scalar radius, Scalar height, Scalar mass,
                             VisBody *vis)
    : BodyCylinder(w, radius, height, mass, vis)
{
    setRot(Quat(Vec3(1., 0., 0.), M_PI * .5));
}

BodyTriMesh::BodyTriMesh(World *w, const sim::Vec3 *coords, size_t coords_len,
                         const unsigned int *ids, size_t ids_len,
                         VisBody *vis)
    : BodySimple(w)
{
    addTriMesh(coords, coords_len, ids, ids_len, vis);
}

} /* namespace ode */

} /* namespace sim */
