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
    VisBody *vb = b->visBody();

    pos = dBodyGetPosition(body);
    rot = dBodyGetQuaternion(body);
    b->setPos(pos[0], pos[1], pos[2]);
    b->setRot(rot[1], rot[2], rot[3], rot[0]);

    //DBG(body << " " << vb << " " << DBGV(b->pos()));

    if (vb){
        vb->setPosRot(b->pos(), b->rot());
    }
}

void bodyMovedCBCompound(dBodyID body)
{
    const dReal *pos;
    const dReal *rot;
    BodyCompound *b = (BodyCompound *)dBodyGetData(body);

    pos = dBodyGetPosition(body);
    rot = dBodyGetQuaternion(body);
    b->setPos(pos[0], pos[1], pos[2]);
    b->setRot(rot[1], rot[2], rot[3], rot[0]);

    //DBG(body << " " << DBGV(b->pos()));
    b->_applyGeomsToVis();
}


Body::Body(World *w)
    : _world(w), _body(0), _shape(0), _move_cb(bodyMovedCB)
{
}

Body::~Body()
{
    if (_body)
        dBodyDestroy(_body);
    if (_shape)
        dGeomDestroy(_shape);
}


void Body::_applyPosRot()
{
    dQuaternion q;
    qToODE(rot(), q);

    dGeomSetPosition(_shape, pos().x(), pos().y(), pos().z());
    dGeomSetQuaternion(_shape, q);
}

void Body::_enableShape()
{
    dGeomEnable(_shape);
}

void Body::_enableBody()
{
    if (_body){
        dBodyEnable(_body);
        dBodySetMovedCallback(_body, _move_cb);
    }
}

void Body::_enableVisBody()
{
    VisBody *vis = visBody();
    VisWorld *vw = world()->visWorld();

    if (!vis || vis == SIM_BODY_DEFAULT_VIS || !vw)
        return;

    vis->setPosRot(pos(), rot());
    vw->addBody(vis);
}

void Body::_disableShape()
{
    dGeomDisable(_shape);
}

void Body::_disableBody()
{
    if (_body){
        dBodyDisable(_body);
    }
}

void Body::_disableVisBody()
{
    // TODO
}

void Body::activate()
{
    _applyPosRot();
    _enableShape();
    _enableBody();
    _enableVisBody();
}

void Body::deactivate()
{
    _disableVisBody();
    _disableBody();
    _disableShape();
}

void Body::_set(VisBody *vis, dGeomID shape, dMass *mass)
{
    setVisBody(vis);

    _shape = shape;

    // create body
    _body = dBodyCreate(_world->world());

    // assign shape to body
    dGeomSetBody(_shape, _body);

    // set body's mass
    dBodySetMass(_body, mass);

    // disable body and shape
    dBodyDisable(_body);
    dGeomDisable(_shape);

    // assign reference to this class
    dBodySetData(_body, this);
    dGeomSetData(_shape, this);
}

void Body::_setOnlyShape(VisBody *vis, dGeomID shape)
{
    setVisBody(vis);

    _shape = shape;

    // disable shape
    dGeomDisable(_shape);

    // assign reference to this class
    dGeomSetData(_shape, this);
}

BodyCube::BodyCube(World *world, Scalar w, Scalar mass, VisBody *vis)
    : Body(world)
{
    dGeomID shape;

    shape = dCreateBox(_world->space(), w, w, w);

    if (vis == SIM_BODY_DEFAULT_VIS)
        vis = new VisBodyCube(w);

    if (isZero(mass)){
        _setOnlyShape(vis, shape);
    }else{
        dMass m;
        dMassSetBoxTotal(&m, mass, w, w, w);
        _set(vis, shape, &m);
    }
}

BodyBox::BodyBox(World *w, const Vec3 &dim, Scalar mass, VisBody *vis)
    : Body(w)
{
    dGeomID shape;

    shape = dCreateBox(_world->space(), dim.x(), dim.y(), dim.z());

    if (vis == SIM_BODY_DEFAULT_VIS)
        vis = new VisBodyBox(dim);

    if (isZero(mass)){
        _setOnlyShape(vis, shape);
    }else{
        dMass m;
        dMassSetBoxTotal(&m, mass, dim.x(), dim.y(), dim.z());
        _set(vis, shape, &m);
    }
}

BodySphere::BodySphere(World *w, Scalar radius, Scalar mass, VisBody *vis)
    : Body(w)
{
    dGeomID shape;

    shape = dCreateSphere(_world->space(), radius);

    if (vis == SIM_BODY_DEFAULT_VIS)
        vis = new VisBodySphere(radius);

    if (isZero(mass)){
        _setOnlyShape(vis, shape);
    }else{
        dMass m;
        dMassSetSphereTotal(&m, mass, radius);
        _set(vis, shape, &m);
    }
}

BodyCylinder::BodyCylinder(World *w, Scalar radius, Scalar height, Scalar mass,
                           VisBody *vis)
    : Body(w)
{
    dGeomID shape;

    shape = dCreateCylinder(_world->space(), radius, height);

    if (vis == SIM_BODY_DEFAULT_VIS)
        vis = new VisBodyCylinder(radius, height);

    if (isZero(mass)){
        _setOnlyShape(vis, shape);
    }else{
        dMass m;
        dMassSetCylinderTotal(&m, mass, 3, radius, height);
        _set(vis, shape, &m);
    }
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
                         Scalar mass, VisBody *vis)
    : Body(w)
{
    dGeomID shape;

    shape = geomBuildTriMesh(coords, coords_len, ids, ids_len, _world->space());

    if (vis == SIM_BODY_DEFAULT_VIS)
        vis = new sim::VisBodyTriMesh(coords, coords_len, ids, ids_len);

    if (isZero(mass)){
        _setOnlyShape(vis, shape);
    }else{
        dMass m;
        dMassSetTrimeshTotal(&m, 1., shape);

        // When assigning a mass to a rigid body, the center of mass must be
        // (0,0,0) relative to the body's position!
        dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);

        _set(vis, shape, &m);
    }
}

BodyCompound::BodyCompound(World *w)
    : Body(w), _next_id(1)
{
    setVisBody(0);
    dMassSetZero(&_mass);
    _move_cb = bodyMovedCBCompound;
}

BodyCompound::~BodyCompound()
{
    for_each(_shapes_it_t, _shapes){
        dGeomDestroy(it->second->shape);

        if (it->second->vis)
            delete it->second->vis;

        delete it->second;
    }
    _shapes.clear();
}

BodyCompound::shape_t *BodyCompound::_aShape()
{
    if (_shapes.size() == 0)
        return 0;
    return _shapes.begin()->second;
}

const BodyCompound::shape_t *BodyCompound::_aShape() const
{
    if (_shapes.size() == 0)
        return 0;
    return _shapes.begin()->second;
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

int BodyCompound::_addShape(dGeomID shape, VisBody *vis,
                             const Vec3 &pos, const Quat &rot)
{
    shape_t *s = new shape_t(shape, vis, pos, rot);

    _shapes.insert(_shapes_t::value_type(_next_id, s));

    return _next_id++;
}

int BodyCompound::addCube(Scalar w, VisBody *vis,
                          const Vec3 &pos, const Quat &rot)
{
    dGeomID shape;
    shape = dCreateBox(world()->space(), w, w, w);

    if (vis == SIM_BODY_DEFAULT_VIS)
        vis = new VisBodyCube(w);

    return _addShape(shape, vis, pos, rot);
}

int BodyCompound::addBox(const Vec3 &dim, VisBody *vis,
                         const Vec3 &pos, const Quat &rot)
{
    dGeomID shape;
    shape = dCreateBox(world()->space(), dim.x(), dim.y(), dim.z());

    if (vis == SIM_BODY_DEFAULT_VIS)
        vis = new VisBodyBox(dim);

    return _addShape(shape, vis, pos, rot);
}

int BodyCompound::addSphere(Scalar radius, VisBody *vis,
                            const Vec3 &pos, const Quat &rot)
{
    dGeomID shape;
    shape = dCreateSphere(world()->space(), radius);

    if (vis == SIM_BODY_DEFAULT_VIS)
        vis = new VisBodySphere(radius);

    return _addShape(shape, vis, pos, rot);
}

int BodyCompound::addCylinderZ(Scalar radius, Scalar height, VisBody *vis,
                               const Vec3 &pos, const Quat &rot)
{
    dGeomID shape;
    shape = dCreateCylinder(world()->space(), radius, height);

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
    dGeomID shape;

    shape = geomBuildTriMesh(coords, coords_len, ids, ids_len, _world->space());

    if (vis == SIM_BODY_DEFAULT_VIS)
        vis = new VisBodyTriMesh(coords, coords_len, ids, ids_len);

    return _addShape(shape, vis, pos, rot);
}

void BodyCompound::setMassCube(Scalar w, Scalar mass)
{
    dMassSetBoxTotal(&_mass, mass, w, w, w);
}

void BodyCompound::setMassBox(const Vec3 &dim, Scalar mass)
{
    dMassSetBoxTotal(&_mass, mass, dim.x(), dim.y(), dim.z());
}

void BodyCompound::setMassSphere(Scalar radius, Scalar mass)
{
    dMassSetSphereTotal(&_mass, mass, radius);
}

void BodyCompound::setMassCylinder(Scalar radius, Scalar height, Scalar mass)
{
    dMassSetCylinderTotal(&_mass, mass, 3, radius, height);
}

void BodyCompound::rmShape(int id)
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


void BodyCompound::_applyGeomsToVis()
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

void BodyCompound::activate()
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

    Body::activate();
}

void BodyCompound::_applyPosRot()
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

void BodyCompound::_enableShape()
{
    for_each(_shapes_it_t, _shapes){
        dGeomEnable(it->second->shape);
        dGeomSetData(it->second->shape, this);
    }
}

void BodyCompound::_enableVisBody()
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

void BodyCompound::_disableShape()
{
    for_each(_shapes_it_t, _shapes){
        dGeomDisable(it->second->shape);
    }
}

void BodyCompound::_disableVisBody()
{
    for_each(_shapes_it_t, _shapes){
        // TODO
    }
}

} /* namespace ode */

} /* namespace sim */
