#include "sim/ode/body.hpp"
#include "sim/ode/world.hpp"
#include "sim/ode/math.hpp"
#include "sim/msg.hpp"
#include "sim/common.hpp"

namespace sim {

namespace ode {

static void bodyMovedCB(dBodyID body)
{
    const dReal *pos;
    const dReal *rot;
    Body *b = (Body *)dBodyGetData(body);
    VisBody *vb = b->visBody();

    pos = dBodyGetPosition(body);
    rot = dBodyGetQuaternion(body);
    b->setPos(pos[0], pos[1], pos[2]);
    b->setRot(rot[1], rot[2], rot[3], rot[0]);

    DBG(body << " " << DBGV(b->pos()));

    if (vb){
        vb->setPosRot(b->pos(), b->rot());
    }
}


Body::Body(World *w)
    : _world(w), _body(0), _shape(0)
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
        dBodySetMovedCallback(_body, bodyMovedCB);
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
    DBG(_body);
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

    shape = dCreateTriMesh(_world->space(), data, 0, 0, 0);

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

} /* namespace ode */

} /* namespace sim */
