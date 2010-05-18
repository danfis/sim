#include "sim/ode/body.hpp"
#include "sim/ode/world.hpp"
#include "sim/ode/math.hpp"
#include "sim/msg.hpp"

namespace sim {

namespace ode {

static void bodyMovedCB(dBodyID body)
{
    Body *b = (Body *)dBodyGetData(body);
    VisBody *vb = b->visBody();

    if (vb){
        Vec3 pos = b->pos();
        Quat rot = b->rot();
        vb->setPosRot(pos, rot);

        //DBG(b << " " << DBGV(pos));
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

void Body::setPos(const Vec3 &v)
{
    dGeomSetPosition(_shape, v.x(), v.y(), v.z());
}

void Body::setRot(const Quat &q)
{
    dQuaternion quat;
    qToODE(q, quat);
    dGeomSetQuaternion(_shape, quat);
}

void Body::pos(Scalar *x, Scalar *y, Scalar *z) const
{
    const dReal *pos;
    pos = dGeomGetPosition(_shape);
    *x = pos[0];
    *y = pos[1];
    *z = pos[2];
}
Vec3 Body::pos() const
{
    const dReal *pos;
    pos = dGeomGetPosition(_shape);
    return Vec3(pos[0], pos[1], pos[2]);
}

void Body::rot(Scalar *x, Scalar *y, Scalar *z, Scalar *w) const
{
    dQuaternion q;
    dGeomGetQuaternion(_shape, q);
    *x = q[1];
    *y = q[2];
    *z = q[3];
    *w = q[0];
}

Quat Body::rot() const
{
    dQuaternion q;
    dGeomGetQuaternion(_shape, q);
    return Quat(q[1], q[2], q[3], q[0]);
}

void Body::setPosRot(const Vec3 &v, const Quat &q)
{
    setPos(v);
    setRot(q);
}

void Body::activate()
{
    if (_body){
        dBodyEnable(_body);
        dBodySetMovedCallback(_body, bodyMovedCB);
        bodyMovedCB(_body);
    }else if (visBody() && visBody() != SIM_BODY_DEFAULT_VIS){
        visBody()->setPosRot(pos(), rot());
    }

    if (visBody() && visBody() != SIM_BODY_DEFAULT_VIS && _world->visWorld()){
        _world->visWorld()->addBody(visBody());
    }
}

void Body::deactivate()
{
    if (_body){
        dBodyDisable(_body);
    }
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

    // disable body
    dBodyDisable(_body);

    // assign reference to this class
    dBodySetData(_body, this);
    dGeomSetData(_shape, this);
}

void Body::_setOnlyShape(VisBody *vis, dGeomID shape)
{
    setVisBody(vis);

    _shape = shape;

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

BodyBox::BodyBox(World *w, Vec3 dim, Scalar mass, VisBody *vis)
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
    dQuaternion quat;
    qToODE(Quat(Vec3(0., 1., 0.), M_PI * .5), quat);
    dGeomSetQuaternion(_shape, quat);
}

BodyCylinderY::BodyCylinderY(World *w, Scalar radius, Scalar height, Scalar mass,
                             VisBody *vis)
    : BodyCylinder(w, radius, height, mass, vis)
{
    dQuaternion quat;
    qToODE(Quat(Vec3(1., 0., 0.), M_PI * .5), quat);
    dGeomSetQuaternion(_shape, quat);
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
        dMassSetTrimeshTotal(&m, mass, shape);
        _set(vis, shape, &m);
    }
}

} /* namespace ode */

} /* namespace sim */
