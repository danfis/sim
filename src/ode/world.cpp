#include <algorithm>

#include "sim/ode/world.hpp"
#include "sim/msg.hpp"

namespace sim {

namespace ode {

void __collision (void *data, dGeomID o1, dGeomID o2)
{
    World *world = (World *)data;
    Body *b1, *b2;

    b1 = (Body *)dGeomGetData(o1);
    b2 = (Body *)dGeomGetData(o2);

    if (b1 && b2){
        if (b1->collInfo().dont_collide_id == b2->collInfo().dont_collide_id
                && b1->collInfo().dont_collide_id != 0){
            return;
        }
    }

    if (dGeomIsSpace(o1) || dGeomIsSpace(o2)){
        dSpaceCollide2(o1, o2, data, &__collision);

        if (dGeomIsSpace (o1))
            dSpaceCollide((dSpaceID)o1, data, &__collision);
        if (dGeomIsSpace (o2))
            dSpaceCollide((dSpaceID)o2, data, &__collision);

        return;
    }   

    const int N = 32; 
    dContact contact[N];
    dJointID joint;
    dContactGeom geoms[N];

    // store contacts in geoms array
    int n = dCollide(o1, o2, N, geoms, sizeof(dContactGeom));

    if (n > 0){ 
        //DBG("Collides: " << n << " - " << o1 << " " << o2);
        for (int i=0; i<n; i++){
            // copy default contact setting
            contact[i] = world->_default_contact;
            // apply geom (pair of geoms)
            contact[i].geom = geoms[i];

            // create joint
            joint = dJointCreateContact(world->_world, world->_coll_contacts, &contact[i]);
            dJointAttach(joint, dGeomGetBody(contact[i].geom.g1),
                         dGeomGetBody(contact[i].geom.g2));
        }
    }
}


World::World()
    : sim::World()
{
    dInitODE2(0);

    _world = dWorldCreate();
    _space = dHashSpaceCreate(0);
    _coll_contacts = dJointGroupCreate(0);
   
    // set default parameters
    dWorldSetERP(_world, 0.2);
    dWorldSetCFM(_world, 1e-5);
    //dWorldSetERP(_world, 0.5);
    //dWorldSetCFM(_world, 0.001);
    //dWorldSetCFM(_world, 0.01);
   
    _default_contact.surface.mu = dInfinity;
    _default_contact.surface.mode = dContactSoftCFM | dContactApprox1_1 | dContactApprox1_2;
    _default_contact.surface.soft_cfm = 0.0001;

    /*
    _default_contact.surface.slip1 = 0.7;
    _default_contact.surface.slip2 = 0.7;
    _default_contact.surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactSlip1 | dContactSlip2;
    _default_contact.surface.mu = 50.;
    //_default_contact.surface.mu = dInfinity;
    _default_contact.surface.soft_erp = 0.96;
    _default_contact.surface.soft_cfm = 0.04;

    _default_contact.surface.mode = dContactBounce | dContactSoftCFM;
    _default_contact.surface.mu = dInfinity;
    _default_contact.surface.mu2 = 0;
    _default_contact.surface.bounce = 0.1;
    _default_contact.surface.bounce_vel = 0.1;
    _default_contact.surface.soft_cfm = 0.001;
    */
}

World::~World()
{
    if (_world)
        dWorldDestroy(_world);
    if (_space)
        dSpaceDestroy(_space);
    if (_coll_contacts)
        dJointGroupDestroy(_coll_contacts);
}

void World::_contactEnableMode(int mode)
{
    _default_contact.surface.mode |= mode;
}

void World::_contactDisableMode(int mode)
{
    _default_contact.surface.mode &= ~mode;
}

bool World::_contactEnabledMode(int mode) const
{
    return (_default_contact.surface.mode & mode) != 0;
}

void World::setERP(double erp)
{
    dWorldSetERP(_world, erp);
}

double World::erp() const
{
    return dWorldGetERP(_world);
}

void World::setCFM(double cfm)
{
    dWorldSetCFM(_world, cfm);
}

double World::cfm() const
{
    return dWorldGetCFM(_world);
}

void World::setContactMu(double mu)
{
    _default_contact.surface.mu = mu;
}

double World::contactMu() const
{
    return _default_contact.surface.mu;
}

void World::setContactMu2(double mu)
{
    if (mu < 0.){
        _contactDisableMode(dContactMu2);
    }else{
        _contactEnableMode(dContactMu2);
        _default_contact.surface.mu2 = mu;
    }
}

double World::contactMu2() const
{
    if (!_contactEnabledMode(dContactMu2)){
        return -1.;
    }

    return _default_contact.surface.mu2;
}

void World::setContactBounce(double restitution, double vel)
{
    if (restitution < 0.){
        _contactDisableMode(dContactBounce);
    }else{
        _contactEnableMode(dContactBounce);
        _default_contact.surface.bounce = restitution;
        _default_contact.surface.bounce_vel = vel;
    }
}

bool World::contactBounce(double *rest, double *vel) const
{
    *rest = _default_contact.surface.bounce;
    *vel  = _default_contact.surface.bounce_vel;
    return _contactEnabledMode(dContactBounce);
}

void World::setContactSoftCFM(double cfm)
{
    if (cfm < 0.){
        _contactDisableMode(dContactSoftCFM);
    }else{
        _contactEnableMode(dContactSoftCFM);
        _default_contact.surface.soft_cfm = cfm;
    }
}

double World::contactSoftCFM() const
{
    if (!_contactEnabledMode(dContactSoftCFM)){
        return -1.;
    }else{
        return _default_contact.surface.soft_cfm;
    }
}

void World::setContactSoftERP(double erp)
{
    if (erp < 0.){
        _contactDisableMode(dContactSoftERP);
    }else{
        _contactEnableMode(dContactSoftERP);
        _default_contact.surface.soft_erp = erp;
    }
}

double World::contactSoftERP() const
{
    if (!_contactEnabledMode(dContactSoftERP)){
        return -1.;
    }else{
        return _default_contact.surface.soft_erp;
    }
}

void World::setContactApprox1(bool yes)
{
    if (yes){
        _contactEnableMode(dContactApprox1_1);
    }else{
        _contactDisableMode(dContactApprox1_1);
    }
}

void World::setContactApprox2(bool yes)
{
    if (yes){
        _contactEnableMode(dContactApprox1_2);
    }else{
        _contactDisableMode(dContactApprox1_2);
    }
}

bool World::contactApprox1() const
{
    return _contactEnabledMode(dContactApprox1_1);
}

bool World::contactApprox2() const
{
    return _contactEnabledMode(dContactApprox1_2);
}

void World::setContactSlip1(double slip)
{
    if (slip < 0.){
        _contactDisableMode(dContactSlip1);
    }else{
        _contactEnableMode(dContactSlip1);
        _default_contact.surface.slip1 = slip;
    }
}

void World::setContactSlip2(double slip)
{
    if (slip < 0.){
        _contactDisableMode(dContactSlip2);
    }else{
        _contactEnableMode(dContactSlip2);
        _default_contact.surface.slip2 = slip;
    }
}

double World::contactSlip1() const
{
    if (!_contactEnabledMode(dContactSlip1)){
        return -1.;
    }else{
        return _default_contact.surface.slip1;
    }
}

double World::contactSlip2() const
{
    if (!_contactEnabledMode(dContactSlip2)){
        return -1.;
    }else{
        return _default_contact.surface.slip2;
    }
}

void World::init()
{
    dWorldSetGravity(_world, _gravity.x(), _gravity.y(), _gravity.z());
}

void World::finish()
{
}

void World::step(const sim::Time &time, unsigned int substeps)
{
    Scalar fixed = time.inSF() / (double)substeps;

    //DBG(fixed << " " << time);

    for (size_t i = 0; i < substeps; i++){
        dSpaceCollide(_space, this, __collision);
        dWorldStep(_world, fixed);
        //dWorldStepFast1(_world, fixed, 5);
        dJointGroupEmpty(_coll_contacts);
    }
}

bool World::done()
{
    return false;
}



/*
void World::addBody(Body *obj)
{
    // TODO
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
    // TODO
    btTypedConstraint *c = j->joint();

    DBG(c);
    if (c)
        _world->addConstraint(c);
}
*/






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
    return new BodyTriMesh(this, coords, coords_len, indices, indices_len, mass, vis);
}

/*
sim::Joint *World::createJointFixed(sim::Body *oA, sim::Body *oB)
{
    return new JointFixed(this, (Body *)oA, (Body *)oB);
}

sim::Joint *World::createJointHinge(sim::Body *oA, sim::Body *oB,
                                    const Vec3 &anchor, const Vec3 &axis)
{
    return new JointHinge(this, (Body *)oA, (Body *)oB, anchor, axis);
}

sim::Joint *World::createJointHinge2(sim::Body *oA, sim::Body *oB, const Vec3 &anchor,
                                     const Vec3 &axis1, const Vec3 &axis2)
{
    return new JointHinge2(this, (Body *)oA, (Body *)oB, anchor, axis1, axis2);
}


sim::ActuatorWheelCylinderX *World::createActuatorWheelCylinderX
                                (Scalar radius, Scalar height, Scalar mass)
{
    return new ActuatorWheelCylinderX(this, radius, height, mass);
}
*/


} /* namespace ode */

} /* namespace sim */

