#ifndef _SIM_ODE_WORLD_HPP_
#define _SIM_ODE_WORLD_HPP_

#include <ode/ode.h>

#include "sim/visworld.hpp"
#include "sim/world.hpp"
#include "sim/ode/body.hpp"
/*
#include "sim/ode/joint.hpp"
#include "sim/ode/actuator.hpp"
#include "sim/ode/collision_detection.hpp"
*/

namespace sim {

namespace ode {

void __collision (void *data, dGeomID o1, dGeomID o2);

/**
 * Physical representation world.
 */
class World : public sim::World {
  protected:
    dWorldID _world;
    dSpaceID _space;
    dJointGroupID _coll_contacts;

    dContact _default_contact; /*! default setting of contact */


    friend void __collision(void *, dGeomID, dGeomID);

  public:
    World();
    virtual ~World();

    dWorldID world() { return _world; }
    const dWorldID world() const { return _world; }
    dSpaceID space() { return _space; }
    const dSpaceID space() const { return _space; }

    /* \{ */
    /**
     * Sets up Error Reduction Parameter.
     * Value should be between 0 and 1.
     * Default is 0.2.
     *
     * The ERP specifies what proportion of the joint error will be fixed
     * during the next simulation step. If ERP=0 then no correcting force
     * is applied and the bodies will eventually drift apart as the
     * simulation proceeds. If ERP=1 then the simulation will attempt to
     * fix all joint error during the next time step. However, setting
     * ERP=1 is not recommended, as the joint error will not be completely
     * fixed due to various internal approximations. A value of ERP=0.1 to
     * 0.8 is recommended (0.2 is the default).
     * See http://www.ode.org/ode-latest-userguide.html#sec_3_7_0
     */
    void setERP(double erp);

    /**
     * Sets up Constraint Force Mixing.
     * Default is 1e-5;
     *
     * Increasing CFM, especially the global CFM, can reduce the numerical
     * errors in the simulation. If the system is near-singular, then this
     * can markedly increase stability. In fact, if the system is
     * mis-behaving, one of the first things to try is to increase the
     * global CFM.
     * See http://www.ode.org/ode-latest-userguide.html#sec_3_8_0
     */
    void setCFM(double cfm);

    /**
     * Coulomb friction coefficient. This must be in the range 0 to
     * dInfinity. 0 results in a frictionless contact, and dInfinity
     * results in a contact that never slips. Note that frictionless
     * contacts are less time consuming to compute than ones with friction,
     * and infinite friction contacts can be cheaper than contacts with
     * finite friction. This must always be set.
     * Default is dInfinity.
     * See http://www.ode.org/ode-latest-userguide.html#sec_3_11_1
     * and http://www.ode.org/ode-latest-userguide.html#sec_7_3_7
     */
    void setContactMu(double mu);

    /**
     * Optional Coulomb friction coefficient for friction direction 2
     * (0..dInfinity).
     * Negative number will disable it.
     * In default it's disabled.
     *
     * See http://www.ode.org/ode-latest-userguide.html#sec_7_3_7
     */
    void setContactMu2(double mu);

    /**
     * If set, the contact surface is bouncy, in other words the bodies
     * will bounce off each other.
     * Restitution parameter (0..1). 0 means the surfaces are not bouncy at
     * all, 1 is maximum bouncyness.
     * Vel parameter is the minimum incoming velocity necessary for bounce.
     * Incoming velocities below this will effectively have a restitution
     * parameter of 0.
     * If restitution is set to negative number mode is disabled.
     * Disabled by default.
     *
     * See http://www.ode.org/ode-latest-userguide.html#sec_7_3_7
     */
    void setContactBounce(double restitution, double vel);

    /**
     * The Constraint Force Mixing parameter of the contact normal.
     * Negative number disable it.
     * 0.0001 by default.
     *
     * See http://www.ode.org/ode-latest-userguide.html#sec_7_3_7
     */
    void setContactSoftCFM(double cfm);

    /**
     * The Error Reduction Parameter of the contact normal.
     * Negative number disable it.
     * Disabled by default.
     *
     * See http://www.ode.org/ode-latest-userguide.html#sec_7_3_7
     */
    void setContactSoftERP(double erp);

    /**
     * Use the friction pyramid approximation for friction direction 1. If
     * this is not specified then the constant-force-limit approximation is
     * used (and mu is a force limit).
     * Enabled by default (both 1 and 2).
     *
     * See http://www.ode.org/ode-latest-userguide.html#sec_7_3_7
     * and http://opende.sourceforge.net/wiki/index.php/Manual_(All)#The_Problem
     */
    void setContactApprox1(bool yes = true);
    void setContactApprox2(bool yes = true);

    // TODO: slip{1,2}, motion{1,2}
    /* \} */


    /**
     * Initializes world.
     */
    void init();

    /**
     * Finalize world.
     */
    void finish();

    /**
     * Performes one step.
     */
    void step(const sim::Time &time, unsigned int substeps = 1);

    bool done();

    sim::Body *createBodyCube(Scalar width, Scalar mass, VisBody *vis = SIM_BODY_DEFAULT_VIS);
    sim::Body *createBodyBox(Vec3 dim, Scalar mass, VisBody *vis = SIM_BODY_DEFAULT_VIS);
    sim::Body *createBodySphere(Scalar radius, Scalar mass, VisBody *vis = SIM_BODY_DEFAULT_VIS);
    sim::Body *createBodyCylinderX(Scalar radius, Scalar height, Scalar mass,
                                   VisBody *vis = SIM_BODY_DEFAULT_VIS);
    sim::Body *createBodyCylinderY(Scalar radius, Scalar height, Scalar mass,
                                   VisBody *vis = SIM_BODY_DEFAULT_VIS);
    sim::Body *createBodyCylinderZ(Scalar radius, Scalar height, Scalar mass,
                                   VisBody *vis = SIM_BODY_DEFAULT_VIS);
    sim::Body *createBodyTriMesh(const Vec3 *coords, size_t coords_len,
                                 const unsigned int *indices, size_t indices_len,
                                 Scalar mass, VisBody *vis = SIM_BODY_DEFAULT_VIS);

    /*
    sim::Joint *createJointFixed(sim::Body *oA, sim::Body *oB);
    sim::Joint *createJointHinge(sim::Body *A, sim::Body *oB,
                                 const Vec3 &anchor, const Vec3 &axis);
    sim::Joint *createJointHinge2(sim::Body *A, sim::Body *oB, const Vec3 &anchor,
                                  const Vec3 &axis1, const Vec3 &axis2);

    sim::ActuatorWheelCylinderX *createActuatorWheelCylinderX
                                    (Scalar radius, Scalar height, Scalar mass);
                                    */


    /**
     * Adds joint to world. This function is used by .activate() method of
     * Joint and should NOT be used directly.
     */
    //void addJoint(Joint *j);

    /**
     * Adds body to world. This function is used by .activate() method of
     * Body and should NOT be used directly.
     */
    //void addBody(Body *obj);

  protected:
    void _contactEnableMode(int mode);
    void _contactDisableMode(int mode);
};

} /* namespace ode */

} /* namespace sim */

#endif /* _SIM_ODE_WORLD_HPP_ */
