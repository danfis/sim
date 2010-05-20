#ifndef _SIM_ODE_WORLD_HPP_
#define _SIM_ODE_WORLD_HPP_

#include <ode/ode.h>

#include "sim/visworld.hpp"
#include "sim/world.hpp"
#include "sim/ode/body.hpp"
#include "sim/ode/joint.hpp"
#include "sim/ode/actuator.hpp"

namespace sim {

namespace ode {

void __collision (void *data, dGeomID o1, dGeomID o2);

/**
 * Physical representation world.
 */
class World : public sim::World {
  public:
    enum StepType {
        STEP_TYPE_NORMAL,
        STEP_TYPE_QUICK
    };

  protected:
    dWorldID _world;
    dSpaceID _space;
    dJointGroupID _coll_contacts;

    dContact _default_contact; /*! default setting of contact */

    StepType _step_type;


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
     * Using this method can be changed which type of step will be used.
     * dWorldStep() or dWorldQuickStep().
     * See http://opende.sourceforge.net/wiki/index.php/Manual_(All)#Stepping_Functions
     */
    void setStepType(StepType type);
    void setQuickStepIterations(int num);

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
    double erp() const;

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
    double cfm() const;

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
    double contactMu() const;

    /**
     * Optional Coulomb friction coefficient for friction direction 2
     * (0..dInfinity).
     * Negative number will disable it.
     * In default it's disabled.
     *
     * See http://www.ode.org/ode-latest-userguide.html#sec_7_3_7
     */
    void setContactMu2(double mu);
    double contactMu2() const;

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
    bool contactBounce(double *rest, double *vel) const;

    /**
     * The Constraint Force Mixing parameter of the contact normal.
     * Negative number disable it.
     * 0.0001 by default.
     *
     * See http://www.ode.org/ode-latest-userguide.html#sec_7_3_7
     */
    void setContactSoftCFM(double cfm);
    double contactSoftCFM() const;

    /**
     * The Error Reduction Parameter of the contact normal.
     * Negative number disable it.
     * Disabled by default.
     *
     * See http://www.ode.org/ode-latest-userguide.html#sec_7_3_7
     */
    void setContactSoftERP(double erp);
    double contactSoftERP() const;

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
    bool contactApprox1() const;
    bool contactApprox2() const;

    /**
     * The coefficients of force-dependent-slip (FDS) for friction
     * directions 1 and 2.
     * Disabled by default.
     * See http://www.ode.org/ode-latest-userguide.html#sec_7_3_7
     */
    void setContactSlip1(double slip);
    void setContactSlip2(double slip);
    double contactSlip1() const;
    double contactSlip2() const;

    // TODO: motion{1,2}


    /**
     * Enables auto disabling newly created bodies. A body is disabled it
     * has been idle for given number of simulation steps _and_ it has been
     * idle for a given amount of simulation time.
     * A body is considered to be idle when the magnitudes of both its
     * linear velocity and angular velocity are below given thresholds.
     */
    void setAutoDisable(Scalar linear_treshold = 0.01, Scalar angular_treshold = 0.01,
                        int steps = 10, Scalar time = 0);
    void resetAutoDisable();
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
    sim::Body *createBodyCompound();

    sim::Joint *createJointFixed(sim::Body *oA, sim::Body *oB);
    sim::Joint *createJointHinge(sim::Body *A, sim::Body *oB,
                                 const Vec3 &anchor, const Vec3 &axis);
    sim::Joint *createJointHinge2(sim::Body *A, sim::Body *oB, const Vec3 &anchor,
                                  const Vec3 &axis1, const Vec3 &axis2);

    sim::ActuatorWheelCylinderX *createActuatorWheelCylinderX
                                    (Scalar radius, Scalar height, Scalar mass);

  protected:
    void _contactEnableMode(int mode);
    void _contactDisableMode(int mode);
    bool _contactEnabledMode(int mode) const;
};

} /* namespace ode */

} /* namespace sim */

#endif /* _SIM_ODE_WORLD_HPP_ */

