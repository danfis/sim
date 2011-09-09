/***
 * sim
 * ---------------------------------
 * Copyright (c)2010-2011 Daniel Fiser <danfis@danfis.cz>
 *
 *  This file is part of sim.
 *
 *  sim is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation; either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  sim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _SIM_WORLD_HPP_
#define _SIM_WORLD_HPP_

#include <sim/config.hpp>

#include <sim/body.hpp>
#include <sim/joint.hpp>
#include <sim/visworld.hpp>
#include <sim/time.hpp>

namespace sim {


/**
 * Physical representation world.
 */
class World {
  protected:
    VisWorld *_vis; //!< Reference to visual representation

    Vec3 _gravity;

    World() : _vis(0), _gravity(0., 0., -9.81){}

  public:
    /* \{ */
    virtual VisWorld *visWorld() { return _vis; }
    virtual const VisWorld *visWorld() const { return _vis; }
    virtual void setVisWorld(VisWorld *w) { _vis = w; }
    /* \} */

    const Vec3 &gravity() const { return _gravity; }
    void setGravity(const Vec3 &g) { _gravity = g; }

    /* \{ */
    /**
     * Initializes world.
     */
    virtual void init() = 0;

    /**
     * Finalize world.
     */
    virtual void finish() = 0;

    /**
     * Performes one simulation step.
     */
    virtual void step(const Time &time, unsigned int substeps = 1) = 0;

    virtual bool done() = 0;
    /* \} */

    /* \{ */
    virtual Body *createBodyCube(Scalar width, Scalar mass, VisBody *vis = SIM_BODY_DEFAULT_VIS)
        { return 0; }
    virtual Body *createBodyBox(Vec3 dim, Scalar mass, VisBody *vis = SIM_BODY_DEFAULT_VIS)
        { return 0; }
    virtual Body *createBodySphere(Scalar radius, Scalar mass, VisBody *vis = SIM_BODY_DEFAULT_VIS)
        { return 0; }
    virtual Body *createBodyCylinderX(Scalar radius, Scalar height, Scalar mass,
                                      VisBody *vis = SIM_BODY_DEFAULT_VIS)
        { return 0; }
    virtual Body *createBodyCylinderY(Scalar radius, Scalar height, Scalar mass,
                                      VisBody *vis = SIM_BODY_DEFAULT_VIS)
        { return 0; }
    virtual Body *createBodyCylinderZ(Scalar radius, Scalar height, Scalar mass,
                                      VisBody *vis = SIM_BODY_DEFAULT_VIS)
        { return 0; }
    virtual Body *createBodyTriMesh(const Vec3 *coords, size_t coords_len,
                                    const unsigned int *indices, size_t indices_len,
                                    VisBody *vis = SIM_BODY_DEFAULT_VIS)
        { return 0; }
    virtual Body *createBodyCompound()
        { return 0; }

    virtual Joint *createJointFixed(Body *oA, Body *oB) { return 0; }
    virtual Joint *createJointHinge(Body *A, Body *oB,
                                    const Vec3 &anchor, const Vec3 &axis)
        { return 0; }
    virtual Joint *createJointHinge2(Body *A, Body *oB, const Vec3 &anchor,
                                     const Vec3 &axis1, const Vec3 &axis2)
        { return 0; }
    /* \} */
};

class WorldODE : public World {
  protected:
    WorldODE() : World() {}

  public:
    enum StepType {
        STEP_TYPE_NORMAL,
        STEP_TYPE_QUICK
    };

    /* \{ */
    /**
     * Using this method can be changed which type of step will be used.
     * dWorldStep() or dWorldQuickStep().
     * See http://opende.sourceforge.net/wiki/index.php/Manual_(All)#Stepping_Functions
     */
    virtual void setStepType(StepType type) = 0;
    virtual void setQuickStepIterations(int num) = 0;

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
    virtual void setERP(double erp) = 0;
    virtual double erp() const = 0;

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
    virtual void setCFM(double cfm) = 0;
    virtual double cfm() const = 0;

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
    virtual void setContactMu(double mu) = 0;
    virtual double contactMu() const = 0;

    /**
     * Optional Coulomb friction coefficient for friction direction 2
     * (0..dInfinity).
     * Negative number will disable it.
     * In default it's disabled.
     *
     * See http://www.ode.org/ode-latest-userguide.html#sec_7_3_7
     */
    virtual void setContactMu2(double mu) = 0;
    virtual double contactMu2() const = 0;

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
    virtual void setContactBounce(double restitution, double vel) = 0;
    virtual bool contactBounce(double *rest, double *vel) const = 0;

    /**
     * The Constraint Force Mixing parameter of the contact normal.
     * Negative number disable it.
     * 0.0001 by default.
     *
     * See http://www.ode.org/ode-latest-userguide.html#sec_7_3_7
     */
    virtual void setContactSoftCFM(double cfm) = 0;
    virtual double contactSoftCFM() const = 0;

    /**
     * The Error Reduction Parameter of the contact normal.
     * Negative number disable it.
     * Disabled by default.
     *
     * See http://www.ode.org/ode-latest-userguide.html#sec_7_3_7
     */
    virtual void setContactSoftERP(double erp) = 0;
    virtual double contactSoftERP() const = 0;

    /**
     * Use the friction pyramid approximation for friction direction 1. If
     * this is not specified then the constant-force-limit approximation is
     * used (and mu is a force limit).
     * Enabled by default (both 1 and 2).
     *
     * See http://www.ode.org/ode-latest-userguide.html#sec_7_3_7
     * and http://opende.sourceforge.net/wiki/index.php/Manual_(All)#The_Problem
     */
    virtual void setContactApprox1(bool yes = true) = 0;
    virtual void setContactApprox2(bool yes = true) = 0;
    virtual bool contactApprox1() const = 0;
    virtual bool contactApprox2() const = 0;

    /**
     * The coefficients of force-dependent-slip (FDS) for friction
     * directions 1 and 2.
     * Disabled by default.
     * See http://www.ode.org/ode-latest-userguide.html#sec_7_3_7
     */
    virtual void setContactSlip1(double slip) = 0;
    virtual void setContactSlip2(double slip) = 0;
    virtual double contactSlip1() const = 0;
    virtual double contactSlip2() const = 0;

    // TODO: motion{1,2}


    /**
     * Enables auto disabling newly created bodies. A body is disabled it
     * has been idle for given number of simulation steps _and_ it has been
     * idle for a given amount of simulation time.
     * A body is considered to be idle when the magnitudes of both its
     * linear velocity and angular velocity are below given thresholds.
     */
    virtual void setAutoDisable(Scalar linear_treshold = 0.01, Scalar angular_treshold = 0.01,
                                int steps = 10, Scalar time = 0) = 0;
    virtual void resetAutoDisable() = 0;
    /* \} */

};

class WorldBullet : public World {
  protected:
    WorldBullet() : World() {}

  public:
};


/**
 * Factory for producing physical world.
 */
namespace WorldFactory {
    WorldODE *ODE();
    WorldBullet *Bullet();
} /* namespace WorldFactory */


} /* namespace sim */

#endif /* _SIM_WORLD_HPP_ */

