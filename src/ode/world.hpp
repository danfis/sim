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
};

} /* namespace ode */

} /* namespace sim */

#endif /* _SIM_ODE_WORLD_HPP_ */

