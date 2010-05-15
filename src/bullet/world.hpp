#ifndef _SIM_BULLET_WORLD_HPP_
#define _SIM_BULLET_WORLD_HPP_

#include <BulletCollision/CollisionDispatch/btCollisionConfiguration.h>
#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>
#include <BulletCollision/BroadphaseCollision/btBroadphaseInterface.h>
#include <BulletDynamics/ConstraintSolver/btConstraintSolver.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>

#include "sim/visworld.hpp"
#include "sim/world.hpp"
#include "sim/bullet/body.hpp"
#include "sim/bullet/joint.hpp"
#include "sim/bullet/actuator.hpp"
#include "sim/bullet/collision_detection.hpp"

namespace sim {

namespace bullet {


/**
 * Physical representation world.
 */
class World : public sim::World {
  protected:
    btCollisionConfiguration *_coll_conf;
    CollisionDispatcher *_dispatch;
    btBroadphaseInterface *_broadphase;
    btConstraintSolver *_solver;
    btDynamicsWorld *_world;

  public:
    World();
    virtual ~World();

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

    sim::Body *createBodyCube(Scalar width, Scalar mass);
    sim::Body *createBodyBox(Vec3 dim, Scalar mass);
    sim::Body *createBodySphere(Scalar radius, Scalar mass);
    sim::Body *createBodyCylinderX(Scalar radius, Scalar height, Scalar mass);
    sim::Body *createBodyCylinderY(Scalar radius, Scalar height, Scalar mass);
    sim::Body *createBodyCylinderZ(Scalar radius, Scalar height, Scalar mass);
    sim::Body *createBodyTriMesh(const Vec3 *coords, size_t coords_len,
                                 const unsigned int *indices, size_t indices_len);

    sim::Joint *createJointFixed(sim::Body *oA, sim::Body *oB);
    sim::Joint *createJointHinge(sim::Body *A, sim::Body *oB,
                                 const Vec3 &anchor, const Vec3 &axis);
    sim::Joint *createJointHinge2(sim::Body *A, sim::Body *oB, const Vec3 &anchor,
                                  const Vec3 &axis1, const Vec3 &axis2);

    sim::ActuatorWheelCylinderX *createActuatorWheelCylinderX
                                    (Scalar radius, Scalar height, Scalar mass);


    /**
     * Adds joint to world. This function is used by .activate() method of
     * Joint and should NOT be used directly.
     */
    void addJoint(Joint *j);

    /**
     * Adds body to world. This function is used by .activate() method of
     * Body and should NOT be used directly.
     */
    void addBody(Body *obj);
};

} /* namespace bullet */

} /* namespace sim */

#endif /* _SIM_BULLET_WORLD_HPP_ */

