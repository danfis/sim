#ifndef _SIM_WORLD_HPP_
#define _SIM_WORLD_HPP_

#include "body.hpp"
#include "joint.hpp"
#include "actuator.hpp"
#include "visworld.hpp"

namespace sim {


/**
 * Physical representation world.
 */
class World {
  protected:
    VisWorld *_vis; //!< Reference to visual representation

  public:
    /* \{ */
    virtual VisWorld *visWorld() { return _vis; }
    virtual const VisWorld *visWorld() const { return _vis; }
    virtual void setVisWorld(VisWorld *w) { return _vis = w; }
    /* \} */

    /* \{ */
    /**
     * Initializes world.
     */
    virtual void init() = 0;

    /**
     * Destroys world.
     */
    virtual void destroy() = 0;

    /**
     * Performes one simulation step.
     */
    virtual void step() = 0;

    virtual bool done() = 0;
    /* \} */

    /* \{ */
    virtual Body *createBodyCube(Scalar width, Scalar mass) { return 0; }
    virtual Body *createBodyBox(Vec3 dim, Scalar mass) { return 0; }
    virtual Body *createSphere(Scalar radius, Scalar mass) { return 0; }
    virtual Body *createBodyCylinderX(Scalar radius, Scalar height, Scalar mass)
        { return 0; }
    virtual Body *createBodyCylinderY(Scalar radius, Scalar height, Scalar mass)
        { return 0; }
    virtual Body *createBodyCylinderZ(Scalar radius, Scalar height, Scalar mass)
        { return 0; }

    virtual Joint *createJointFixed(Body *oA, Body *oB) { return 0; }
    virtual Joint *createJointHinge(Body *A, Body *oB,
                                    const Vec3 &anchor, const Vec3 &axis)
        { return 0; }
    virtual Joint *createJointHinge2(Body *A, Body *oB, const Vec3 &anchor,
                                     const Vec3 &axis1, const Vec2 &axis2)
        { return 0; }

    virtual ActuatorWheelCylinderX *createActuatorCylinderX
                                        (Scalar radius, Scalar height,
                                         Scalar mass)
        { return 0; }
    /* \} */
};

}

#endif /* _SIM_WORLD_HPP_ */

