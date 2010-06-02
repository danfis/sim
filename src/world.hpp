#ifndef _SIM_WORLD_HPP_
#define _SIM_WORLD_HPP_

#include "sim/body.hpp"
#include "sim/joint.hpp"
#include "sim/visworld.hpp"
#include "sim/time.hpp"

namespace sim {


/**
 * Physical representation world.
 */
class World {
  protected:
    VisWorld *_vis; //!< Reference to visual representation

    Vec3 _gravity;

  public:
    World() : _vis(0), _gravity(0., 0., -9.81){}

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
                                    Scalar mass, VisBody *vis = SIM_BODY_DEFAULT_VIS)
        { return 0; }
    virtual Body *createBodyCompound()
        { return 0; }

    virtual Joint *createJointFixed(Body *oA, Body *oB) { return 0; }
    virtual Joint *createJointHinge(Body *A, Body *oB,
                                    const Vec3 &anchor, const Vec3 &axis,
                                    Scalar axis_offset = 0.)
        { return 0; }
    virtual Joint *createJointHinge2(Body *A, Body *oB, const Vec3 &anchor,
                                     const Vec3 &axis1, const Vec3 &axis2)
        { return 0; }
    /* \} */
};

} /* namespace sim */

#endif /* _SIM_WORLD_HPP_ */

