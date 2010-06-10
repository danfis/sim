/***
 * sim
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
 *
 *  This file is part of sim.
 *
 *  sim is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation; either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  sim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

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

} /* namespace sim */

#endif /* _SIM_WORLD_HPP_ */

