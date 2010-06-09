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

#ifndef _SIM_ODE_OBJ_HPP_
#define _SIM_ODE_OBJ_HPP_

#include <ode/ode.h>

#include "sim/body.hpp"
#include "sim/visbody.hpp"

namespace sim {

namespace ode {

// Forward declaration
class World;

void bodyMovedCB(dBodyID body);


class Body : public sim::Body {
  protected:
    struct shape_t {
        dGeomID shape;
        VisBody *vis;
        Vec3 pos;
        Quat rot;
        shape_t(dGeomID s, VisBody *v, const Vec3 &pos, const Quat &rot)
            : shape(s), vis(v), pos(pos), rot(rot) {}
    };
    typedef std::map<int, shape_t *> _shapes_t;
    typedef _shapes_t::iterator _shapes_it_t;
    typedef _shapes_t::const_iterator _shapes_cit_t;

    World *_world;
    dBodyID _body;
    _shapes_t _shapes;
    void (*_move_cb)(dBodyID);
    VisBody *_vis;

    int _next_id;

    dMass _mass;

    friend void bodyMovedCB(dBodyID);

  public:
    Body(World *w);
    virtual ~Body();

    /* \{ */
    dBodyID body() { return _body; }
    const dBodyID body() const { return _body; }
    World *world() { return _world; }
    const World *world() const { return _world; }
    /* \} */

    /* \{ */
    /**
     * Adds cube to compound shape.
     * Visual representation can be provided.
     * Parameters pos_offset and rot_offset define relative offset of shape
     * from origin (0., 0., 0.).
     * Returns ID of shape that can be used lately for modifications.
     */
    int addCube(Scalar width,
                VisBody *vis = SIM_BODY_DEFAULT_VIS,
                const Vec3 &pos_offset = Vec3(0., 0., 0.),
                const Quat &rot_offset = Quat(0., 0., 0., 1.));
    int addBox(const Vec3 &dim,
               VisBody *vis = SIM_BODY_DEFAULT_VIS,
               const Vec3 &pos_offset = Vec3(0., 0., 0.),
               const Quat &rot_offset = Quat(0., 0., 0., 1.));
    int addSphere(Scalar radius,
                  VisBody *vis = SIM_BODY_DEFAULT_VIS,
                  const Vec3 &pos_offset = Vec3(0., 0., 0.),
                  const Quat &rot_offset = Quat(0., 0., 0., 1.));
    int addCylinderZ(Scalar radius, Scalar height,
                     VisBody *vis = SIM_BODY_DEFAULT_VIS,
                     const Vec3 &pos_offset = Vec3(0., 0., 0.),
                     const Quat &rot_offset = Quat(0., 0., 0., 1.));
    int addCylinderY(Scalar radius, Scalar height,
                     VisBody *vis = SIM_BODY_DEFAULT_VIS,
                     const Vec3 &pos_offset = Vec3(0., 0., 0.),
                     const Quat &rot_offset = Quat(0., 0., 0., 1.));
    int addCylinderX(Scalar radius, Scalar height,
                     VisBody *vis = SIM_BODY_DEFAULT_VIS,
                     const Vec3 &pos_offset = Vec3(0., 0., 0.),
                     const Quat &rot_offset = Quat(0., 0., 0., 1.));
    int addTriMesh(const sim::Vec3 *coords, size_t coords_len,
                   const unsigned int *indices, size_t indices_len,
                   VisBody *vis = SIM_BODY_DEFAULT_VIS,
                   const Vec3 &pos_offset = Vec3(0., 0., 0.),
                   const Quat &rot_offset = Quat(0., 0., 0., 1.));

    /**
     * Removes shape with ID that was returned by add.. method.
     */
    void rmShape(int ID);
    /* \} */

    /* \{ */
    void setMassCube(Scalar width, Scalar mass);
    void setMassBox(const Vec3 &dim, Scalar mass);
    void setMassSphere(Scalar radius, Scalar mass);
    void setMassCylinder(Scalar radius, Scalar height, Scalar mass);
    /* \} */

    /**
     * Returns pointer to VisBody of shape with specified ID.
     */
    VisBody *visBody(int ID);
    const VisBody *visBody(int ID) const;
    void visBodyAll(std::list<const VisBody *> *list) const;
    void visBodyAll(std::list<VisBody *> *list);

    void activate();
    void deactivate();

  protected:
    shape_t *_aShape();
    const shape_t *_aShape() const;
    int _addShape(dGeomID shape, VisBody *vis, const Vec3 &pos, const Quat &rot);

    shape_t *shape(int ID);
    const shape_t *shape(int ID) const;

    void _applyGeomsToVis();
    void _applyPosRot();
    void _enableBody();
    void _enableShape();
    void _enableVisBody();
    void _disableBody();
    void _disableShape();
    void _disableVisBody();
};

class BodySimple : public Body {
  public:
    BodySimple(World *w) : Body(w) {}
    virtual ~BodySimple() {}

    VisBody *visBody() { return shape(1)->vis; }
    const VisBody *visBody() const { return shape(1)->vis; }
    void setVisBody(VisBody *vis) { shape(1)->vis = vis; }
};

/**
 * Body with general box shape.
 * Constructot takes vector with lengths of three box's edges and amount of
 * mass.
 */
class BodyBox : public BodySimple {
  public:
    BodyBox(World *w, const Vec3 &dim, Scalar mass,
            VisBody *vis = SIM_BODY_DEFAULT_VIS);
};

/**
 * Body with shape of cube.
 * Constructor takes width of edge and amount of mass.
 */
class BodyCube : public BodyBox {
  public:
    BodyCube(World *w, Scalar width, Scalar mass,
             VisBody *vis = SIM_BODY_DEFAULT_VIS);
};

/**
 * Body with sphere shape.
 * Contructor takes radius of sphere and mass.
 */
class BodySphere : public BodySimple {
  public:
    BodySphere(World *w, Scalar radius, Scalar mass,
               VisBody *vis = SIM_BODY_DEFAULT_VIS);
};

/**
 * Body with cylinder shape placed along Z axis.
 * Constructor takes radius and height of cylinder.
 */
class BodyCylinder : public BodySimple {
  public:
    BodyCylinder(World *w, Scalar radius, Scalar height, Scalar mass,
                 VisBody *vis = SIM_BODY_DEFAULT_VIS);
};

/**
 * Body with shape of cylinder placed along X axis.
 */
class BodyCylinderX : public BodyCylinder {
  public:
    BodyCylinderX(World *w, Scalar radius, Scalar height, Scalar mass,
                  VisBody *vis = SIM_BODY_DEFAULT_VIS);
};

/**
 * Body with shape of cylinder placed along Y axis.
 */
class BodyCylinderY : public BodyCylinder {
  public:
    BodyCylinderY(World *w, Scalar radius, Scalar height, Scalar mass,
                  VisBody *vis = SIM_BODY_DEFAULT_VIS);
};

/**
 * Body represented by triangular mesh. This Body is static and thus can't
 * have any mass.
 */
class BodyTriMesh : public BodySimple {
  public:
    BodyTriMesh(World *w, const sim::Vec3 *coords, size_t coords_len,
                const unsigned int *indices, size_t indices_len,
                Scalar mass, VisBody *vis = SIM_BODY_DEFAULT_VIS);
};


} /* namespace ode */

} /* namespace sim */


#endif /* _SIM_ODE_OBJ_HPP_ */
