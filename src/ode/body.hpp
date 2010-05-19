#ifndef _SIM_ODE_OBJ_HPP_
#define _SIM_ODE_OBJ_HPP_

#include <ode/ode.h>

#include "sim/body.hpp"
#include "sim/visbody.hpp"

namespace sim {

namespace ode {

// Forward declaration
class World;

/**
 * Class representing physical body.
 */
class Body : public sim::Body {
  protected:
    World *_world;
    dBodyID _body;
    dGeomID _shape;

  public:
    Body(World *w);
    virtual ~Body();

    /* \{ */
    dBodyID body() { return _body; }
    const dBodyID body() const { return _body; }
    dGeomID shape() { return _shape; }
    const dGeomID shape() const { return _shape; }
    World *world() { return _world; }
    const World *world() const { return _world; }
    /* \} */

    /* \{ */
    /**
     * Activates body in world - world will realize this body.
     */
    virtual void activate();

    /**
     * Deactives body - body will be removed from a world.
     */
    virtual void deactivate();
    /* \} */
  protected:
    virtual void _applyPosRot();
    virtual void _enableShape();
    virtual void _enableBody();
    virtual void _enableVisBody();
    virtual void _disableShape();
    virtual void _disableBody();
    virtual void _disableVisBody();

    void _set(VisBody *vis, dGeomID shape, dMass *mass);
    void _setOnlyShape(VisBody *vis, dGeomID shape);
};


/**
 * Body with shape of cube.
 * Constructor takes width of edge and amount of mass.
 */
class BodyCube : public Body {
  public:
    BodyCube(World *w, Scalar width, Scalar mass,
             VisBody *vis = SIM_BODY_DEFAULT_VIS);
};

/**
 * Body with general box shape.
 * Constructot takes vector with lengths of three box's edges and amount of
 * mass.
 */
class BodyBox : public Body {
  public:
    BodyBox(World *w, const Vec3 &dim, Scalar mass,
            VisBody *vis = SIM_BODY_DEFAULT_VIS);
};

/**
 * Body with sphere shape.
 * Contructor takes radius of sphere and mass.
 */
class BodySphere : public Body {
  public:
    BodySphere(World *w, Scalar radius, Scalar mass,
               VisBody *vis = SIM_BODY_DEFAULT_VIS);
};

/**
 * Body with cylinder shape placed along Z axis.
 * Constructor takes radius and height of cylinder.
 */
class BodyCylinder : public Body {
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
class BodyTriMesh : public Body {
  public:
    BodyTriMesh(World *w, const sim::Vec3 *coords, size_t coords_len,
                const unsigned int *indices, size_t indices_len,
                Scalar mass, VisBody *vis = SIM_BODY_DEFAULT_VIS);
};

} /* namespace ode */

} /* namespace sim */

#endif /* _SIM_ODE_OBJ_HPP_ */
