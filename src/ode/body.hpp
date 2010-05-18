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
     * Set position of body in 3D space.
     */
    virtual void setPos(const Vec3 &v);

    /**
     * Set rotation of body in 3D space.
     */
    virtual void setRot(const Quat &q);

    /**
     * Set position and rotation of body at once.
     */
    virtual void setPosRot(const Vec3 &v, const Quat &q);

    /**
     * Returns position of body.
     */
    virtual Vec3 pos() const;
    virtual void pos(Scalar *x, Scalar *y, Scalar *z) const;

    /**
     * Returns rotation of body.
     */
    virtual Quat rot() const;
    virtual void rot(Scalar *x, Scalar *y, Scalar *z, Scalar *w) const;
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
    BodyBox(World *w, Vec3 dim, Scalar mass,
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
