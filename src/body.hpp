#ifndef _SIM_OBJ_HPP_
#define _SIM_OBJ_HPP_

#include "visbody.hpp"

#define SIM_BODY_DEFAULT_VIS ((sim::VisBody *)(0x1))

namespace sim {


/**
 * Struct with collision info.
 * This struct is used internally by collicion detector and should be set
 * carefully.
 *
 * This will be probably changed in future - for now it is fastest solution
 * how to solve some collision detection problems.
 */
struct BodyCollisionInfo {
    /**
     * If two bodies have same value of .dont_collide_id and this value
     * isn't zero collision is not performed.
     */
    unsigned long dont_collide_id;

    BodyCollisionInfo() : dont_collide_id(0) {}
};

/**
 * Class representing physical body.
 */
class Body {
  protected:
    BodyCollisionInfo _collision_info;

    VisBody *_vis;

  public:
    Body();
    virtual ~Body();

    /* \{ */
    /**
     * Returns position of body.
     */
    virtual Vec3 pos() const = 0;
    virtual void pos(Scalar *x, Scalar *y, Scalar *z) const = 0;

    /**
     * Returns rotation of body.
     */
    virtual Quat rot() const = 0;
    virtual void rot(Scalar *x, Scalar *y, Scalar *z, Scalar *w) const = 0;

    /**
     * Set position of body in 3D space.
     */
    virtual void setPos(const Vec3 &v) = 0;
    virtual void setPos(const Vec3 *v) { setPos(*v); }
    virtual void setPos(const Scalar x, const Scalar y, const Scalar z)
        { setPos(Vec3(x, y, z)); }

    /**
     * Set rotation of body in 3D space.
     */
    virtual void setRot(const Quat &q) = 0;
    virtual void setRot(const Quat *q) { setRot(*q); }
    virtual void setRot(const Scalar x, const Scalar y, const Scalar z, const Scalar w)
        { setRot(Quat(x, y, z, w)); }

    /**
     * Set position and rotation of body at once.
     */
    virtual void setPosRot(const Vec3 &v, const Quat &q) = 0;
    virtual void setPosRot(const Vec3 *v, const Quat *q) { setPosRot(*v, *q); }
    /* \} */

    /* \{ */
    virtual VisBody *visBody() { return _vis; }
    virtual const VisBody *visBody() const { return _vis; }
    virtual void setVisBody(VisBody *vis) { _vis = vis; }
    /* \} */

    /* \{ */
    /**
     * Activates body in world - world will realize this body.
     */
    virtual void activate() = 0;

    /**
     * Deactives body - body will be removed from a world.
     */
    virtual void deactivate() = 0;


    const BodyCollisionInfo &collInfo() { return _collision_info; }
    void collSetDontCollideId(unsigned long id)
        { _collision_info.dont_collide_id = id; }
    /* \} */
};

}

#endif /* _SIM_OBJ_HPP_ */
