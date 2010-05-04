#ifndef _SIM_ACTOR_HPP_
#define _SIM_ACTOR_HPP_

#include <list>

#include "body.hpp"
#include "joint.hpp"
#include "actuator.hpp"

namespace sim {

class World;

class Actor {
  protected:
    World *_world;

  public:
    Actor(World *w) : _world(w) {}
    virtual ~Actor() {}

    World *world() { return _world; }
    const World *world() const { return _world; }

    virtual void activate() = 0;
    virtual void deactivate() = 0;
};


class Robot4Wheels : public Actor {
  protected:
    BodyBox *_chasis;
    ActuatorWheelCylinderX *_wheels[4];

  public:
    Robot4Wheels(World *w);

    void setPos(const Scalar x, const Scalar y, const Scalar z)
        { setPos(Vec3(x, y, z)); }
    void setPos(const Vec3 *v) { setPos(*v); }
    void setPos(const Vec3 &v);

    Vec3 pos() const { return _chasis->pos(); }
    void pos(Scalar *x, Scalar *y, Scalar *z) const { _chasis->pos(x, y, z); }

    void activate();
    void deactivate();
};

} /* namespace sim */

#endif /* _SIM_ACTOR_HPP_ */
