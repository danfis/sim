#ifndef _SIM_ACTOR_HPP_
#define _SIM_ACTOR_HPP_

#include <list>

#include "body.hpp"
#include "joint.hpp"


namespace sim {

class Actor {
  protected:
    std::list<Body *> _bodies;
    std::list<Joint *> _joints;

  public:
    Actor() {}
    virtual ~Actor();

    std::list<Body *> &bodies() { return _bodies; }
    const std::list<Body *> &bodies() const { return _bodies; }
    std::list<Joint *> &joints() { return _joints; }
    const std::list<Joint *> &joints() const { return _joints; }

    void setPos(const Scalar x, const Scalar y, const Scalar z)
        { setPos(Vec3(x, y, z)); }
    void setPos(const Vec3 *v) { setPos(*v); }
    void setPos(const Vec3 &v);

  protected:
    void _addBody(Body *b);
    void _addJoint(Joint *b);
};


class Robot1 : public Actor {
  public:
    Robot1();
};

} /* namespace sim */

#endif /* _SIM_ACTOR_HPP_ */
