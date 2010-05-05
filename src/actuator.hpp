#ifndef _SIM_ACTUATOR_HPP_
#define _SIM_ACTUATOR_HPP_

#include "body.hpp"
#include "joint.hpp"

namespace sim {

class World;

class ActuatorWheelCylinderX {
  protected:
    World *_world;
    BodyCylinderX *_wheel;
    JointHinge2 *_joint;

  public:
    ActuatorWheelCylinderX(World *w,
                          Scalar radius, Scalar height, Scalar mass);
    virtual ~ActuatorWheelCylinderX();

    BodyCylinderX *wheel() { return _wheel; }
    const BodyCylinderX *wheel() const { return _wheel; }
    JointHinge2 *joint() { return _joint; }
    const JointHinge2 *joint() const { return _joint; }

    void connectToChasis(Body *b);

    void activate();
    void deactivate();

    void setPos(const Scalar x, const Scalar y, const Scalar z)
        { setPos(Vec3(x, y, z)); }
    void setPos(const Vec3 *v) { setPos(*v); }
    void setPos(const Vec3 &v) { _wheel->setPos(v); }

    Vec3 pos() const { return _wheel->pos(); }
    void pos(Scalar *x, Scalar *y, Scalar *z) const { _wheel->pos(x, y, z); }

    void applyTorque(const Vec3 &v);
};

} /* namespace sim */

#endif /* _SIM_ACTUATOR_HPP_ */
