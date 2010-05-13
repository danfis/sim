#ifndef _SIM_ACTUATOR_HPP_
#define _SIM_ACTUATOR_HPP_

#include "sim/body.hpp"

namespace sim {

class ActuatorWheelCylinderX {
  public:
    /**
     * Connects wheel to chasis.
     * You should set up wheel before it is connected to chasis.
     */
    virtual void connectToChasis(Body *b) = 0;

    virtual void activate() = 0;
    virtual void deactivate() = 0;

    /**
     * Sets position of wheel.
     */
    virtual void setPos(const Vec3 &v) = 0;
    virtual void setPos(const Vec3 *v) { setPos(*v); }
    virtual void setPos(const Scalar x, const Scalar y, const Scalar z)
        { setPos(Vec3(x, y, z)); }

    /**
     * Returns position of wheel.
     */
    virtual Vec3 pos() const = 0;
    virtual void pos(Scalar *x, Scalar *y, Scalar *z) const = 0;

    /**
     * Applies torque on wheel.
     */
    virtual void applyTorque(const Vec3 &v) = 0;
    virtual void applyTorqueImpulse(const Vec3 &v) = 0;
};

} /* namespace sim */

#endif /* _SIM_ACTUATOR_HPP_ */
