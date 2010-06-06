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

#ifndef _SIM_BULLET_ACTUATOR_HPP_
#define _SIM_BULLET_ACTUATOR_HPP_

#include "sim/actuator.hpp"
#include "sim/bullet/body.hpp"
#include "sim/bullet/joint.hpp"

namespace sim {

namespace bullet {

// Forward declaration
class World;


class ActuatorWheelCylinderX : public sim::ActuatorWheelCylinderX {
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

    /**
     * Connects wheel to chasis.
     * You should set up wheel before it is connected to chasis.
     */
    void connectToChasis(sim::Body *b);

    void activate();
    void deactivate();

    /**
     * Sets position of wheel.
     */
    void setPos(const Vec3 &v) { _wheel->setPos(v); }
    void setPos(const Vec3 *v) { setPos(*v); }
    void setPos(const Scalar x, const Scalar y, const Scalar z)
        { setPos(Vec3(x, y, z)); }

    /**
     * Returns position of wheel.
     */
    Vec3 pos() const { return _wheel->pos(); }
    void pos(Scalar *x, Scalar *y, Scalar *z) const { _wheel->pos(x, y, z); }

    /**
     * Applies torque on wheel.
     */
    void applyTorque(const Vec3 &v);
    void applyTorqueImpulse(const Vec3 &v);
};

} /* namespace bullet */

} /* namespace sim */

#endif /* _SIM_BULLET_ACTUATOR_HPP_ */
