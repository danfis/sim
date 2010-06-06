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

#include "sim/bullet/actuator.hpp"
#include "sim/bullet/math.hpp"

namespace sim {

namespace bullet {

ActuatorWheelCylinderX::ActuatorWheelCylinderX(World *w,
                        Scalar radius, Scalar height, Scalar mass)
    : _world(w)
{
    _wheel = new BodyCylinderX(_world, radius, height, mass);
    _joint = 0;
}

ActuatorWheelCylinderX::~ActuatorWheelCylinderX()
{
    if (_wheel)
        delete _wheel;
    if (_joint)
        delete _joint;
}

void ActuatorWheelCylinderX::connectToChasis(sim::Body *_b)
{
    Body *b = (Body *)_b;

    Vec3 axis1(0., 0., 1.);
    Vec3 axis2(1., 0., 0.);

    _joint = new JointHinge2(_world, b, _wheel, _wheel->pos(), axis1, axis2);
    /*
    _joint->setLimitLinAxis1(-0.01, 0.01);
    _joint->setLimitLinAxis2(0., 0.);
    _joint->setLimitAngAxis1(-0.05, 0.05);
    _joint->setLimitAngAxis2(-10., 10.);
    */
}

void ActuatorWheelCylinderX::activate()
{
    if (_wheel && _joint){
        _wheel->activate();
        _joint->activate();
    }
    // TODO: raise error otherwise?
}

void ActuatorWheelCylinderX::deactivate()
{
    /* TODO: */
}

void ActuatorWheelCylinderX::applyTorque(const Vec3 &v)
{
    _wheel->body()->applyTorque(vToBt(v));
}

void ActuatorWheelCylinderX::applyTorqueImpulse(const Vec3 &v)
{
    _wheel->body()->applyTorqueImpulse(vToBt(v));
}


} /* namespace bullet */

} /* namespace sim */
