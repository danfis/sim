#include "sim/ode/actuator.hpp"
#include "sim/ode/math.hpp"

namespace sim {

namespace ode {

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
}

void ActuatorWheelCylinderX::activate()
{
    if (_wheel)
        _wheel->activate();
    if (_joint)
        _joint->activate();
}

void ActuatorWheelCylinderX::deactivate()
{
    if (_wheel)
        _wheel->deactivate();
    if (_joint)
        _joint->deactivate();
}

void ActuatorWheelCylinderX::applyTorque(const Vec3 &v)
{
    //_wheel->body()->applyTorque(vToBt(v));
}

void ActuatorWheelCylinderX::applyTorqueImpulse(const Vec3 &v)
{
    //_wheel->body()->applyTorqueImpulse(vToBt(v));
}


} /* namespace ode */

} /* namespace sim */
