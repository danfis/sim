#include "actuator.hpp"

namespace sim {

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

void ActuatorWheelCylinderX::connectToChasis(Body *b)
{
    Vec3 axis1(0., 0., 1.);
    Vec3 axis2(1., 0., 0.);

    _joint = new JointHinge2(_world, b, _wheel, _wheel->pos(), axis1, axis2);
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

}
