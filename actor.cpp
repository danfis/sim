#include "actor.hpp"
#include "msg.hpp"

namespace sim {

Robot4Wheels::Robot4Wheels(World *w)
    : Actor(w)
{
    size_t i;

    _chasis = new sim::BodyBox(_world, sim::Vec3(.6, 1., 0.4), 1.);
    for (i = 0; i < 4; i++){
        _wheels[i] = new sim::ActuatorWheelCylinderX(_world, 0.2, 0.2, 1.);
    }

    _wheels[0]->setPos(0.405, 0.4, -0.2);
    _wheels[1]->setPos(-0.405, 0.4, -0.2);
    _wheels[2]->setPos(0.405, -0.4, -0.2);
    _wheels[3]->setPos(-0.405, -0.4, -0.2);

    for (i = 0; i < 4; i++){
        _wheels[i]->connectToChasis(_chasis);
    }

    // set colors
    _chasis->visBody()->setColor(osg::Vec4(0.2, 0.2, 1.0, 1.));
    _wheels[0]->wheel()->visBody()->setColor(osg::Vec4(1., 0.8, 0.8, 1.));
    _wheels[1]->wheel()->visBody()->setColor(osg::Vec4(1., 0.8, 0.8, 1.));
    _wheels[2]->wheel()->visBody()->setColor(osg::Vec4(0.8, 1., 0.8, 1.));
    _wheels[3]->wheel()->visBody()->setColor(osg::Vec4(0.8, 1., 0.8, 1.));
}

void Robot4Wheels::setPos(const Vec3 &v)
{
    _chasis->setPos(v);
    _wheels[0]->setPos(v + Vec3(0.405, 0.4, -0.2));
    _wheels[1]->setPos(v + Vec3(-0.405, 0.4, -0.2));
    _wheels[2]->setPos(v + Vec3(0.405, -0.4, -0.2));
    _wheels[3]->setPos(v + Vec3(-0.405, -0.4, -0.2));
}


void Robot4Wheels::activate()
{
    size_t i;

    _chasis->activate();
    for (i = 0; i < 4; i++){
        _wheels[i]->activate();
    }
}

void Robot4Wheels::deactivate()
{
    /* TODO */
}

}
