#include <sim/sim.hpp>
#include <sim/msg.hpp>
#include "robot_sssa.hpp"

using sim::Vec3;
using sim::Quat;

SSSAComp::SSSAComp(const sim::Vec3 &pos)
    : _sim(0), _robot(0), _pos(pos)
{
}

void SSSAComp::init(sim::Sim *sim)
{
    _sim = sim;

    _robot = new sim::robot::SSSA(_sim->world(), _pos);//, Quat(Vec3(0., 0., 1.), M_PI / 2.));
    _robot->activate();
    DBG(this << " " << DBGV(_pos));

    _sim->regPostStep(this);

    _sim->regMessage(this, sim::MessageKeyPressed::Type);
}

void SSSAComp::finish()
{
}


void SSSAComp::cbPostStep()
{
    /*
    Vec3 pos, dir, up;
    size_t i;

    for (i = 0; i < 3; i++){
        _robot->socketSetup(i, &pos, &dir, &up);
        DBG(this << " socket " << i << ": " << DBGV(pos) << " " <<
                DBGV(dir) << " " << DBGV(up));
    }
    _robot->ballSetup(&pos, &dir, &up);
    DBG(this << " ball: " << DBGV(pos) << " " << DBGV(dir) << " " <<
            DBGV(up));
    DBG(this << " arm angle: " << _robot->armAngle());
    */
}

void SSSAComp::processMessage(const sim::Message &msg)
{
    if (msg.type() == sim::MessageKeyPressed::Type){
        _keyPressedMsg((const sim::MessageKeyPressed &)msg);
    }
}

void SSSAComp::_keyPressedMsg(const sim::MessageKeyPressed &msg)
{
    int key = msg.key();

    //DBG("Component: " << this << " - key pressed: " << msg.key());

    if (key == 'h'){
        _robot->addVelLeft(0.1);
    }else if (key == 'j'){
        _robot->addVelLeft(-0.1);
    }else if (key == 'k'){
        _robot->addVelRight(0.1);
    }else if (key == 'l'){
        _robot->addVelRight(-0.1);

    }else if (key == 'n'){
        _robot->addVelArm(0.1);
    }else if (key == 'm'){
        _robot->addVelArm(-0.1);
    }else if (key == ','){
        _robot->fixArm();
    }else if (key == '.'){
        _robot->unfixArm();
    }else if (key == 'v'){
        _robot->reachArmAngle(M_PI / 4.);
    }else if (key == 'b'){
        _robot->reachArmAngle(-M_PI / 4.);
    }
    DBG("Velocity: " << _robot->velLeft() << " " << _robot->velRight() << " " << _robot->velArm());
}
