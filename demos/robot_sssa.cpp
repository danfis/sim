#include <sim/sim.hpp>
#include <sim/msg.hpp>
#include "robot_sssa.hpp"

using sim::Vec3;

SSSAComp::SSSAComp(const sim::Vec3 &pos)
    : _sim(0), _robot(0), _pos(pos)
{
}

void SSSAComp::init(sim::Sim *sim)
{
    _sim = sim;

    _robot = new sim::robot::SSSA(_sim->world(), _pos);
    _robot->activate();

    _sim->regPostStep(this);

    DBG(this << " " << DBGV(_pos));
}

void SSSAComp::finish()
{
}


void SSSAComp::cbPostStep()
{
    Vec3 pos, dir;
    size_t i;

    for (i = 0; i < 3; i++){
        _robot->socketPosDir(i, &pos, &dir);
        DBG(this << " socket " << i << ": " << DBGV(pos) << " " << DBGV(dir));
    }
    _robot->ballPosDir(&pos, &dir);
    DBG(this << " ball: " << DBGV(pos) << " " << DBGV(dir));
}
