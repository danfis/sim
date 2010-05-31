#include <sim/sim.hpp>
#include "robot_sssa.hpp"

using sim::Vec3;

SSSAComp::SSSAComp()
    : _sim(0), _robot(0)
{
}

void SSSAComp::init(sim::Sim *sim)
{
    _sim = sim;

    _robot = new sim::robot::SSSA(_sim->world(), Vec3(0., 0., 1.2));
    _robot->activate();
}

void SSSAComp::finish()
{
}

