#include "sim.hpp"
#include "arena.hpp"

Sim::Sim()
    : sim::Sim(), _arena(0)
{
    DBG("Using ODE");

    sim::ode::World *w = new sim::ode::World();

    setWorld(w);

    w->setCFM(0.0001);
    w->setERP(0.8);
    w->setStepType(sim::ode::World::STEP_TYPE_QUICK);
    w->setAutoDisable(0.01, 0.01, 5, 0.);

    w->setContactApprox1(true);
    w->setContactApprox2(true);
    w->setContactBounce(0.1, 0.1);

    setTimeStep(sim::Time::fromMs(20));
    setTimeSubSteps(2);

    //pauseSimulation();

    createArena();
    createRobots();
}

void Sim::createArena()
{
    _arena = new Arena(this);
    addComponent(_arena);
}

void Sim::createRobots()
{
    Robot *rob;

    rob = new Robot(Vec3(-7., 0., .6), Quat(0, 0, 0, 1), true);
    addComponent(rob);
    _robots.push_back(rob);
}
