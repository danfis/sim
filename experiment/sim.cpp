#include "sim.hpp"
#include "arena.hpp"

#define ODE 1

Sim::Sim()
    : sim::Sim(), _arena(0)
{
#if ODE == 1
    DBG("Using ODE");
    sim::ode::World *w = new sim::ode::World();

    setWorld(w);

    w->setCFM(.0001);
    w->setERP(0.8);
    w->setStepType(sim::ode::World::STEP_TYPE_QUICK);
    w->setAutoDisable(0.01, 0.01, 5, 0.);

    w->setContactApprox1(true);
    w->setContactApprox2(true);
    w->setContactBounce(0.1, 0.1);
#else
    DBG("Using Bullet");
    sim::bullet::World *w = new sim::bullet::World();
    setWorld(w);
#endif

    setTimeStep(sim::Time::fromMs(5));
    setTimeSubSteps(2);

    pauseSimulation();

    _povray = 0;

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

    /*
    rob = new Robot(Vec3(-7., 0., .6), Quat(0, 0, 0, 1), true);
    addComponent(rob);
    _robots.push_back(rob);

    rob = new Robot(Vec3(-5.746, 0., .6));
    addComponent(rob);
    _robots.push_back(rob);

    rob = new Robot(Vec3(-4.492, 0., .6));
    addComponent(rob);
    _robots.push_back(rob);

    rob = new Robot(Vec3(-3.238, 0., .6));
    addComponent(rob);
    _robots.push_back(rob);

    rob = new Robot(Vec3(-1.984, 0., .6));
    addComponent(rob);
    _robots.push_back(rob);
    */

    //rob = new Robot(Vec3(-7., 0., .6), Quat(0, 0, 0, 1), true);
    //rob = new Robot(Vec3(-7., 0., .6), Quat(Vec3(0, 0, 1), M_PI_4 / 4.), true);
    //rob = new Robot(Vec3(-8., 0., .6), Quat(0, 0, 0, 1), true);
    rob = new Robot(Vec3(8., 10., .6), Quat(Vec3(0, 0, -1), M_PI_2 + M_PI_4 / 4.), true);
    addComponent(rob);
    _robots.push_back(rob);
}

void Sim::init()
{
    sim::Sim::init();
    connectRobots();

    //_povray = new sim::comp::PovrayFull("povray/");
    //addComponent(_povray);
}

void Sim::connectRobots()
{
    std::list<Robot *>::iterator it, it2, it_end;

    it = it2 = _robots.begin();
    it_end = _robots.end();
    for (; it != it_end; ++it){
        it2 = it;
        for (++it2; it2 != it_end; ++it2){
            if (!(*it)->robot() || !(*it2)->robot())
                continue;

            if (!(*it)->robot()->connectTo(*(*it2)->robot())){
                (*it2)->robot()->connectTo(*(*it)->robot());
            }
        }
    }
}

void Sim::waitForRobot(Robot *r)
{
    Robot *rob;
    Vec3 pos;
    Quat rot;
    Vec3 tr(1, 0, 0);

    if (r->waitForRobot()){
        if (r->robot()->isSocketConnected(1))
            return;

        pos = r->robot()->pos();
        rot = r->robot()->rot();
        tr = tr * 1.254;
        tr = rot * tr;
        pos = pos + tr;

        //DBG("pos: " << pos.x() << " " << pos.y() << " " << pos.z());
        //DBG("rot: " << rot.x() << " " << rot.y() << " " << rot.z() << " " << rot.w());

        rob = new Robot(pos, rot, false);
        addComponent(rob);
        _robots.push_back(rob);
    }
}
