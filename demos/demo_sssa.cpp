#include <iostream>
#include <sim/sim.hpp>
#include <sim/ode/world.hpp>
#include <sim/msg.hpp>
#include <sim/sensor/camera.hpp>

#include "robot_sssa.hpp"

using namespace sim::ode;
using sim::Vec3;
using sim::Time;
using namespace std;

class S : public sim::Sim {
  public:
    S()
        : Sim()
    {
        World *w = new World();

        setTimeStep(Time::fromMs(20));
        setTimeSubSteps(2);

        setWorld(w);

        w->setCFM(0.0001);
        w->setERP(0.8);
        //w->setStepType(World::STEP_TYPE_QUICK);
        w->setAutoDisable(0.01, 0.01, 5, 0.);

        w->setContactApprox1(true);
        w->setContactApprox2(true);
        w->setContactBounce(0.1, 0.1);

        createArena();
        createRobot();



    }

    void init()
    {
        sim::Sim::init();

        visWorld()->step();
        sleep(1);
    }

    void createArena()
    {
        osg::Vec4 color(0., 0.7, 0.1, 1.);
        BodyCompound *c;
        int id;
        sim::ode::World *w = (sim::ode::World *)world();

        c = (BodyCompound *)w->createBodyCompound();
        id = c->addBox(Vec3(10., 10., 0.1));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");
        id = c->addBox(Vec3(0.1, 10., 0.5), SIM_BODY_DEFAULT_VIS, Vec3(-5, 0., .25));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");
        id = c->addBox(Vec3(0.1, 10., 0.5), SIM_BODY_DEFAULT_VIS, Vec3(5, 0., .25));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");
        id = c->addBox(Vec3(10., .1, 0.3), SIM_BODY_DEFAULT_VIS, Vec3(0., 5, .25));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");
        id = c->addBox(Vec3(10., .1, 0.3), SIM_BODY_DEFAULT_VIS, Vec3(0., -5, .25));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");

        id = c->addCube(.2, SIM_BODY_DEFAULT_VIS, Vec3(0.5, 0.5, .1));
        c->visBody(id)->setColor(0., .1, .8, 1.);
        c->visBody(id)->setTexture("wood.ppm");
        c->activate();
    }

    void createRobot()
    {
        SSSAComp *comp;
        comp = new SSSAComp(Vec3(0., 0, 1.5));
        addComponent(comp);

        comp = new SSSAComp(Vec3(1.2, 1.2, 1.5));
        addComponent(comp);
        comp = new SSSAComp(Vec3(1.2, 2.3, 1.5));
        addComponent(comp);
    }

};

int main(int argc, char *argv[])
{
    S s;

    s.run();

    return 0;
}