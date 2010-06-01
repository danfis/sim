#include <iostream>
#include <sim/sim.hpp>
#include <sim/ode/world.hpp>
#include <sim/msg.hpp>
#include <sim/comp/syrotek.hpp>
#include "sim/comp/povray.hpp"

using namespace sim::ode;
using sim::Vec3;
using sim::Time;
using namespace std;

static bool use_cam = false;

class Syrotek : public sim::comp::Syrotek {
  public:
    Syrotek(const sim::Vec3 &pos)
        : sim::comp::Syrotek(pos)
    {
    }

    void init(sim::Sim *sim)
    {
        sim::comp::Syrotek::init(sim);
        _useKeyboard();

        if (use_cam){
            _useCamera(100, 100);
            cam()->enableView();
        }
    }
};

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

		sim::comp::Povray *pc = new sim::comp::Povray("povray/");
		addComponent(pc);
		regPostStep(pc);


    }

    void createArena()
    {
        osg::Vec4 color(0., 0.7, 0.1, 1.);
        BodyCompound *c;
        int id;
        sim::ode::World *w = (sim::ode::World *)world();

        c = (BodyCompound *)w->createBodyCompound();
        id = c->addBox(Vec3(2., 2., 0.1));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");
        id = c->addBox(Vec3(0.1, 2., 0.5), SIM_BODY_DEFAULT_VIS, Vec3(-1., 0., .25));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");
        id = c->addBox(Vec3(0.1, 2., 0.5), SIM_BODY_DEFAULT_VIS, Vec3(1., 0., .25));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");
        id = c->addBox(Vec3(2., .1, 0.3), SIM_BODY_DEFAULT_VIS, Vec3(0., 1., .25));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");
        id = c->addBox(Vec3(2., .1, 0.3), SIM_BODY_DEFAULT_VIS, Vec3(0., -1., .25));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");

        id = c->addCube(.2, SIM_BODY_DEFAULT_VIS, Vec3(0.5, 0.5, .1));
        c->visBody(id)->setColor(0., .1, .8, 1.);
        c->visBody(id)->setTexture("wood.ppm");
        c->activate();

        {
            sim::Body *b;

            b = w->createBodyCube(0.1, 0.1);
            b->setPos(0., 0., 1.);
            b->activate();

            b = w->createBodySphere(0.1, 0.1);
            b->setPos(0.15, 0.4, .4);
            b->activate();
        }
    }

    void createRobot()
    {
        Syrotek *comp = new Syrotek(Vec3(0., 0., 0.08));
        addComponent(comp);
    }

};

int main(int argc, char *argv[])
{
    if (argc > 1){
        if (strcmp(argv[1], "--cam") == 0){
            use_cam = true;
        }
    }

    S s;
    s.run();

    return 0;
}
