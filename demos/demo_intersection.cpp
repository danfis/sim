#include <iostream>
#include <sim/sim.hpp>
#include <sim/ode/world.hpp>
#include <sim/msg.hpp>
#include <sim/sensor/camera.hpp>
#include <sim/sensor/rangefinder.hpp>

using namespace sim::ode;
using sim::Scalar;
using sim::Vec3;
using sim::Quat;
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
        createIntersectors();
    }

    void createArena()
    {
        osg::Vec4 color(0., 0.7, 0.1, 1.);
        BodyCompound *c;
        int id;
        sim::ode::World *w = (sim::ode::World *)world();

        c = (BodyCompound *)w->createBodyCompound();
        id = c->addBox(Vec3(2., 2., 0.1));
        id = c->addBox(Vec3(0.1, 2., 0.5), SIM_BODY_DEFAULT_VIS, Vec3(-1., 0., .25));
        id = c->addBox(Vec3(2., .1, 0.3), SIM_BODY_DEFAULT_VIS, Vec3(0., 1., .25));
        id = c->addBox(Vec3(2., .1, 0.3), SIM_BODY_DEFAULT_VIS, Vec3(0., -1., .25));

        id = c->addBox(Vec3(3., 1., 0.1), SIM_BODY_DEFAULT_VIS, Vec3(2.5, 0., 0.));

        id = c->addBox(Vec3(2., 2., 0.1), SIM_BODY_DEFAULT_VIS, Vec3(5., 0., 0.));
        id = c->addBox(Vec3(2., .1, 0.3), SIM_BODY_DEFAULT_VIS, Vec3(5., 1., .25));
        id = c->addBox(Vec3(2., .1, 0.3), SIM_BODY_DEFAULT_VIS, Vec3(5., -1., .25));
        id = c->addBox(Vec3(0.1, 2., 0.5), SIM_BODY_DEFAULT_VIS, Vec3(6., 0., .25));
        {
            std::list<sim::VisBody *> list;
            std::list<sim::VisBody *>::iterator it, it_end;
            c->visBodyAll(&list);

            it = list.begin();
            it_end = list.end();
            for (; it != it_end; ++it){
                (*it)->setColor(color);
                (*it)->setTexture("wood.ppm");
            }
        }


        id = c->addCube(.2, SIM_BODY_DEFAULT_VIS, Vec3(0.5, 0.5, .1));
        c->visBody(id)->setColor(0., .1, .8, 1.);
        c->visBody(id)->setTexture("wood.ppm");
        c->activate();

        id = c->addCube(.2, SIM_BODY_DEFAULT_VIS, Vec3(5., 0., .1));
        c->visBody(id)->setColor(0., .1, .8, 1.);
        c->visBody(id)->setTexture("wood.ppm");
        c->activate();

        id = c->addCube(.1, SIM_BODY_DEFAULT_VIS, Vec3(1., 0., .1));
        c->visBody(id)->setColor(0., .1, .8, 1.);
        c->visBody(id)->setTexture("wood.ppm");
        c->activate();

        id = c->addBox(Vec3(.1, .1, 1.), SIM_BODY_DEFAULT_VIS, Vec3(2.5, 0., .5));
        c->visBody(id)->setColor(0., .1, .8, 1.);
        c->visBody(id)->setTexture("wood.ppm");
        c->activate();

        id = c->addCube(.1, SIM_BODY_DEFAULT_VIS, Vec3(3.5, 0., .1));
        c->visBody(id)->setColor(0., .1, .8, 1.);
        c->visBody(id)->setTexture("wood.ppm");
        c->activate();

        id = c->addBox(Vec3(0.5, .1, .2), SIM_BODY_DEFAULT_VIS,
                       Vec3(2., 0., .1), Quat(Vec3(0., 0., 1.), M_PI / 4.));
        c->visBody(id)->setColor(0., .1, .8, 1.);
        c->visBody(id)->setTexture("wood.ppm");
        c->activate();

        {
            sim::Body *b;

            b = w->createBodyCube(0.1, 0.1);
            b->setPos(2., 0., 1.);
            b->activate();

            b = w->createBodySphere(0.1, 0.1);
            b->setPos(0.35, 0.4, .5);
            b->activate();
        }
    }

    void createIntersectors()
    {
        sim::sensor::RangeFinder *rf;

        //rf = new sim::sensor::RangeFinder(80., 181, M_PI);
        rf = new sim::sensor::RangeFinder(1., 181, M_PI);
        //rf->setPosRot(Vec3(0., 0., 0.2), Quat(Vec3(1., 0., 0.), M_PI / 4.));
        rf->setPosRot(Vec3(0., 0., 0.2));
        rf->enableVis();
        //rf->setPosRot(Vec3(0., 0., 0.2), Quat(Vec3(0., 0., 1.), M_PI / 4.));
        //rf->setPosRot(Vec3(0., 0., 0.2), Quat(Vec3(0., 0., 1.), -M_PI / 4.));
        //rf->setPosRot(Vec3(0., 0., 0.2), Quat(Vec3(1., 0., 0.), M_PI / 4.));
        addComponent(rf);
    }
};

int main(int argc, char *argv[])
{
    S s;

    s.run();

    return 0;
}
