#include <unistd.h>
#include <iostream>

#include "sim/ode/world.hpp"
#include "sim/sim.hpp"
#include "msg.hpp"

using namespace sim::ode;
using sim::Vec3;

class S : public sim::Sim {
  public:
    S()
        : Sim()
    {
        setTimeStep(sim::Time::fromMs(50));
        setTimeSubSteps(1);

        World *w = new World();
        sim::Body *b;

        setWorld(w);
        w->setCFM(1e-10);
        //w->setERP(0.5);
        //w->setContactSoftCFM(0.01);
        //w->setContactApprox1(false);
        //w->setContactApprox2(false);
        //w->setContactBounce(0.1, 0.1);
    
        /*
        b = w->createBodyBox(sim::Vec3(20., 20., 1.), 0.);
        b->setPos(0., 0., -10.);
        b->activate();
        */

        b = w->createBodyCube(1., 1.);
        b->visBody()->setColor(osg::Vec4(0.5, 0.5, 0.5, 0.5));
        b->setPos(-6.5, 6.5, -5.);
        b->activate();

        b = w->createBodySphere(1.3, 2.);
        b->visBody()->setColor(osg::Vec4(0.5, 0.1, 0.8, 0.3));
        b->setPos(-3., 0., 0.);
        b->activate();


        b = w->createBodyCube(1., 1.);
        b->visBody()->setColor(osg::Vec4(0., 0., 0., 1.));
        b->setPos(-3., 0.2, 3.);
        b->activate();

        b = w->createBodyCylinderZ(.5, 1., 3.);
        b->setPos(3., 0., 0.);
        b->activate();

        b = w->createBodyCylinderX(.5, 1., 3.);
        b->setPos(0., 3., 0.);
        b->activate();

        b = w->createBodyCylinderY(.5, 1., 3.);
        b->setPos(0., -3., 0.);
        b->activate();

        /*
        createFixed();
        createHinge();
        createRobot();
        */

        Vec3 verts[] = { Vec3(-10., 10., -0.5),
                         Vec3(0., 10., -0.8),
                         Vec3(10., 10., -0.5),
                         Vec3(-10., 0., 0.5),
                         Vec3(0., 0., 0.),
                         Vec3(10., 0., 0.7),
                         Vec3(-10., -10., 0.4),
                         Vec3(0., -10., 1.),
                         Vec3(10., -10., 0.8) };
        unsigned int ind[] = { 0, 4, 1,
                               1, 5, 2,
                               0, 3, 4,
                               1, 4, 5,
                               3, 7, 4,
                               4, 8, 5,
                               3, 6, 7,
                               4, 7, 8 };
        b = w->createBodyTriMesh(verts, 9, ind, 24, 0.);
        b->visBody()->setColor(0.9, 0.6, 0.3, 1.);
        b->setPos(0., 0., -5.3);
        b->activate();
        /*
        b = w->createBodyBox(Vec3(20, 20, 2), 0.);
        b->setPos(0., 0., -10);
        b->activate();
        */
    }

    void init()
    {
        sim::Sim::init();
        for (size_t i = 0; i < 300; i++){
            visWorld()->step();
            std::cerr << i << "\r";
            usleep(10000);
        }
        std::cerr << std::endl;

        timeRealRestart();
    }

  protected:
    /*
    void createFixed()
    {
        sim::Vec3 pos(-6., -6., -9.);
        sim::Body *cu = world()->createBodyCube(1., 1.);
        sim::Body *s = world()->createBodySphere(0.5, 3.);
        sim::Joint *j;

        cu->setPos(pos);
        s->setPos(pos + sim::Vec3(0.9, 0., 1.0));
        j = world()->createJointFixed(cu, s);

        cu->activate();
        s->activate();
        j->activate();

        cu->visBody()->setText("Fixed", 1., osg::Vec4(0.9, 0.6, 0.3, 1.));
    }

    void createHinge()
    {
        sim::Vec3 pos(-6., 6., -9.);
        sim::Body *cy = world()->createBodyCylinderZ(0.5, 0.5, 1.);
        sim::Body *cu = world()->createBodyCube(1., 1.);
        sim::Joint *j;

        cu->setPos(pos);
        cy->setPos(pos + sim::Vec3(0., 0., 2.));

        j = world()->createJointHinge(cu, cy, pos + sim::Vec3(0., 0., 1.), sim::Vec3(0., 1., 0.));

        cu->activate();
        cy->activate();
        j->activate();

        cu->visBody()->setText("Hinge", 1., osg::Vec4(0.7, 0.6, 0.3, 1.));
    }

    void createRobot()
    {
        sim::Vec3 pos(3., 3., -6.);
        sim::Body *chasis;
        sim::ActuatorWheelCylinderX *w[4];
        size_t i;

        chasis = world()->createBodyBox(sim::Vec3(.6, 1., 0.4), 1.);
        for (i = 0; i < 4; i++){
            w[i] = world()->createActuatorWheelCylinderX(0.2, 0.2, 1.);
        }

        chasis->setPos(pos);
        w[0]->setPos(pos.x() + 0.405, pos.y() + 0.4, pos.z() - 0.2);
        w[1]->setPos(pos.x() - 0.405, pos.y() + 0.4, pos.z() - 0.2);
        w[2]->setPos(pos.x() + 0.405, pos.y() - 0.4, pos.z() - 0.2);
        w[3]->setPos(pos.x() - 0.405, pos.y() - 0.4, pos.z() - 0.2);

        for (i = 0; i < 4; i++){
            w[i]->connectToChasis(chasis);
        }

        chasis->activate();
        for (i = 0; i < 4; i++){
            w[i]->activate();
        }

        chasis->visBody()->setText("Robot", 1., osg::Vec4(0.5, 0.6, 0.3, 1.));
    }
    */
};


int main(int argc, char *argv[])
{
    S s;

    s.run();

    return 0;
}

