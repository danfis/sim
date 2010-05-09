#include <unistd.h>
#include <iostream>

#include "sim/bullet/world.hpp"
#include "sim/sim.hpp"
#include "msg.hpp"

using namespace sim::bullet;

class S : public sim::Sim {
  public:
    S()
        : Sim()
    {
        World *world = new World();
        sim::VisWorld *visworld = new sim::VisWorld();
        sim::Body *b;

        setWorld(world);
        setVisWorld(visworld);
    
        b = world->createBodyBox(sim::Vec3(20., 20., 1.), 0.);
        b->setPos(0., 0., -10.);
        b->activate();

        b = world->createBodyCube(1., 1.);
        b->setPos(-6., 6., -5.);
        b->activate();

        b = world->createBodySphere(1.3, 2.);
        b->setPos(-3., 0., 0.);
        b->activate();

        b = world->createBodyCube(1., 1.);
        b->visBody()->setColor(osg::Vec4(0., 0., 0., 1.));
        b->setPos(-3., 0.2, 3.);
        b->activate();

        b = world->createBodyCylinderZ(.5, 1., 3.);
        b->setPos(3., 0., 0.);
        b->activate();

        b = world->createBodyCylinderX(.5, 1., 3.);
        b->setPos(0., 3., 0.);
        b->activate();

        b = world->createBodyCylinderY(.5, 1., 3.);
        b->setPos(0., -3., 0.);
        b->activate();

        createFixed();
        createHinge();
        createRobot();
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
    }

  protected:
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
    }
};


int main(int argc, char *argv[])
{
    S s;

    s.run();

    return 0;
}

