/***
 * sim
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
 *
 *  This file is part of sim.
 *
 *  sim is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation; either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  sim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <sim/sim.hpp>
#include <sim/ode/world.hpp>
#include <sim/msg.hpp>
#include <sim/sensor/camera.hpp>

#include "robot_sssa.hpp"

using namespace sim::ode;
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

        setTimeStep(Time::fromMs(10));
        setTimeSubSteps(2);

        setWorld(w);

        w->setCFM(0.0001);
        w->setERP(0.8);
        w->setStepType(World::STEP_TYPE_QUICK);
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

        pauseSimulation();
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
        sim::robot::SSSA *r1, *r2;
        r1 = new sim::robot::SSSA(world(), Vec3(2., 2., .6));
        r1->activate();

        r2 = new sim::robot::SSSA(world(), Vec3(.746, 2., .6));
        r2->activate();

        DBG("can connect: " << r1->canConnectTo(*r2));
        r1->connectTo(*r2);

        SSSAComp *comp;
        comp = new SSSAComp(r1);
        addComponent(comp);

        r2 = new sim::robot::SSSA(world(), Vec3(2., 3.254, .6),
                                  Quat(Vec3(0., 0., 1.), M_PI / 2.));
        r2->activate();
        DBG("can connect: " << r2->canConnectTo(*r1));
        r2->connectTo(*r1);


        r2 = new sim::robot::SSSA(world(), Vec3(2., 0.746, .6),
                                  Quat(Vec3(0., 0., 1.), -M_PI / 2.));
        r2->activate();
        DBG("can connect: " << r2->canConnectTo(*r1));
        r2->connectTo(*r1);

    }

};

int main(int argc, char *argv[])
{
    S s;

    s.run();

    return 0;
}
