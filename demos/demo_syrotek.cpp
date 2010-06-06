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
#include <sim/comp/syrotek.hpp>
#include "sim/comp/povray.hpp"

using namespace sim::ode;
using sim::Vec3;
using sim::Time;
using namespace std;

static bool use_cam = false;
static bool use_rf = false;

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
        _useJoystick();

        if (use_cam){
            _useCamera(100, 100);
            cam()->enableView();
        }

        if (use_rf){
            _useRangeFinder();
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
    for (int i = 1; i < argc; i++){
        if (strcmp(argv[i], "--cam") == 0){
            use_cam = true;
        }else if (strcmp(argv[i], "--rf") == 0){
            use_rf = true;
        }else if (strcmp(argv[i], "--help") == 0){
            printf("%s [OPTIONS]\n", argv[0]);
            printf("  OPTIONS:\n");
            printf("    --help  Print out this help\n");
            printf("    --cam   Turn on camera sensor\n");
            printf("    --rf    Turn on range finder sensor\n");
            printf("\n");
            return -1;
        }
    }

    S s;
    s.run();

    return 0;
}
