/***
 * sim
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
 *
 *  This file is part of sim.
 *
 *  sim is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation; either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  sim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <sim/sim.hpp>
#include <sim/msg.hpp>
#include <sim/comp/syrotek.hpp>
#include "sim/comp/povray.hpp"
#include "sim/alg/surfnav.hpp"

using sim::Vec3;
using sim::Time;
using namespace std;

static const char *load = 0;
static const char *save = 0;

class Syrotek : public sim::comp::Syrotek {
    sim::alg::SurfSegment _surf;
    float _fwd, _k;

  public:
    Syrotek(const sim::Vec3 &pos)
        : sim::comp::Syrotek(pos)
    {
        if (load != 0){
            DBG("Loading SurfSegment from " << load);
            _surf.learnLoad(load);
        }
    }

    void init(sim::Sim *sim)
    {
        sim::comp::Syrotek::init(sim);
        _useKeyboard();
        _useJoystick();
        _useCamera(320, 240);

        cam()->enableView();

        sim->regPreStep(this);
    }

    void cbPreStep()
    {
        if (_surf.learning()
                && cam()->image()
                && cam()->image()->s() > 0
                && cam()->image()->t() > 0){
            Vec3 odo = robot()->odometry();
            //DBG("odo: " << odo.x() << " " << odo.y() << " " << odo.z());
            _surf.learn(cam()->image(), odo.x(), odo.y());
        }else if (_surf.traversing()
                    && cam()->image()
                    && cam()->image()->s() > 0
                    && cam()->image()->t() > 0){
            Vec3 odo = robot()->odometry();
            float hd = _surf.traverse(cam()->image(), odo.x(), odo.y());
            DBG("SURF hd = " << hd);

            robot()->setVelLeft(_fwd + -_k * hd);
            robot()->setVelRight(_fwd + _k * hd);
        }
    }

    void _keyPressedMsg(const sim::MessageKeyPressed &msg)
    {
        int key = msg.key();

        if (key == 's'){
            float x = robot()->chasis()->pos().x();
            float y = robot()->chasis()->pos().y();
            _surf.learnStart(x, y);
            robot()->setVelLeft(15.);
            robot()->setVelRight(15.);
            DBG("SurfNav segment started");
        }else if (key == 'S'){
            _surf.learnFinish();
            robot()->setVelLeft(0.);
            robot()->setVelRight(0.);

            DBG("SurfNav segment finished");

            if (save != 0){
                DBG("Saving SurfSegment into " << save);
                _surf.learnSave(save);
            }
        }else if (key == 't'){
            float x = robot()->chasis()->pos().x();
            float y = robot()->chasis()->pos().y();
            _surf.traverseStart(x, y);
            robot()->setVelLeft(4.);
            robot()->setVelRight(4.);
            _fwd = 0.f;
            _k = .1f;
            DBG("SurfNav traverse");
        }else if (key == 'T'){
            _surf.traverseFinish();
            robot()->setVelLeft(0.);
            robot()->setVelRight(0.);
            DBG("SurfNav traverse STOPED");
        }else if (key == 'y'){
            _fwd += 0.1;
        }else if (key == 'Y'){
            _fwd -= 0.1;
        }else if (key == 'u'){
            _k += 0.1f;
            DBG("k: " << _k);
        }else if (key == 'U'){
            _k -= 0.1f;
            DBG("k: " << _k);
        }else{
            sim::comp::Syrotek::_keyPressedMsg(msg);
        }
    }
};


class S : public sim::Sim {
  public:
    S()
        : Sim()
    {
        sim::ode::World *w = new sim::ode::World();

        setTimeStep(Time::fromMs(20));
        setTimeSubSteps(2);

        w->setCFM(0.0001);
        w->setERP(0.8);
        w->setAutoDisable(0.01, 0.01, 5, 0.);
        w->setContactApprox1(true);
        w->setContactApprox2(true);
        w->setContactBounce(0.1, 0.1);

        setWorld(w);

        createArena();
        createRobot();
    }

    void createArena()
    {
        osg::Vec4 color(0., 0.7, 0.1, 1.);
        sim::Body *c;
        int id;
        sim::ode::World *w = (sim::ode::World *)world();

        // floor
        c = w->createBodyCompound();
        id = c->addBox(Vec3(10., 4., 0.05));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("ground.ppm");

        // left and right wall
        id = c->addBox(Vec3(0.05, 4., 1.), SIM_BODY_DEFAULT_VIS, Vec3(-5., 0., .5));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("ground.ppm");
        id = c->addBox(Vec3(0.05, 4., 1.), SIM_BODY_DEFAULT_VIS, Vec3(5., 0., .5));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("ground.ppm");

        // other two walls
        id = c->addBox(Vec3(10., .1, 0.4), SIM_BODY_DEFAULT_VIS, Vec3(0., 2., .2));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("ground.ppm");
        id = c->addBox(Vec3(10., .1, 0.4), SIM_BODY_DEFAULT_VIS, Vec3(0., -2., .2));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("ground.ppm");

        // pillars
        {
            Vec3 size(0.2, 0.2, 2.);
            osg::Vec4 color(0.6, 0.6, 0.8, 1.);

            id = c->addBox(size, SIM_BODY_DEFAULT_VIS, Vec3(-3.5, -1., 1.));
            c->visBody(id)->setColor(color);
            c->visBody(id)->setTexture("concrete.ppm");
            id = c->addBox(size, SIM_BODY_DEFAULT_VIS, Vec3(-2.5, -1., 1.));
            c->visBody(id)->setColor(color);
            c->visBody(id)->setTexture("concrete.ppm");
            id = c->addBox(size, SIM_BODY_DEFAULT_VIS, Vec3(-1.5, -1., 1.));
            c->visBody(id)->setColor(color);
            c->visBody(id)->setTexture("concrete.ppm");
            id = c->addBox(size, SIM_BODY_DEFAULT_VIS, Vec3(-.5, -1., 1.));
            c->visBody(id)->setColor(color);
            c->visBody(id)->setTexture("concrete.ppm");
            id = c->addBox(size, SIM_BODY_DEFAULT_VIS, Vec3(.5, -1., 1.));
            c->visBody(id)->setColor(color);
            c->visBody(id)->setTexture("concrete.ppm");
            id = c->addBox(size, SIM_BODY_DEFAULT_VIS, Vec3(1.5, -1., 1.));
            c->visBody(id)->setColor(color);
            c->visBody(id)->setTexture("concrete.ppm");
            id = c->addBox(size, SIM_BODY_DEFAULT_VIS, Vec3(2.5, -1., 1.));
            c->visBody(id)->setColor(color);
            c->visBody(id)->setTexture("concrete.ppm");
            id = c->addBox(size, SIM_BODY_DEFAULT_VIS, Vec3(3.5, -1., 1.));
            c->visBody(id)->setColor(color);
            c->visBody(id)->setTexture("concrete.ppm");

            id = c->addBox(size, SIM_BODY_DEFAULT_VIS, Vec3(-3.5, 1., 1.));
            c->visBody(id)->setColor(color);
            c->visBody(id)->setTexture("concrete.ppm");
            id = c->addBox(size, SIM_BODY_DEFAULT_VIS, Vec3(-2.5, 1., 1.));
            c->visBody(id)->setColor(color);
            c->visBody(id)->setTexture("concrete.ppm");
            id = c->addBox(size, SIM_BODY_DEFAULT_VIS, Vec3(-1.5, 1., 1.));
            c->visBody(id)->setColor(color);
            c->visBody(id)->setTexture("concrete.ppm");
            id = c->addBox(size, SIM_BODY_DEFAULT_VIS, Vec3(-.5, 1., 1.));
            c->visBody(id)->setColor(color);
            c->visBody(id)->setTexture("concrete.ppm");
            id = c->addBox(size, SIM_BODY_DEFAULT_VIS, Vec3(.5, 1., 1.));
            c->visBody(id)->setColor(color);
            c->visBody(id)->setTexture("concrete.ppm");
            id = c->addBox(size, SIM_BODY_DEFAULT_VIS, Vec3(1.5, 1., 1.));
            c->visBody(id)->setColor(color);
            c->visBody(id)->setTexture("concrete.ppm");
            id = c->addBox(size, SIM_BODY_DEFAULT_VIS, Vec3(2.5, 1., 1.));
            c->visBody(id)->setColor(color);
            c->visBody(id)->setTexture("concrete.ppm");
            id = c->addBox(size, SIM_BODY_DEFAULT_VIS, Vec3(3.5, 1., 1.));
            c->visBody(id)->setColor(color);
            c->visBody(id)->setTexture("concrete.ppm");

            color.z() -= 0.2;

            id = c->addSphere(0.08, SIM_BODY_DEFAULT_VIS, Vec3(-3.5, -1.05, .1));
            c->visBody(id)->setColor(color);
            c->visBody(id)->setTexture("rocks.ppm");
            id = c->addSphere(0.08, SIM_BODY_DEFAULT_VIS, Vec3(-2.5, -1.04, .05));
            c->visBody(id)->setColor(color);
            c->visBody(id)->setTexture("rocks.ppm");
            id = c->addSphere(0.08, SIM_BODY_DEFAULT_VIS, Vec3(-1.5, -1.055, .12));
            c->visBody(id)->setColor(color);
            c->visBody(id)->setTexture("rocks.ppm");
            id = c->addSphere(0.08, SIM_BODY_DEFAULT_VIS, Vec3(-.5, -1.05, .2));
            c->visBody(id)->setColor(color);
            c->visBody(id)->setTexture("rocks.ppm");
            id = c->addSphere(0.08, SIM_BODY_DEFAULT_VIS, Vec3(.5, -1.03, .1));
            c->visBody(id)->setColor(color);
            c->visBody(id)->setTexture("rocks.ppm");
            id = c->addSphere(0.08, SIM_BODY_DEFAULT_VIS, Vec3(1.5, -1.05, .15));
            c->visBody(id)->setColor(color);
            c->visBody(id)->setTexture("rocks.ppm");
            id = c->addSphere(0.08, SIM_BODY_DEFAULT_VIS, Vec3(2.5, -1.03, .11));
            c->visBody(id)->setColor(color);
            c->visBody(id)->setTexture("rocks.ppm");
        }

        id = c->addBox(Vec3(1., 0.2, 0.5), SIM_BODY_DEFAULT_VIS, Vec3(3., 0.1, 0.25));
        c->visBody(id)->setColor(osg::Vec4(0.7, 0.3, 0.5, 1.));

        c->activate();
    }

    void createRobot()
    {
        Syrotek *comp = new Syrotek(Vec3(-3.5, -1.2, 0.08));
        addComponent(comp);
    }
};

int main(int argc, char *argv[])
{
    for (int i = 1; i < argc; i++){
        if (strcmp(argv[i], "--save") == 0 && i + 1 < argc){
            save = argv[i + 1];
            i++;
        }else if (strcmp(argv[i], "--load") == 0 && i + 1 < argc){
            load = argv[i + 1];
            i++;
        }
    }

    S s;
    s.run();

    return 0;
}

