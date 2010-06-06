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
#include <sim/comp/povray.hpp>
#include <sim/comp/syrotek.hpp>

using namespace sim::ode;
using sim::Scalar;
using sim::Vec3;
using sim::Quat;
using sim::Time;
using namespace std;

static bool dump = false;

class Leader : public sim::comp::Syrotek {
  public:
    Leader(const sim::Vec3 &pos)
        : sim::comp::Syrotek(pos)
    {
    }

    void init(sim::Sim *sim)
    {
        sim::comp::Syrotek::init(sim);
        _useKeyboard();
        _useJoystick();
        _useCamera(200, 200);
        cam()->enableView();

        if (dump)
            cam()->enableDump("cam-leader/");
    }
};

class Follower : public sim::comp::Syrotek {
    osg::Vec4 _ref_color;

  public:
    Follower(const Vec3 &pos, const sim::comp::Syrotek *leader)
        : sim::comp::Syrotek(pos, osg::Vec4(0., 0.1, 0.7, 0.7)),
          _ref_color(leader->color())
    {
        DBG(DBGV(_ref_color));
    }

    void init(sim::Sim *s)
    {
        sim::comp::Syrotek::init(s);

        _useCamera(300, 300);
        cam()->enableView();
        cam()->attachToBody(robot()->chasis(), Vec3(0.13, 0., 0.15),
                            Quat(Vec3(0., 1., 0.), M_PI / 4.));

        if (dump)
            cam()->enableDump("cam-follower/");

        sim()->regPreStep(this);
    }

    Scalar _evalPixel(const osg::Vec4 &color)
    {
        Scalar e = 0.;

        for (size_t i = 0; i < 3; i++){
            e += (color[i] - _ref_color[i]) * (color[i] - _ref_color[i]);
        }
        e = sqrt(e);
        if (e < 0.0000001)
            e = 0.0000001;
        e = 1. / e;
        return e;
    }

    osg::Vec2 _findMean(osg::Image *image)
    {
        size_t width = image->s();
        size_t height = image->t();
        size_t w, h;
        osg::Vec2 mean(0., 0.);
        osg::Vec4 color;
        osg::Vec2 sum(0., 0.);
        Scalar eval, sum_eval;

        //DBG(image->s() << " " << image->t() << " " << image->r());
        for (w = 0; w < width; w++){
            for (h = 0; h < height; h++){
                color = image->getColor(w, h);
                eval = _evalPixel(color);
                //DBG(color.x() << " " << color.y() << " " << color.z());

                sum += osg::Vec2(w, h) * eval;
                sum_eval += eval;
            }
        }

        if (sum_eval > 0.){
            mean = sum / sum_eval;
        }else{
            mean = osg::Vec2(width, height);
        }

        mean -= osg::Vec2(width, height) / 2.;

        return mean;
    }

    void cbPreStep()
    {
        osg::Image *image = cam()->image();

        if (!image->valid())
            return;

        osg::Vec2 mean = _findMean(image);
        Scalar kf = 0.5, ko = 0.3;
        Scalar vleft, vright;

        //DBG(mean.x() << " " << mean.y());
        vleft = mean.y();
        vright = mean.y();

        vright = vleft = mean.y() * kf;
        vleft += (mean.x() / 2.) * ko;
        vright += (-mean.x() / 2.) * ko;
        //DBG("v: " << vleft << " " << vright);

        robot()->setVelLeft(vleft);
        robot()->setVelRight(vright);
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
        createRobots();

        if (dump){
            sim::comp::Povray *pov = new sim::comp::Povray("povray/");
            addComponent(pov);
        }
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
            b->setPos(0., 0., 1.);
            b->activate();

            b = w->createBodySphere(0.1, 0.1);
            b->setPos(0.35, 0.4, .5);
            b->activate();
        }
    }

    void createRobots()
    {
        Leader *leader = new Leader(Vec3(0., 0., 0.08));
        addComponent(leader);
        Follower *fol = new Follower(Vec3(-0.2, 0., 0.08), leader);
        addComponent(fol);
    }
};

int main(int argc, char *argv[])
{
    if (argc > 1){
        if (strcmp(argv[1], "--dump") == 0)
            dump = true;
    }

    S s;

    s.run();

    return 0;
}
