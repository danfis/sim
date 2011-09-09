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

#include <unistd.h>
#include <string.h>
#include <iostream>

#include <sim/sim.hpp>
#include <sim/world.hpp>
#include <sim/msg.hpp>
#include <sim/comp/povray_full.hpp>
#include <sim/comp/povray_step.hpp>
#include <sim/comp/blender.hpp>

#include "bunny.hpp"

using sim::Vec3;
using sim::Quat;
using namespace std;

bool use_ode = true;
const char *povray_full = 0;
const char *povray_step = 0;
const char *blender = 0;

class S : public sim::Sim {
  public:
    S()
        : Sim()
    {
        if (use_ode){
            initODE();
        }else{
            initBullet();
        }

        if (povray_full){
            sim::comp::PovrayFull *pov = new sim::comp::PovrayFull(povray_full);
            addComponent(pov);
        }

        if (povray_step){
            sim::comp::PovrayStep *pov = new sim::comp::PovrayStep(povray_step);
            addComponent(pov);
        }

        if (blender){
            sim::comp::Blender *b = new sim::comp::Blender(blender);
            addComponent(b);
        }

        setTimeStep(sim::Time::fromMs(20));
        setTimeSubSteps(2);

        setTimeLimit(sim::Time::fromMs(100000));

        pauseSimulation();

        sim::Body *b;
        sim::World *w = world();

        createShapes();
        createCompound();
        createFixed();
        createHinge();
        createHingeLim();
        createHinge2();
        createHinge2Lim();
        createRobot();
        createRobotMove(sim::Vec3(10., -10., -4.9));
        createRobotMove(sim::Vec3(15., -15., -4.9));
        createSeeSaw();
        createBunny();

        Vec3 verts[] = { Vec3(-10., 10., -0.5),
                         Vec3(0., 10., -0.8),
                         Vec3(10., 10., -0.9),
                         Vec3(-10., 0., 0.5),
                         Vec3(0., 0., 0.),
                         Vec3(20., 0., 0.),
                         Vec3(-10., -10., 0.4),
                         Vec3(0., -20., 0.),
                         Vec3(20., -20., 0.) };
        unsigned int ind[] = { 0, 3, 1,
                               3, 4, 1,
                               1, 4, 2,
                               4, 5, 2,
                               3, 6, 4,
                               6, 7, 4,
                               4, 7, 5,
                               7, 8, 5 };
        b = w->createBodyTriMesh(verts, 9, ind, 24);
        b->visBody()->setColor(0.9, 0.6, 0.3, 1.);
        b->setPos(0., 0., -5.3);
        b->activate();
    }

  protected:
    void initBullet()
    {
#ifdef SIM_HAVE_BULLET
        DBG("Using Bullet");

        sim::WorldBullet *w = sim::WorldFactory::Bullet();
        setWorld(w);
#endif /* SIM_HAVE_BULLET */
    }

    void initODE()
    {
#ifdef SIM_HAVE_ODE
        DBG("Using ODE");

        sim::WorldODE *w = sim::WorldFactory::ODE();

        setWorld(w);
        w->setCFM(1e-10);
        w->setCFM(0.01);
        w->setERP(0.5);
        w->setStepType(sim::WorldODE::STEP_TYPE_QUICK);
        w->setAutoDisable(0.01, 0.01, 5, 0.);

        //w->setContactSoftCFM(0.0000001);
        //w->setContactApprox1(false);
        //w->setContactApprox2(false);
        //w->setContactBounce(0.1, 0.1);
#endif /* SIM_HAVE_ODE */
    }

    void createShapes()
    {
        sim::World *w = world();
        sim::Body *b;

        b = w->createBodySphere(1.3, 2.);
        b->visBody()->setColor(osg::Vec4(0.5, 0.1, 0.8, 0.3));
        b->setPos(-3., 0., -3.);
        b->activate();


        b = w->createBodyCube(1., 1.);
        b->visBody()->setColor(osg::Vec4(0., 0., 0., 1.));
        b->setPos(-3., 0.2, 3.);
        b->activate();

        b = w->createBodyCylinderZ(.5, 1., 3.);
        b->visBody()->setTexture("wood.ppm");
        b->visBody()->setColor(1., 0., 0., 1.);
        b->visBody()->setText("CylZ", 1., osg::Vec4(0., 0., 0., 1.));
        b->setPos(3., 0., 0.);
        b->activate();

        b = w->createBodyCylinderX(.5, 1., 3.);
        b->visBody()->setText("CylX", 1., osg::Vec4(0., 0., 0., 1.));
        b->setPos(0., 3., 0.);
        b->activate();

        b = w->createBodyCylinderY(.5, 1., 3.);
        b->visBody()->setText("CylY", 1., osg::Vec4(0., 0., 0., 1.));
        b->setPos(0., -3., 0.);
        b->activate();
    }

    void createCompound()
    {
        sim::World *w = world();
        sim::Body *b;
        int id;

        sim::Body *c = w->createBodyCompound();
        c->addCube(1.);
        c->addBox(Vec3(0.5, 0.1, 0.3), SIM_BODY_DEFAULT_VIS, Vec3(0.7, 0.7, 0.7));
        c->addSphere(0.7, SIM_BODY_DEFAULT_VIS, Vec3(-0.7, 0.7, 0.7));
        c->addCylinderZ(0.4, 1., SIM_BODY_DEFAULT_VIS, Vec3(1., 0., 0.));
        c->addCylinderY(0.4, 1., SIM_BODY_DEFAULT_VIS, Vec3(2., 0., 0.));
        c->addCylinderX(0.4, 1., SIM_BODY_DEFAULT_VIS, Vec3(3., 0., 0.));
        c->addTriMesh(bunny_coords, bunny_coords_len,
                      bunny_ids, bunny_ids_len,
                      SIM_BODY_DEFAULT_VIS,
                      Vec3(5., 3., 0.),
                      Quat(Vec3(1., 0., 0.), M_PI * .5));
        c->setPos(Vec3(15., 0., 13.));
        c->activate();

        c = w->createBodyCompound();
        c->addCube(1.);
        c->addBox(Vec3(0.5, 0.1, 0.3), SIM_BODY_DEFAULT_VIS, Vec3(0.7, 0.7, 0.7));
        c->addSphere(0.7, SIM_BODY_DEFAULT_VIS, Vec3(-0.7, 0.7, 0.7));
        c->addCylinderZ(0.4, 1., SIM_BODY_DEFAULT_VIS, Vec3(1., 0., 0.));
        c->addCylinderY(0.4, 1., SIM_BODY_DEFAULT_VIS, Vec3(2., 0., 0.));
        c->addCylinderX(0.4, 1., SIM_BODY_DEFAULT_VIS, Vec3(3., 0., 0.));
        c->addTriMesh(bunny_coords, bunny_coords_len,
                      bunny_ids, bunny_ids_len,
                      SIM_BODY_DEFAULT_VIS,
                      Vec3(5., 3., 0.),
                      Quat(Vec3(1., 0., 0.), M_PI * .5));
        c->setPos(Vec3(15., -5., 13.));
        c->setMassCube(3., 3.);
        c->activate();

        c = w->createBodyCompound();
        id = c->addBox(Vec3(15., 15., 0.5));
        c->visBody(id)->setColor(0.6, 0., 0.6, 0.1);
        id = c->addBox(Vec3(5., 15., 0.5), SIM_BODY_DEFAULT_VIS,
                       Vec3(9., 0., 2.), Quat(Vec3(0., 1., 0.), -M_PI / 4.));
        c->visBody(id)->setColor(0.6, 0., 0.6, 0.1);
        id = c->addBox(Vec3(5., 15., 0.5), SIM_BODY_DEFAULT_VIS,
                       Vec3(-9., 0., 2.), Quat(Vec3(0., 1., 0.), M_PI / 4.));
        c->visBody(id)->setColor(0.6, 0., 0.6, 0.1);
        id = c->addBox(Vec3(19., .5, 5.), SIM_BODY_DEFAULT_VIS,
                       Vec3(0., 7.5, 2.5));
        c->visBody(id)->setColor(0.6, 0., 0.6, 0.1);
        id = c->addBox(Vec3(19., .5, 5.), SIM_BODY_DEFAULT_VIS,
                       Vec3(0., -7.5, 2.5));
        c->visBody(id)->setColor(0.6, 0., 0.6, 0.1);

        c->setPos(Vec3(15., 0., 10.));
        c->activate();

        Vec3 start(13., -2., 20.);
        for (size_t i = 0; i < 15; i++){
            for (size_t j = 0; j < 15; j++){
                b = w->createBodyCube(0.5, 1.);
                b->visBody()->setColor(0., 0.6, 0.6, 0.3);
                b->setPos(start + Vec3(0.55 * i, 0.55 * j, 0.));
                b->activate();
            }
        }
    }

    void createFixed()
    {
        sim::Vec3 pos(-6., -6., -3.);
        sim::Body *cu = world()->createBodyCube(1., 1.);
        sim::Body *s = world()->createBodySphere(0.5, 3.);
        sim::Joint *j;

        DBG(cu);
        DBG(cu->visBody());
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
        sim::Vec3 pos(10., -2., -4.6);
        sim::Body *cy = world()->createBodyCylinderZ(0.5, 0.5, .2);
        sim::Body *cu = world()->createBodyCube(1., 1.);
        sim::Joint *j;

        cu->setPos(pos);
        cy->setPos(pos + sim::Vec3(0., 0., 2.));

        j = world()->createJointHinge(cu, cy, pos + sim::Vec3(0., 0., 1.), sim::Vec3(0., 1., 0.));

        cu->activate();
        cy->activate();
        j->activate();

        cu->visBody()->setText("Hinge", 1., osg::Vec4(0.7, 0.6, 0.3, 1.));

        sim::Body *b2 = world()->createBodyBox(Vec3(.5, 2.0, 0.5), 2.);
        b2->visBody()->setColor(osg::Vec4(0.5, 0.5, 0.5, 0.5));
        b2->setPos(pos + Vec3(-0.5, -1.0, 2.7));
        b2->activate();
    }

    void createHingeLim()
    {
        sim::Vec3 pos(10., -4., -4.6);
        sim::Body *cy = world()->createBodyCylinderZ(0.5, 0.5, .2);
        sim::Body *cu = world()->createBodyCube(1., 1.);
        sim::Joint *j;

        cu->setPos(pos);
        cy->setPos(pos + sim::Vec3(0., 0., 2.));

        j = world()->createJointHinge(cu, cy, pos + sim::Vec3(0., 0., 1.), sim::Vec3(0., 1., 0.));
        j->setParamLimitLoHi(-M_PI / 2., 0.);

        cu->activate();
        cy->activate();
        j->activate();

        cu->visBody()->setText("Hinge lim.", 1., osg::Vec4(0.7, 0.6, 0.3, 1.));
    }

    void createHinge2()
    {
        sim::Vec3 pos(6., -6., -3.76);
        sim::Body *s = world()->createBodyCylinderY(0.5, 0.5, 0.5);
        sim::Body *b = world()->createBodyBox(Vec3(1.5, 1.5, 3.), 10.);
        sim::Joint *j;

        b->setPos(pos);
        s->setPos(pos + sim::Vec3(2.0, 0., 1.));

        j = world()->createJointHinge2(b, s, pos + Vec3(0., 0., 1.),
                                       Vec3(0., 0., 1.), Vec3(1., 0., 0.));

        b->activate();
        s->activate();
        j->activate();

        b->visBody()->setColor(0.5, 0.7, 0.3, 0.3);
        b->visBody()->setText("Hinge2", 1., osg::Vec4(0.5, 0.7, 0.3, 1.));
        s->visBody()->setColor(0., 1., 0., 1.);

        sim::Body *b2 = world()->createBodyCube(0.5, 1.);
        b2->visBody()->setColor(1., 0., 0., 0.3);
        b2->setPos(pos + Vec3(2.1, -0.3, 2.5));
        b2->activate();
    }

    void createHinge2Lim()
    {
        sim::Vec3 pos(6., -11., -3.76);
        sim::Body *s = world()->createBodyCylinderY(0.5, 0.5, 1.5);
        sim::Body *b = world()->createBodyBox(Vec3(1.5, 1.5, 3.), 10.);
        sim::Joint *j;

        b->setPos(pos);
        s->setPos(pos + sim::Vec3(2.0, 0., 1.));

        j = world()->createJointHinge2(b, s, pos + Vec3(0., 0., 1.),
                                       Vec3(0., 0., 1.), Vec3(1., 0., 0.));

        j->setParamLimitLoHi(-M_PI / 4., M_PI / 4.);

        b->activate();
        s->activate();
        j->activate();

        b->visBody()->setColor(0.5, 0.7, 0.3, 0.3);
        b->visBody()->setText("Hinge2 lim.", 1., osg::Vec4(0.5, 0.7, 0.3, 1.));
        s->visBody()->setColor(0., 1., 0., 1.);

        sim::Body *b2 = world()->createBodyCube(0.5, 2.);
        b2->visBody()->setColor(1., 0., 0., 0.3);
        b2->setPos(pos + Vec3(2.1, -0.3, 2.5));
        b2->activate();
    }

    void createRobot()
    {
        sim::Vec3 pos(3. + 2., 3., -5.);
        sim::Body *chasis;
        sim::Body *w[4];
        sim::Joint *jw[4];
        size_t i;

        chasis = world()->createBodyBox(sim::Vec3(.6, 1., 0.4), 1.);
        for (i = 0; i < 4; i++){
            w[i] = world()->createBodyCylinderX(0.2, 0.2, 1.);
        }

        chasis->setPos(pos);
        w[0]->setPos(pos.x() + 0.415, pos.y() + 0.4, pos.z() - 0.2);
        w[1]->setPos(pos.x() - 0.415, pos.y() + 0.4, pos.z() - 0.2);
        w[2]->setPos(pos.x() + 0.415, pos.y() - 0.4, pos.z() - 0.2);
        w[3]->setPos(pos.x() - 0.415, pos.y() - 0.4, pos.z() - 0.2);

        for (i = 0; i < 4; i++){
            jw[i] = world()->createJointHinge(chasis, w[i], w[i]->pos(), Vec3(-1., 0., 0.));
        }

        chasis->activate();
        for (i = 0; i < 4; i++){
            w[i]->activate();
            jw[i]->activate();
        }

        chasis->visBody()->setText("Robot", 1., osg::Vec4(0.5, 0.6, 0.3, 1.));
    }

    void createRobotMove(const Vec3 &pos)
    {
        sim::Body *chasis;
        sim::Body *w[4];
        sim::Joint *jw[4];
        size_t i;

        chasis = world()->createBodyBox(sim::Vec3(.6, 1., 0.4), 1.);
        for (i = 0; i < 4; i++){
            w[i] = world()->createBodyCylinderX(0.2, 0.2, 1.);
        }

        chasis->setPos(pos);
        w[0]->setPos(pos.x() + 0.415, pos.y() + 0.4, pos.z() - 0.2);
        w[1]->setPos(pos.x() - 0.415, pos.y() + 0.4, pos.z() - 0.2);
        w[2]->setPos(pos.x() + 0.415, pos.y() - 0.4, pos.z() - 0.2);
        w[3]->setPos(pos.x() - 0.415, pos.y() - 0.4, pos.z() - 0.2);

        for (i = 0; i < 4; i++){
            jw[i] = world()->createJointHinge(chasis, w[i], w[i]->pos(), Vec3(1., 0., 0.));
        }

        jw[0]->setParamVel(5.);
        jw[0]->setParamFMax(10.);
        jw[1]->setParamVel(5.);
        jw[1]->setParamFMax(10.);

        chasis->activate();
        for (i = 0; i < 4; i++){
            w[i]->activate();
            jw[i]->activate();
        }

        chasis->visBody()->setText("Robot move", 1., osg::Vec4(0.5, 0.6, 0.3, 1.));
    }

    void createBunny()
    {
        sim::Body *bunny;

        bunny = world()->createBodyTriMesh(bunny_coords, bunny_coords_len,
                                           bunny_ids, bunny_ids_len);
        bunny->visBody()->setColor(0.4, 0.4, 0.4, 1.);
        bunny->setRot(sim::Quat(Vec3(1., 0., 0.), M_PI * 0.5));
        bunny->activate();
    }

    void createSeeSaw()
    {
        sim::Vec3 pos(15., -5., -5);
        sim::Body *b;
        sim::Joint *j;

        b = world()->createBodyBox(Vec3(2., 5., 0.01), 1.);
        b->visBody()->setColor(0.7, 0.5, 0.1, 1.);
        b->visBody()->setTexture("wood.ppm");
        b->setPos(pos);
        b->setRot(Quat(Vec3(1., 0., 0.), M_PI / 50.));
        b->activate();

        j = world()->createJointHinge(b, 0, b->pos(), Vec3(1., 0., 0.));
        j->activate();
    }
};


int main(int argc, char *argv[])
{
    for (int i = 1; i < argc; i++){
        if (strcmp(argv[i], "--ode") == 0){
            use_ode = true;
        }else if (strcmp(argv[i], "--bullet") == 0){
            use_ode = false;
        }else if (strcmp(argv[i], "--povray-full") == 0 && i + 1 < argc){
            povray_full = argv[i + 1];
            i++;
        }else if (strcmp(argv[i], "--povray-step") == 0 && i + 1 < argc){
            povray_step = argv[i + 1];
            i++;
        }else if (strcmp(argv[i], "--blender") == 0 && i + 1 < argc){
            blender = argv[i + 1];
            i++;
        }
    }

    S s;
    s.run();

    return 0;
}
