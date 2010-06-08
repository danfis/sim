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

#include <unistd.h>
#include <iostream>

#include "sim/bullet/world.hpp"
#include "sim/sim.hpp"
#include "msg.hpp"

#include "bunny.hpp"

using namespace sim::bullet;
using sim::Vec3;
using sim::Quat;
using namespace std;


class S : public sim::Sim {
  public:
    S()
        : sim::Sim()
    {
        setTimeStep(sim::Time::fromMs(20));
        setTimeSubSteps(2);

        setTimeLimit(sim::Time::fromMs(100000));
        pauseSimulation();

        World *w = new World();
        sim::Body *b;

        setWorld(w);
        //w->setCFM(1e-10);
        //w->setCFM(0.01);
        //w->setERP(0.5);
        //w->setStepType(World::STEP_TYPE_QUICK);
        //w->setAutoDisable(0.01, 0.01, 5, 0.);
        //w->setContactSoftCFM(0.0000001);
        //w->setContactApprox1(false);
        //w->setContactApprox2(false);
        //w->setContactBounce(0.1, 0.1);
    
        /*
        b = w->createBodyBox(sim::Vec3(20., 20., 1.), 0.);
        b->setPos(0., 0., -10.);
        b->activate();
        */

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

        {
            int id;

            BodyCompound *c = new BodyCompound(w);
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

            b = w->createBodyCube(0.5, 1.);
            b->setPos(15., 0., 15.);
            b->activate();

            c = new BodyCompound(w);
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

            c = new BodyCompound(w);
            id = c->addBox(Vec3(15., 15., 0.5));
            c->visBody(id)->setColor(0.6, 0., 0.6, 0.1);
            id = c->addBox(Vec3(5., 15., 0.5), SIM_BODY_DEFAULT_VIS,
                           Vec3(9., 0., 2.),
                           Quat(Vec3(0., 1., 0.), -M_PI / 4.));
            c->visBody(id)->setColor(0.6, 0., 0.6, 0.1);
            id = c->addBox(Vec3(5., 15., 0.5), SIM_BODY_DEFAULT_VIS,
                           Vec3(-9., 0., 2.),
                           Quat(Vec3(0., 1., 0.), M_PI / 4.));
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

        /*
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
        */

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
        DBG(j);

        cu->activate();
        cy->activate();
        j->activate();

        cu->visBody()->setText("Hinge", 1., osg::Vec4(0.7, 0.6, 0.3, 1.));
    }

    void createRobot()
    {
        /*
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
        */
    }
};


int main(int argc, char *argv[])
{
    S s;

    s.run();

    return 0;
}

