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
#include <sim/world.hpp>
#include <sim/sensor/camera.hpp>
#include <sim/comp/sssa.hpp>
#include <sim/msg.hpp>

using sim::Scalar;
using sim::Vec3;
using sim::Quat;
using sim::Time;
using namespace std;

bool use_ode = true;
size_t step = 0;
size_t STEPS = 1000;
Scalar maxvel = 5.;
Scalar minvel = -5.;

struct State {
    Scalar vel_left;
    Scalar vel_right;
    Scalar vel_arm;
};

class SSSA : public sim::comp::SSSA {
    std::vector<State> _states;
    int id;

  public:
    SSSA(int id, const Vec3 &pos, const Quat &rot = Quat(0., 0., 0., 1.))
        : sim::comp::SSSA(pos, rot), _states(STEPS)
    {
        Scalar r;

        for (size_t i = 0; i < STEPS; i++){
            r = (((Scalar)rand() / (Scalar)RAND_MAX) * .5) - .25;
            _states[i].vel_left = r;
            r = (((Scalar)rand() / (Scalar)RAND_MAX) * .5) - .25;
            _states[i].vel_right = r;
            r = (((Scalar)rand() / (Scalar)RAND_MAX) * .5) - .25;
            _states[i].vel_arm = r;
        }
    }

    void init(sim::Sim *sim)
    {
        sim::comp::SSSA::init(sim);
        sim->regPreStep(this);
    }

    void cbPreStep()
    {
        State state = _states[step];
        //DBG(step << " " << state.vel_left << " " << state.vel_right << " " << state.vel_arm);
        robot()->addVelLeft(state.vel_left);
        robot()->addVelRight(state.vel_right);
        robot()->addVelArm(state.vel_arm);

        if (robot()->velLeft() > maxvel)
            robot()->setVelLeft(maxvel);
        if (robot()->velLeft() < minvel)
            robot()->setVelLeft(minvel);

        if (robot()->velRight() > maxvel)
            robot()->setVelRight(maxvel);
        if (robot()->velRight() < minvel)
            robot()->setVelRight(minvel);

        if (robot()->velArm() > maxvel)
            robot()->setVelArm(maxvel);
        if (robot()->velArm() < minvel)
            robot()->setVelArm(minvel);
    }
};

class RobotsManager : public sim::Component {
    sim::Sim *_sim;
    std::vector<SSSA *> sssas;

  public:
    RobotsManager()
    {
    }

    void init(sim::Sim *sim)
    {
        _sim = sim;

        std::list<sim::Component *> comps;
        sim->components(&comps);

        std::list<sim::Component *>::iterator it, it_end;
        SSSA *sssa;

        it = comps.begin();
        it_end = comps.end();
        for (; it != it_end; ++it){
            sssa = dynamic_cast<SSSA *>(*it);

            if (sssa && sssa->robot()){
                sssas.push_back(sssa);
            }
        }

        for (size_t i = 0; i < sssas.size(); i++){
            for (size_t j = i + 1; j < sssas.size(); j++){
                if (!sssas[i]->robot()->connectTo(*sssas[j]->robot())){
                    sssas[j]->robot()->connectTo(*sssas[i]->robot());
                }
            }
        }

        sim->regPostStep(this);
    }

    void cbPostStep()
    {
        //DBG("Step " << step);

        if (step >= STEPS){
            _sim->terminateSimulation();
        }

        step++;
    }
};

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

        setTimeStep(Time::fromMs(10));
        setTimeSubSteps(2);

        setSimulateReal(false);
        pauseSimulation();

        createArena();
        createRobot();
    }

    void initBullet()
    {
#ifdef SIM_HAVE_BULLET
        DBG("Using Bullet");

        sim::bullet::World *w = new sim::bullet::World();
        setWorld(w);
#endif /* SIM_HAVE_BULLET */
    }

    void initODE()
    {
#ifdef SIM_HAVE_ODE
        DBG("Using ODE");

        sim::ode::World *w = new sim::ode::World();

        setWorld(w);

        w->setCFM(0.0001);
        w->setERP(0.8);
        w->setStepType(sim::ode::World::STEP_TYPE_QUICK);
        w->setAutoDisable(0.01, 0.01, 5, 0.);

        w->setContactApprox1(true);
        w->setContactApprox2(true);
        w->setContactBounce(0.1, 0.1);
#endif /* SIM_HAVE_ODE */
    }

    void createArena()
    {
        osg::Vec4 color(0., 0.7, 0.1, 1.);
        sim::Body *c;
        int id;
        sim::World *w = world();

        c = w->createBodyCompound();
        id = c->addBox(Vec3(20., 20., 0.1));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");
        id = c->addBox(Vec3(0.1, 20., 4.), SIM_BODY_DEFAULT_VIS, Vec3(-10, 0., 2.));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");
        id = c->addBox(Vec3(0.1, 20., 4.), SIM_BODY_DEFAULT_VIS, Vec3(10, 0., 2.));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");
        id = c->addBox(Vec3(20., .1, 4.), SIM_BODY_DEFAULT_VIS, Vec3(0., 10, 2.));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");
        id = c->addBox(Vec3(20., .1, 4.), SIM_BODY_DEFAULT_VIS, Vec3(0., -10, 2.));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");

        id = c->addCube(.2, SIM_BODY_DEFAULT_VIS, Vec3(0.5, 0.5, .1));
        c->visBody(id)->setColor(0., .1, .8, 1.);
        c->visBody(id)->setTexture("wood.ppm");
        c->activate();
    }

    void createRobot()
    {
        SSSA *rob;

        rob = new SSSA(0, Vec3(2., 2., .6));
        addComponent(rob);

        rob = new SSSA(1, Vec3(.746, 2., .6));
        addComponent(rob);

        rob = new SSSA(2, Vec3(-.508, 2., .6));
        addComponent(rob);

        rob = new SSSA(3, Vec3(3.254, 2., .6));
        addComponent(rob);

        rob = new SSSA(4, Vec3(4.508, 2., .6));
        addComponent(rob);

        rob = new SSSA(5, Vec3(2., 3.254, .6), Quat(Vec3(0., 0., 1.), M_PI / 2.));
        addComponent(rob);

        rob = new SSSA(6, Vec3(2., 4.508, .6), Quat(Vec3(0., 0., 1.), M_PI / 2.));
        addComponent(rob);

        rob = new SSSA(7, Vec3(2., 0.746, .6), Quat(Vec3(0., 0., 1.), -M_PI / 2.));
        addComponent(rob);

        rob = new SSSA(8, Vec3(2., -.508, .6), Quat(Vec3(0., 0., 1.), -M_PI / 2.));
        addComponent(rob);

        // and as last add component which will connect all robots
        addComponent(new RobotsManager());
    }

};

int main(int argc, char *argv[])
{
    for (int i = 1; i < argc; i++){
        if (strcmp(argv[i], "--ode") == 0){
            use_ode = true;
        }else if (strcmp(argv[i], "--bullet") == 0){
            use_ode = false;
        }
    }

    srand(587165789);

    S s;
    s.run();

    return 0;
}

