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
#include <sim/world.hpp>
#include <sim/sensor/camera.hpp>
#include <sim/comp/sssa.hpp>
#include <sim/comp/rserver.hpp>
#include <sim/msg.hpp>

using sim::Vec3;
using sim::Quat;
using sim::Time;
using namespace std;

bool use_ode = true;

class SSSA;

SSSA *active = 0;

class SSSA : public sim::comp::SSSA {
    uint16_t _id;
    sim::Sim *_sim;

  public:
    SSSA(uint16_t id, const Vec3 &pos, const Quat &rot = Quat(0., 0., 0., 1.))
        : sim::comp::SSSA(pos, rot), _id(id)
    {
    }

    void init(sim::Sim *sim)
    {
        sim::comp::SSSA::init(sim);
        sim->regMessage(this, sim::MessageKeyPressed::Type);
        DBG("");
        sim->regMessage(this, sim::comp::RMessageInPing::Type);

        _sim = sim;
    }

    void processMessage(const sim::Message &msg)
    {
        sim::comp::RMessageIn *rmsg;

        if (msg.type() == sim::MessageKeyPressed::Type){
            if (active == this)
                _keyPressedMsg((const sim::MessageKeyPressed &)msg);
        }else if (msg.type() == sim::comp::RMessageInPing::Type){
            rmsg = (sim::comp::RMessageIn *)&msg;
            if (rmsg->msgID() == _id){
                _sim->sendMessage(new sim::comp::RMessageOutPong(rmsg->msgID()));
                DBG("ID: " << rmsg->msgID() << ", type: " << (int)rmsg->msgType());
            }
        }
    }

    void _keyPressedMsg(const sim::MessageKeyPressed &msg)
    {
        int key = msg.key();

        //DBG("Component: " << this << " - key pressed: " << msg.key());

        if (key == 'h'){
            _robot->addVelLeft(0.1);
        }else if (key == 'j'){
            _robot->addVelLeft(-0.1);
        }else if (key == 'k'){
            _robot->addVelRight(0.1);
        }else if (key == 'l'){
            _robot->addVelRight(-0.1);

        }else if (key == 'n'){
            _robot->addVelArm(0.1);
        }else if (key == 'm'){
            _robot->addVelArm(-0.1);
        }else if (key == ','){
            _robot->fixArm();
        }else if (key == '.'){
            _robot->unfixArm();
        }else if (key == 'v'){
            _robot->reachArmAngle(M_PI / 4.);
        }else if (key == 'b'){
            _robot->reachArmAngle(-M_PI / 4.);
        }
        DBG("Velocity: " << _robot->velLeft() << " " << _robot->velRight() << " " << _robot->velArm());
    }
};

class RobotsManager : public sim::Component {
    std::vector<SSSA *> sssas;
    size_t active_id;

  public:
    RobotsManager()
        : active_id(0)
    {
    }

    void init(sim::Sim *sim)
    {
        std::list<sim::Component *> comps;
        sim->components(&comps);

        std::list<sim::Component *>::iterator it, it_end;
        SSSA *sssa;

        it = comps.begin();
        it_end = comps.end();
        for (; it != it_end; ++it){
            sssa = dynamic_cast<SSSA *>(*it);

            if (sssa && sssa->robot()){
                DBG(sssa << " " << sssa->robot());
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

        _activateRobot(0);

        sim->regMessage(this, sim::MessageKeyPressed::Type);

        sim->regMessage(this, sim::comp::RMessageIn::Type);
    }

    void processMessage(const sim::Message &msg)
    {
        sim::comp::RMessageIn *rmsg;

        DBG("type: " << msg.type());
        if (msg.type() == sim::MessageKeyPressed::Type){
            const sim::MessageKeyPressed &m = (const sim::MessageKeyPressed &)msg;
            if (m.key() == 'u'){
                _activateRobot(active_id + 1);
            }else if (m.key() == 'i'){
                _activateRobot(active_id - 1);
            }
        }else if (msg.type() == sim::comp::RMessageIn::Type){
            rmsg = (sim::comp::RMessageIn *)&msg;
            DBG("ID: " << rmsg->msgID() << ", type: " << (int)rmsg->msgType());
        }
    }

  protected:
    void _activateRobot(size_t i)
    {
        std::list<sim::VisBody *> vis;
        std::list<sim::VisBody *>::iterator it, it_end;

        if (i < 0){
            i = sssas.size() - 1;
        }else if (i >= sssas.size()){
            i = 0;
        }

        active_id = i;

        if (active){
            active->robot()->chasis()->visBodyAll(&vis);

            it = vis.begin();
            it_end = vis.end();
            for (; it != it_end; ++it){
                (*it)->setColor(0., 0.1, 0.7, 0.6);
            }

            vis.clear();
        }

        active = sssas[i];

        vis.clear();
        active->robot()->chasis()->visBodyAll(&vis);

        it = vis.begin();
        it_end = vis.end();
        for (; it != it_end; ++it){
            (*it)->setColor(0.7, 0.1, 0., 0.6);
        }
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

        pauseSimulation();

        createArena();
        createRobot();

        DBG("");
        addComponent(new sim::comp::RServer("any", 9876));
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
        SSSA *rob;

        rob = new SSSA(10, Vec3(2., 2., .6));
        addComponent(rob);

        rob = new SSSA(11, Vec3(.746, 2., .6));
        addComponent(rob);

        rob = new SSSA(12, Vec3(2., 3.254, .6), Quat(Vec3(0., 0., 1.), M_PI / 2.));
        addComponent(rob);

        rob = new SSSA(13, Vec3(2., 0.746, .6), Quat(Vec3(0., 0., 1.), -M_PI / 2.));
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

    S s;
    s.run();

    return 0;
}

