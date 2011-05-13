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
#include <sim/msg.hpp>
#include "sinController.hpp"


using sim::Vec3;
using sim::Quat;
using sim::Time;
using namespace std;
using sim::comp::SSSA;


bool use_ode = true;
bool use_gui = true;


class CEdge {
    public:
        int from,to;
        CEdge(const int f, const int t):from(f),to(t){}
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

        sim->regMessage(this, sim::MessageKeyPressed::Type);
    }

    void processMessage(const sim::Message &msg)
    {
    }

  protected:
};

class S : public sim::Sim {
  public:
    S()
        : Sim(0,0,use_gui) 
    {
        if (use_ode){
            initODE();
        }else{
            initBullet();
        }

        setTimeStep(Time::fromMs(10));
        setTimeSubSteps(2);

        if (!use_gui) {
            setSimulateReal(true);
        } else {
            pauseSimulation();
        }

        createArena();
        createRobot_snake_sssa();
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
        const double arenaSize = 18;
        osg::Vec4 color(0., 0.7, 0.1, 1.);
        sim::Body *c;
        int id;
        sim::World *w = world();

        c = w->createBodyCompound();
        id = c->addBox(Vec3(arenaSize, arenaSize, 0.1));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");
        id = c->addBox(Vec3(0.1, arenaSize, 0.5), SIM_BODY_DEFAULT_VIS, Vec3(-arenaSize/2, 0., .25));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");
        id = c->addBox(Vec3(0.1, arenaSize, 0.5), SIM_BODY_DEFAULT_VIS, Vec3(arenaSize/2, 0., .25));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");
        id = c->addBox(Vec3(arenaSize, .1, 0.3), SIM_BODY_DEFAULT_VIS, Vec3(0., arenaSize/2, .25));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");
        id = c->addBox(Vec3(arenaSize, .1, 0.3), SIM_BODY_DEFAULT_VIS, Vec3(0., -arenaSize/2, .25));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");

//        id = c->addCube(.2, SIM_BODY_DEFAULT_VIS, Vec3(0.5, 0.5, .1));
 //       c->visBody(id)->setColor(0., .1, .8, 1.);
 //       c->visBody(id)->setTexture("wood.ppm");
        c->activate();
    }

    void createSimpleOrganism()
    {
        SSSA *rob;

        rob = new SSSA(Vec3(2., 2., .6));
        addComponent(rob);

        rob = new SSSA(Vec3(.746, 2., .6));
        addComponent(rob);

        rob = new SSSA(Vec3(2., 3.254, .6), Quat(Vec3(0., 0., 1.), M_PI / 2.));
        addComponent(rob);

        rob = new SSSA(Vec3(2., 0.746, .6), Quat(Vec3(0., 0., 1.), -M_PI / 2.));
        addComponent(rob);

        // and as last add component which will connect all robots
        addComponent(new RobotsManager());
    }


    void createRobot_snake_sssa() {
        const double widthX = 1.254;
        const double widthY = 1.254;
        const double widthZ = 1.254;
        const double iwidthZ = widthZ/2+0.01;
        const double initPosX = 5;
        const double initPosY = 0;

        const int numOfRobots = 4;

        vector<sim::Vec3> positions;
        vector<sim::Quat> rotations;

        // main body
        for(int i=0;i<numOfRobots;i++) {
            positions.push_back(sim::Vec3(initPosX-widthX*i,initPosY+0*widthY,iwidthZ));
            rotations.push_back(sim::Quat(sim::Vec3(0,0,1),0.));
            cerr << positions.back()[0] << " " << positions.back()[1] << " " << positions.back()[2] << "\n";
        }
        vector<sim::robot::SSSA *> robots;

        for(int i=0;i<(int)positions.size();i++) {
        cerr << "A*";
            robots.push_back(new sim::robot::SSSA(world(),positions[i],rotations[i])); 
        cerr << "B*";
            robots.back()->activate();
        cerr << "C*";
//            robots.back()->setVelLeft(0.8);
//            robots.back()->setVelRight(0.8);
        }
        
//        char name[2000];
//        sprintf(name,"%s.position",_prefix);

//        sim::comp::SimpleLogger *sl = new sim::comp::SimpleLogger(robots[0]->chasis(),name);
//        addComponent(sl);

        vector<CEdge> edges;
        for(int i=0;i<numOfRobots-1;i++) {
            edges.push_back(CEdge(i,i+1));
        }        

        cerr << "1*";
        // connect them
        for(int i=0;i<(int)edges.size();i++) {
            cerr << "2*";
            DBG("r"<< edges[i].from << "->r"<< edges[i].to << " = " << 
                    robots[edges[i].from]->canConnectTo(*(robots[edges[i].to])));
            cerr << "3*";
            robots[edges[i].from]->connectTo(*(robots[edges[i].to]));
            cerr << "4*";

            robots[edges[i].from]->setVelArm(0);
            cerr << "5*";
        }

        cerr << "end\n";

//        sim::comp::SnakeBody *sbc = new sim::comp::SnakeBody(robots);
  //      addComponent(sbc);

        SinController *sinc = new SinController(robots[0],1,2,0);
        addComponent(sinc);
    }





};

int main(int argc, char *argv[])
{
    for (int i = 1; i < argc; i++){
        if (strcmp(argv[i], "--ode") == 0){
            use_ode = true;
        }else if (strcmp(argv[i], "--bullet") == 0){
            use_ode = false;
        }else if (strcmp(argv[i], "--nogui") == 0){
            use_gui = false;
        }
    }

    S s;
    s.run();

    return 0;
}
