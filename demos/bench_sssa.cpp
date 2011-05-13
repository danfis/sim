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

#include <sim/comp/snake2.hpp>
#include "sim/comp/simpleLogger.hpp"
#include "sim/comp/watchdog.hpp"

#include "robot_sssa.hpp"

#include <sim/comp/povray.hpp>
#include <sim/comp/blender.hpp>

using namespace sim::ode;
using sim::Vec3;
using sim::Quat;
using sim::Time;
using namespace std;


const int maxSimulationSteps = 5000;
int usedScenario = 0;
bool use_ode = true;

class CEdge {
    public:
        int from,to;
        CEdge(const int f, const int t):from(f),to(t){}
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
//        setVisWorld(NULL);

       setSimulateReal(false);

        createArena();
        switch(usedScenario) {
            case 0:  createRobot_one_sssa(); break;
            case 1:  createRobot_snake_sssa(); break;
            default: {
                cerr << "You have to choose proper test type!\n";
                exit(0);
                     }
        }
       
//        sim::comp::Watchdog *wc = new sim::comp::Watchdog(NULL,30,"/dev/null");
//      addComponent(wc);

//sim::comp::Povray *pc = new sim::comp::Povray("povray/");
//addComponent(pc);
        
//        sim::comp::Blender *bc = new sim::comp::Blender("blender/");
  //      addComponent(bc);



    }

    void initBullet()
    {
#ifdef SIM_HAVE_BULLET
        DBG("Using Bullet");

        sim::bullet::World *w = new sim::bullet::World();
        w->setMaximumNumOfIterations(maxSimulationSteps);
        setWorld(w);
#endif /* SIM_HAVE_BULLET */
    }

    void initODE()
    {
#ifdef SIM_HAVE_ODE
        DBG("Using ODE");

        sim::ode::World *w = new sim::ode::World();

        setWorld(w);
        w->setCFM(1e-10);
        w->setCFM(0.01);
        w->setERP(0.5);
        w->setStepType(sim::ode::World::STEP_TYPE_QUICK);
        w->setAutoDisable(0.01, 0.01, 5, 0.);

        w->setCFM(0.0001);
        w->setERP(0.8);
        w->setStepType(World::STEP_TYPE_QUICK);
        w->setAutoDisable(0.01, 0.01, 5, 0.);

        w->setContactApprox1(true);
        w->setContactApprox2(true);
        w->setContactBounce(0.1, 0.1);
        w->setMaximumNumOfIterations(maxSimulationSteps);

#endif /* SIM_HAVE_ODE */
    }




    void init()
    {
        sim::Sim::init();

//        pauseSimulation();
    }

    void createArena()
    {
        osg::Vec4 color(0., 0.7, 0.1, 1.);
        sim::Body *c;
        int id;
        sim::ode::World *w = (sim::ode::World *)world();
        const double widthX = 80;
        const double widthY = 20;
        c = w->createBodyCompound();
        id = c->addBox(Vec3(widthX, widthY, 0.1));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");
        id = c->addBox(Vec3(0.1, widthY, 0.5), SIM_BODY_DEFAULT_VIS, Vec3(-widthX/2, 0., .25));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");
        id = c->addBox(Vec3(0.1, widthY, 0.5), SIM_BODY_DEFAULT_VIS, Vec3(widthX/2, 0., .25));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");
        id = c->addBox(Vec3(widthX, .1, 0.3), SIM_BODY_DEFAULT_VIS, Vec3(0., widthY/2, .25));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");
        id = c->addBox(Vec3(widthX, .1, 0.3), SIM_BODY_DEFAULT_VIS, Vec3(0., -widthY/2, .25));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");

        
//      id = c->addCube(.2, SIM_BODY_DEFAULT_VIS, Vec3(1.5, 1.5, .1));
//      c->visBody(id)->setColor(0., .1, .8, 1.);
//      c->visBody(id)->setTexture("wood.ppm");

        c->activate();
        
    }

    void createRobot_one_sssa()
    {
        /*
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
        */

        sim::robot::SSSA *r3;
        r3 = new sim::robot::SSSA(world(),Vec3(0,0,0.6));
        r3->activate();

        r3->setVelLeft(3);
        r3->setVelRight(-3);



    }

    void createRobot_snake_sssa() {
        const double widthX = 1.254;
        const double widthY = 1.254;
        const double widthZ = 1.254;
        const double iwidthZ = widthZ/2+0.01;
        const double initPosX = 5;
        const double initPosY = 0;

        vector<sim::Vec3> positions;
        vector<sim::Quat> rotations;

        // main body
        for(int i=0;i<7;i++) {
            positions.push_back(sim::Vec3(initPosX-widthX*i,initPosY+0*widthY,iwidthZ));
            rotations.push_back(sim::Quat(sim::Vec3(0,0,1),0.));
            cerr << positions.back()[0] << " " << positions.back()[1] << " " << positions.back()[2] << "\n";
        }
        


        /*
        // low legs
        for(int i=0;i<2;i++) {
            positions.push_back(sim::Vec3(-widthX*1,-(i+1)*widthY,iwidthZ));
            rotations.push_back(sim::Quat(sim::Vec3(0,0,1),-M_PI/2));
        }

        for(int i=0;i<2;i++) {
            positions.push_back(sim::Vec3(-widthX*5,-(i+1)*widthY,iwidthZ));
            rotations.push_back(sim::Quat(sim::Vec3(0,0,1),-M_PI/2));
        }
        // upper legs
        for(int i=0;i<2;i++) {
            positions.push_back(sim::Vec3(-widthX*1,(i+1)*widthY,iwidthZ));
            rotations.push_back(sim::Quat(sim::Vec3(0,0,1),M_PI/2));
        }
        for(int i=0;i<2;i++) {
            positions.push_back(sim::Vec3(-widthX*5,(i+1)*widthY,iwidthZ));
            rotations.push_back(sim::Quat(sim::Vec3(0,0,1),M_PI/2));
        }
        */

        vector<sim::robot::SSSA *> robots;

        for(int i=0;i<(int)positions.size();i++) {
            robots.push_back(new sim::robot::SSSA(world(),positions[i],rotations[i])); 
            robots.back()->activate();
            robots.back()->setVelLeft(0.8);
            robots.back()->setVelRight(0.8);
        }
        
//        char name[2000];
//        sprintf(name,"%s.position",_prefix);

//        sim::comp::SimpleLogger *sl = new sim::comp::SimpleLogger(robots[0]->chasis(),name);
//        addComponent(sl);

        vector<CEdge> edges;
        for(int i=0;i<6;i++) {
            edges.push_back(CEdge(i,i+1));
        }        
        /*
        edges.push_back(CEdge(9,5));
        edges.push_back(CEdge(10,9));

        edges.push_back(CEdge(7,1));
        edges.push_back(CEdge(8,7));

        edges.push_back(CEdge(11,1));
        edges.push_back(CEdge(12,11));

        edges.push_back(CEdge(13,5));
        edges.push_back(CEdge(14,13));
        */

        // connect them
        for(int i=0;i<(int)edges.size();i++) {
            DBG("r"<< edges[i].from << "->r"<< edges[i].to << " = " << robots[edges[i].from]->canConnectTo(*robots[edges[i].to]));
            robots[edges[i].from]->connectTo(*robots[edges[i].to]);
            robots[edges[i].from]->setVelArm(0);
        }


        sim::comp::SnakeBody *sbc = new sim::comp::SnakeBody(robots);
        addComponent(sbc);
    }





};

int main(int argc, char *argv[]){

    if (argc < 3) {
        cerr << "usage: " << argv[0] << " <useODE(1/0)> <scenario?>\n";
        cerr << "useOde      0 .. Bullet is used\n";
        cerr << "            1 .. ODE is used\n";
        cerr << "scenario    0 .. one sssa\n";
        cerr << "            1 .. sssa snake going forward\n";
        exit(0);
    }
    
    use_ode = atoi(argv[1]);
    usedScenario = atoi(argv[2]);
    
    
    S s;
    s.run();

    return 0;
}
