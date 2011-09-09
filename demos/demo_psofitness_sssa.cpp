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
#include <string.h>
#include <sim/sim.hpp>
#include <sim/world.hpp>
#include <sim/sensor/camera.hpp>
#include <sim/comp/sssa.hpp>
#include <sim/msg.hpp>
#include "sinController.hpp"
#include "sim/comp/watchdog.hpp"
#include "sim/comp/povray.hpp"



using sim::Vec3;
using sim::Quat;
using sim::Time;
using namespace std;
using sim::comp::SSSA;


bool use_ode = true;
bool use_gui = true;
bool use_wheels = true;
bool use_pause = true;
char *inputFile = NULL;
bool use_povray = false;    // if true, povray component is used to log simulation
char *povrayDir = NULL;
double terminateAtTime = 10; // time for simulation

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

        setTimeStep(Time::fromMs(5));
        setTimeSubSteps(2);

        if (!use_gui) {
            setSimulateReal(false);
        } else {
            if (use_pause) {
                pauseSimulation();
            }
        }

        createArena();
        createRobot_snake_sssa();

        if (use_povray) {
            if (!povrayDir) {
                cerr << "Cannot create povray component!\n";
            } else {
                cerr << "Opening povray component, dir is '" << povrayDir << "'\n";
        		sim::comp::Povray *pc = new sim::comp::Povray(povrayDir);
	        	addComponent(pc);
            }
        }
    }

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

        w->setCFM(0.0001);
        w->setERP(0.8);
        w->setStepType(sim::WorldODE::STEP_TYPE_QUICK);
        w->setAutoDisable(0.01, 0.01, 5, 0.);

        w->setContactApprox1(true);
        w->setContactApprox2(true);
        w->setContactBounce(0.1, 0.1);
#endif /* SIM_HAVE_ODE */
    }

    void createArena()
    {
        const double arenaSize = 28;
        osg::Vec4 color(0., 0.7, 0.1, 1.);
        sim::Body *c;
        int id;
        sim::World *w = world();

        c = w->createBodyCompound();
        id = c->addBox(Vec3(arenaSize, arenaSize, 0.1));
        c->visBody(id)->setColor(color);
//        c->visBody(id)->setTexture("wood.ppm");
        id = c->addBox(Vec3(0.1, arenaSize, 0.5), SIM_BODY_DEFAULT_VIS, Vec3(-arenaSize/2, 0., .25));
        c->visBody(id)->setColor(color);
//        c->visBody(id)->setTexture("wood.ppm");
        id = c->addBox(Vec3(0.1, arenaSize, 0.5), SIM_BODY_DEFAULT_VIS, Vec3(arenaSize/2, 0., .25));
        c->visBody(id)->setColor(color);
//        c->visBody(id)->setTexture("wood.ppm");
        id = c->addBox(Vec3(arenaSize, .1, 0.3), SIM_BODY_DEFAULT_VIS, Vec3(0., arenaSize/2, .25));
        c->visBody(id)->setColor(color);
//        c->visBody(id)->setTexture("wood.ppm");
        id = c->addBox(Vec3(arenaSize, .1, 0.3), SIM_BODY_DEFAULT_VIS, Vec3(0., -arenaSize/2, .25));
        c->visBody(id)->setColor(color);
//        c->visBody(id)->setTexture("wood.ppm");

        c->activate();
    }

    void createRobot_snake_sssa() {

        const double widthX = 1.254; // these constants define dimensions of SSSA robots
        const double widthY = 1.254;
        const double widthZ = 1.254;
        const double iwidthZ = widthZ/2+0.01;
        const double initPosX = 0;
        const double initPosY = 0;

        const int numOfRobots = 2;

        vector<sim::Vec3> positions;
        vector<sim::Quat> rotations;

        // main body
        for(int i=0;i<numOfRobots;i++) {
            positions.push_back(sim::Vec3(initPosX-widthX*i,initPosY+0*widthY,iwidthZ));
            rotations.push_back(sim::Quat(sim::Vec3(0,0,1),0.));
//            cerr << positions.back()[0] << " " << positions.back()[1] << " " << positions.back()[2] << "\n";
        }
        vector<sim::robot::SSSA *> robots;

        // first we create the robots
        for(int i=0;i<(int)positions.size();i++) {
            robots.push_back(new sim::robot::SSSA(world(),use_wheels,positions[i],rotations[i])); 
            robots.back()->activate();
        }
        
        // we define which robots should be connected. This way seems to be quite complicated, but
        // for more complex robots is useful..
        vector<CEdge> edges;
        for(int i=0;i<numOfRobots-1;i++) {
            edges.push_back(CEdge(i,i+1));
        }        

        // connect them
        for(int i=0;i<(int)edges.size();i++) {
//            DBG("r"<< edges[i].from << "->r"<< edges[i].to << " = " << 
//                    robots[edges[i].from]->canConnectTo(*(robots[edges[i].to])));
            robots[edges[i].from]->connectTo(*(robots[edges[i].to]));
            robots[edges[i].from]->setVelArm(0);
        }

        vector< vector<double> > inputs;
        if (inputFile) {
            ifstream ifs(inputFile);
            double amplitude, freq, phase;
            while(ifs) {
                if (ifs >> amplitude >> freq >> phase) {
                    vector<double> tmp;
                    tmp.push_back(amplitude);
                    tmp.push_back(freq);
                    tmp.push_back(phase);
                    inputs.push_back(tmp);
                    tmp.clear();
                }
            }
            ifs.close();
            cerr << "Loaded " << inputs.size() << " inputs for organism\n";
            if (numOfRobots != (int)inputs.size()) {
                cerr << "Number of inputs is not same as number of used robots!. robots=" << numOfRobots << "\n";
                exit(0);
            }

            cerr << "Creating sinController form given input file: " << inputFile << "\n";
            for(int i=0;i<(int)inputs.size();i++) {
                SinController *sc = new SinController(robots[i], inputs[i][0], inputs[i][1], inputs[i][2],i);
                addComponent(sc);
            }
        } else {
            cerr << "Creating sinController randomly\n";
            // and we create the components, which will controll the robots.
            // see sinController.hpp for details.
            for(int i=0;i<(int)robots.size();i++) {
                const double r = 0.3*rand()/RAND_MAX; 
                SinController *sinc = new SinController(robots[i],0.8+r,1,i*M_PI/4/robots.size(),i);
                addComponent(sinc);
            }
        }
        
        // this watch dog terminates simulation after predefined time and report position of the given bosy to file
        if (inputFile) {
            char name[2000];
            sprintf(name,"%s.result",inputFile);
            cerr << "Result='" << name << "'\n";
            remove(name);
            sim::comp::Watchdog *wc = new sim::comp::Watchdog(robots[0]->chasis(),terminateAtTime,inputFile,true);
            addComponent(wc);
        }
    }


};

int main(int argc, char *argv[])
{
    for (int i = 1; i < argc; i++){
        if (strcmp(argv[i],"--input") == 0) {
            inputFile = argv[i+1];
            i++;
        } else if (strcmp(argv[i], "--ode") == 0){
            use_ode = true;
        }else if (strcmp(argv[i], "--bullet") == 0){
            use_ode = false;
        }else if (strcmp(argv[i], "--nogui") == 0){
            use_gui = false;
        }else if (strcmp(argv[i], "--nowheels") == 0){
            use_wheels = false;
        }else if (strcmp(argv[i], "--nopause") == 0){
            use_pause = false;
        }else if (strcmp(argv[i], "--withpovray") == 0){
            use_povray = true;
            povrayDir = argv[i+1];
            i++;
        }else if (strcmp(argv[i], "--time") == 0){
            terminateAtTime = atof(argv[i+1]);
            i++;
        }
    }

    S s;
    s.run();

    return 0;
}
