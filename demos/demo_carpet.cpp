#include <iostream>
#include <sim/sim.hpp>
#include <sim/world.hpp>
#include <sim/msg.hpp>
#include <sim/sensor/camera.hpp>
#include <sim/comp/snake2.hpp>
#include <sim/comp/povray.hpp>
#include <sim/comp/blender.hpp>
#include "sim/comp/snake.hpp"
#include "sim/comp/frequency.hpp"
#include "sim/comp/watchdog.hpp"
#include <sim/comp/sssa.hpp>

using sim::Vec3;
using sim::Quat;
using sim::Time;
using namespace std;

class CEdge {
    public:
        int from,to;
        CEdge(const int f, const int t):from(f),to(t){}
};


class MovingCarpet : public sim::Sim {
    
    vector<sim::Joint *> joints;

  public:
    MovingCarpet(const char *paramFile)
        : Sim()
    {
        sim::WorldODE *w = sim::WorldFactory::ODE();

        setTimeStep(Time::fromMs(5));
        setTimeSubSteps(2);
        setSimulateReal(false);

        setWorld(w);

        w->setCFM(0.0001);
        w->setERP(0.8);
        w->setStepType(sim::WorldODE::STEP_TYPE_QUICK);
        w->setAutoDisable(0.01, 0.01, 5, 0.);

        w->setContactApprox1(true);
        w->setContactApprox2(true);
        w->setContactBounce(0.1, 0.1);

        createArena();
//        createRobot();
//        createCarpet(paramFile);
        createCarpetHinge2(paramFile);

//		sim::comp::Povray *pc = new sim::comp::Povray("povray/");
//		addComponent(pc);
        
        sim::comp::Blender *bc = new sim::comp::Blender("blender-carpet/");
        addComponent(bc);

    }

    void init()
    {
        sim::Sim::init();

//        pauseSimulation();
    }

    void createCarpet(const char *paramFile) {
        const int numX = 5;
        const int numY = 5;
        const double size = 1;
        const double dist = size*1.5;
        const double weight = 0.3;

        sim::Body *b;
        vector< sim::Body *> tmpb(vector<sim::Body *>(numY,b));
        vector< vector<sim::Body *> > bodies(numX,tmpb);
        
        sim::Joint *jo;

        for(int i=0;i<numX;i++) {
            for(int j=0;j<numY;j++) {
                b = world()->createBodyBox(Vec3(size,size,size),weight);
                b->visBody()->setColor(0.3,1.0*i/numX,1.0*j/numY,1);
                b->setPos(i*dist,j*dist,size/1+ size/10.0);
                b->activate();
                bodies[i][j] = b;
            }
        }
    
        // joint per collumns
        for(int i=0;i<numX;i++) {
            for(int j=0;j<numY-1;j++) {
                jo = world()->createJointHinge(bodies[i][j],bodies[i][j+1],bodies[i][j]->pos(),Vec3(1,0,0));
                jo->activate();
                joints.push_back(jo);
            }
        }

        // row joints
        for(int j=0;j<numY;j++) {
            for(int i=0;i<numX-1;i++) {
                jo = world()->createJointHinge(bodies[i][j],bodies[i+1][j],bodies[i][j]->pos(),Vec3(0,1,0));
                jo->activate();
                //jo->setParamFMax(10);
                //jo->setParamVel(1);
                joints.push_back(jo);
            }
        }

		//sim::comp::Snake *sc = new sim::comp::Snake(joints);
		//addComponent(sc);
		//regMessage(sc,sim::MessageKeyPressed::Type);
        
       /* 
        for(int i=0;i<joints.size();i++) {        
            const double amp = rand()%4;
            const double freq = rand()%4;
            const double phase = rand()%10;
            sim::comp::Frequency *fc1 = new sim::comp::Frequency(joints[i],amp,freq,phase,0);
            addComponent(fc1);
        }
        */
        cerr << "NUmber of joints: " << joints.size()  << "\n";


        // make frequency controller. the parameters for the controllers are loaded from 'paramFile'

        std::ifstream ifs(paramFile);
        std::cerr << "Loading parametres from " << paramFile << "\n";
        if (ifs) {
            for(int i=0;i<(int)joints.size();i++) {
                
                for(int j=0;j<2;j++) {
                    double amp, freq, phase;
                    if (ifs >> amp >> freq >> phase) {
						if (j == 0) {
                            cerr << "Loading freq parames for joint " << i << "=" << amp << "," << freq << "," << phase << "\n";
                       		sim::comp::Frequency *fc1 = new sim::comp::Frequency(joints[i],amp,freq,phase,j);
                        	addComponent(fc1);
						}
                    } else {
                        std::cerr << "Cannot load frequency parametres for joint " << i << ",type " << j << "!\n";
                        exit(0);
                    }
                }
            }
        } else {
            std::cerr << "Cannot load parametres of frequency controllers from " << paramFile << "!\n";
            exit(0);
        }

        sim::comp::Watchdog *wc = new sim::comp::Watchdog(bodies[numX/2][numY/2],10,paramFile,true);
        addComponent(wc);

    }


    void createCarpetHinge2(const char *paramFile) {
        const int numX = 50;
        const int numY = 10;
        const double size = 0.3;
        const double dist = size*1.5;
        const double weight = 0.3;

        sim::Body *b;
        vector< sim::Body *> tmpb(vector<sim::Body *>(numY,b));
        vector< vector<sim::Body *> > bodies(numX,tmpb);
        
        sim::Joint *jo;

        for(int i=0;i<numX;i++) {
            for(int j=0;j<numY;j++) {
                b = world()->createBodyBox(Vec3(size,size,size),weight);
                b->visBody()->setColor(0.9*(i+j)/(0.5*numX+numY),1.0*i/numX,1*j/numY,1);
                b->setPos(i*dist,j*dist,1+size/1+ size/10.0);
                b->activate();
                bodies[i][j] = b;
            }
        }

        b = world()->createBodyBox(Vec3(2,2,1),0);
        b->visBody()->setColor(1,1,1,1);
        b->setPos(5,5,0.1);
        b->activate();
    
        // joint per collumns
        for(int i=0;i<numX;i++) {
            for(int j=0;j<numY-1;j++) {
                Vec3 anchor(bodies[i][j]->pos());
                anchor[1]+=dist/2;
                jo = world()->createJointHinge2(bodies[i][j],bodies[i][j+1],anchor,Vec3(1,0,0),Vec3(0,1,0));
                jo->activate();
                joints.push_back(jo);
                jo->setParamFMax2(150);
                jo->setParamVel2(-10.0*(1.0*i+j)/(1.0*numX+numY));
            }
        }

        // row joints
        for(int j=0;j<numY;j++) {
            for(int i=0;i<numX-1;i++) {
                Vec3 anchor(bodies[i][j]->pos());
                anchor[0]+=dist/2;
                jo = world()->createJointHinge2(bodies[i][j],bodies[i+1][j],anchor,Vec3(0,1,0),Vec3(1,0,0));
                jo->activate();
                joints.push_back(jo);
//                jo->setParamFMax2(50);
  //              jo->setParamVel2(10+rand()%5);
            }
        }

		//sim::comp::Snake *sc = new sim::comp::Snake(joints);
		//addComponent(sc);
		//regMessage(sc,sim::MessageKeyPressed::Type);
        
        for(int i=0;i<(int)joints.size();i++) {        
            double amp = rand()%4+2;
            double freq = rand()%10+10;
            double phase = rand()%10;
//            sim::comp::Frequency *fc1 = new sim::comp::Frequency(joints[i],amp,freq,phase,0);
  //          addComponent(fc1);

            amp = rand()%4;
            freq = rand()%4;
            phase = rand()%10;

      //      sim::comp::Frequency *fc2 = new sim::comp::Frequency(joints[i],amp,freq,phase,1);
        //    addComponent(fc2);
        }
        cerr << "NUmber of joints: " << joints.size()  << "\n";

/*
        // make frequency controller. the parameters for the controllers are loaded from 'paramFile'

        std::ifstream ifs(paramFile);
        std::cerr << "Loading parametres from " << paramFile << "\n";
        if (ifs) {
            for(int i=0;i<(int)joints.size();i++) {
                
                for(int j=0;j<2;j++) {
                    double amp, freq, phase;
                    if (ifs >> amp >> freq >> phase) {
						if (j == 0) {
                            cerr << "Loading freq parames for joint " << i << "=" << amp << "," << freq << "," << phase << "\n";
                       		sim::comp::Frequency *fc1 = new sim::comp::Frequency(joints[i],amp,freq,phase,j);
                        	addComponent(fc1);
						}
                    } else {
                        std::cerr << "Cannot load frequency parametres for joint " << i << ",type " << j << "!\n";
                        exit(0);
                    }
                }
            }
        } else {
            std::cerr << "Cannot load parametres of frequency controllers from " << paramFile << "!\n";
            exit(0);
        }

        sim::comp::Watchdog *wc = new sim::comp::Watchdog(bodies[numX/2][numY/2],10,paramFile);
        addComponent(wc);
*/
    }


    void createArena()
    {
        osg::Vec4 color(0., 0.7, 0.1, 1.);
        sim::Body *c;
        int id;
        sim::WorldODE *w = (sim::WorldODE *)world();

        c = w->createBodyCompound();
        id = c->addBox(Vec3(30., 30., 0.1));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");
        c->activate();
    }

    void createRobot()
    {
        const double widthX = 1.254;
        const double widthY = 1.254;
        const double widthZ = 1.254;
        const double iwidthZ = widthZ/2+0.01;

        vector<sim::Vec3> positions;
        vector<sim::Quat> rotations;

        // main body
        for(int i=0;i<7;i++) {
            positions.push_back(sim::Vec3(-widthX*i,0*widthY,iwidthZ));
            rotations.push_back(sim::Quat(sim::Vec3(0,0,1),0.));
            cerr << positions.back()[0] << " " << positions.back()[1] << " " << positions.back()[2] << "\n";
        }
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

        vector<sim::robot::SSSA *> robots;

        for(int i=0;i<(int)positions.size();i++) {
            robots.push_back(new sim::robot::SSSA(world(),positions[i],rotations[i])); 
            robots.back()->activate();
        }

        vector<CEdge> edges;
        for(int i=0;i<6;i++) {
            edges.push_back(CEdge(i,i+1));
        }        
        edges.push_back(CEdge(9,5));
        edges.push_back(CEdge(10,9));

        edges.push_back(CEdge(7,1));
        edges.push_back(CEdge(8,7));

        edges.push_back(CEdge(11,1));
        edges.push_back(CEdge(12,11));

        edges.push_back(CEdge(13,5));
        edges.push_back(CEdge(14,13));

        // connect them
        for(int i=0;i<(int)edges.size();i++) {
            DBG("r"<< edges[i].from << "->r"<< edges[i].to << " = " << robots[edges[i].from]->canConnectTo(*robots[edges[i].to]));
            robots[edges[i].from]->connectTo(*robots[edges[i].to]);
            robots[edges[i].from]->setVelArm(0);
        }


        sim::comp::SnakeBody *sbc = new sim::comp::SnakeBody(robots);
        addComponent(sbc);
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
    }

};

int main(int argc, char *argv[])
{
    if (argc < 2) {
        cerr << "usage: " << argv[0] << " <paramFile>\n";
        exit(0);
    }
    MovingCarpet m(argv[1]);

    m.run();

    return 0;
}
