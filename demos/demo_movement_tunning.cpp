#include <iostream>
#include <unistd.h>
#include <fstream>
#include <sstream>
#include <vector>

#include "sim/ode/world.hpp"
#include "sim/sim.hpp"
#include "msg.hpp"
#include "meshes/plane.h"
#include "sim/comp/povray.hpp"
#include "sim/comp/snake.hpp"
#include "sim/comp/frequency.hpp"
#include "sim/comp/watchdog.hpp"

using namespace sim::ode;
using sim::Vec3;
using namespace std;

class SimMovementTunning : public sim::Sim {
	vector<sim::Joint *> snakeJoints;
  public:
    SimMovementTunning	(const char *parametersFile)
        : Sim()
    {
        setTimeStep(sim::Time::fromMs(20));
        setTimeSubSteps(2);
        World *w = new World();
        sim::Body *b;
        setWorld(w);
//        w->setCFM(1e-3);
//      w->setCFM(0.01);
        w->setERP(0.5);
        //w->setContactSoftCFM(0.0000001);
        //w->setContactApprox1(false);
        //w->setContactApprox2(false);
        //w->setContactBounce(0.1, 0.1);
    
		createPlane();
		createSnake(parametersFile);

//		sim::comp::Povray *pc = new sim::comp::Povray("povray/");
//		addComponent(pc);

		sim::comp::Snake *sc = new sim::comp::Snake(snakeJoints);
		addComponent(sc);
		regMessage(sc,sim::MessageKeyPressed::Type);


    }

    void init()
    {
        sim::Sim::init();
        for (size_t i = 0; i < 5; i++){
            visWorld()->step();
            std::cerr << i << "\r";
            usleep(1000);
        }
        std::cerr << std::endl;

        timeRealRestart();
    }

  protected:


	void createSnake(const char *paramFile) {

		const double posx = 0;
		const double posy = 0;
		const double posz = 0.7;
		const double width = 1;
		const double gap = width*1.5;
		const double mass = 0.5;


        vector<sim::Joint *> joints;

        sim::Body *b1 = world()->createBodyCube(width,mass);
		b1->setPos(sim::Vec3(posx,posy,posz));
		b1->visBody()->setColor(1,1,1,1);

        sim::Body *b2 = world()->createBodyCube(width,mass);
		b2->setPos(sim::Vec3(posx+1*gap,posy,posz));
		b2->visBody()->setColor(1,0,1,1);

        sim::Body *b3 = world()->createBodyCube(width,mass);
		b3->setPos(sim::Vec3(posx+2*gap,posy,posz));
		b3->visBody()->setColor(0.15,0.7,0.6,1);

        sim::Body *b4 = world()->createBodyCube(width,mass);
		b4->setPos(sim::Vec3(posx+3*gap,posy,posz));
		b4->visBody()->setColor(0.2,0.3,0.11,1);

        sim::Body *b5 = world()->createBodyCube(width,mass);
		b5->setPos(sim::Vec3(posx+4*gap,posy,posz));
		b5->visBody()->setColor(0.5,0,0.2,1);

        joints.push_back(world()->createJointHinge2(b1,b2,b1->pos(),sim::Vec3(0,1,0),sim::Vec3(1,0,0)));
		joints.push_back(world()->createJointHinge2(b2,b3,b2->pos(),sim::Vec3(0,1,0),sim::Vec3(1,0,0)));
		joints.push_back(world()->createJointHinge2(b3,b4,b3->pos(),sim::Vec3(0,1,0),sim::Vec3(1,0,0)));
		joints.push_back(world()->createJointHinge2(b4,b5,b4->pos(),sim::Vec3(0,1,0),sim::Vec3(1,0,0)));


        b1->activate();
		b2->activate();
		b3->activate();
		b4->activate();
		b5->activate();

        sim::comp::Watchdog *wc = new sim::comp::Watchdog(b3,10,paramFile);
        addComponent(wc);

        const double angleMin1 = -45*M_PI/180.0;
        const double angleMax1 =  45*M_PI/180.0;
        const double angleMin2 = -45*M_PI/180.0;
        const double angleMax2 =  45*M_PI/180.0;
        for(int i=0;i<(int)joints.size();i++) {
            joints[i]->setParamLimitLoHi(angleMin1,angleMax1);
            joints[i]->setParamLimitLoHi2(angleMin2,angleMax2);
            joints[i]->activate();
            snakeJoints.push_back(joints[i]);
        }

		
        // make frequency controller. the parameters for the controllers are loaded from 'paramFile'

        std::ifstream ifs(paramFile);
        std::cerr << "Loading parametres from " << paramFile << "\n";
        if (ifs) {
            for(int i=0;i<(int)joints.size();i++) {

                for(int j=0;j<2;j++) {
                    double amp, freq, phase;
                    if (ifs >> amp >> freq >> phase) {
						if (1 || j == 0 || i==2) {
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



	}

	void createPlane()
    {
        sim::Body *obj;

        obj = world()->createBodyTriMesh(plane10_verts,plane10_verts_len,plane10_ids,plane10_ids_len,0);
		obj->setPos(0,0,0);
        obj->visBody()->setColor(0.7, 1, 0.2, 1.);
        obj->activate();
    }
	

};


int main(const int argc, char **argv) {
	if (argc < 2) {
        std::cerr << "usage: " << argv[0] << " <paramFile>\n";
        std::cerr << "paramFile    ..  contains parameters for frequency controller fo 'snake' robot\n";
        exit(0);
    }
    
    SimMovementTunning s(argv[1]);
	s.run();

    return 0;
}



