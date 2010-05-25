

#include <iostream>
#include <unistd.h>
#include <fstream>
#include <sstream>

#include "sim/ode/world.hpp"
#include "sim/sim.hpp"
#include "msg.hpp"
//#include "meshes/ardrone.h"
#include "meshes/jezek.h"
#include "meshes/plane.h"


int counter = 0;
#define intro counter++; if (id == -1) std::cerr << counter << " :"<< __FUNCTION__ << "\n"; if (id != counter) return;


using namespace sim::ode;
using sim::Vec3;
using namespace std;

class SimTest1 : public sim::Sim {
  public:
    SimTest1()
        : Sim()
    {
        setTimeStep(sim::Time::fromMs(10));
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
    
        /*
        b = w->createBodyBox(sim::Vec3(20., 20., 1.), 0.);
        b->setPos(0., 0., -10.);
        b->activate();
        */

        b = w->createBodySphere(1.3, 2.);
        b->visBody()->setColor(osg::Vec4(1, 0, 0, 0.3));
        b->setPos(5., 0, 0);
        b->activate();

        b = w->createBodySphere(1.3, 2.);
        b->visBody()->setColor(osg::Vec4(0, 1, 0, 0.3));
        b->setPos(0, 5, 0);
        b->activate();

        b = w->createBodySphere(1.3, 2.);
        b->visBody()->setColor(osg::Vec4(0, 0, 1, 0.3));
        b->setPos(0, 0, 5.);
        b->activate();



/*
        b = w->createBodyCube(1., 1.);
        b->visBody()->setColor(osg::Vec4(0., 0., 0., 1.));
        b->setPos(-3., 0.2, 3.);
        b->activate();

        b = w->createBodyCylinderZ(.5, 1., 3.);
        b->visBody()->setTexture("wood.ppm");
        b->visBody()->setColor(1., 0., 0., 1.);
        b->setPos(3., 0., 0.);
        b->activate();

        b = w->createBodyCylinderX(.5, 1., 3.);
        b->setPos(0., 3., 0.);
        b->activate();

        b = w->createBodyCylinderY(.5, 1., 3.);
        b->setPos(0., -3., 0.);
        b->activate();

        createFixed();
        createHinge();
        createHingeLim();
        createHinge2();
        createHinge2Lim();
        createRobot();
        createRobotMove();
        //createBunny();
*/


/*		int *indices = NULL;
		Vec3 *vertices = NULL;
		int indicesSize = 0;
		int verticesSize = 0;
//		const char *rawFile = "trmesh2.raw";
		const char *rawFile = "mar.raw";
		vector<Vec3> points(loadTriangles(rawFile,&indices, &vertices, indicesSize, verticesSize));
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
        unsigned int ind[] = { 0, 4, 1,
                               1, 5, 2,
                               0, 3, 4,
                               1, 4, 5,
                               3, 7, 4,
                               4, 8, 5,
                               3, 6, 7,
                               4, 7, 8 };
        b = w->createBodyTriMesh(verts, 9, ind, 24, 0.);
        b->setPos(0., 0., -5.3);
        b->activate();
        
		/*
		b = w->createBodyTriMesh(vertices,verticesSize,(const unsigned int*)indices,indicesSize,0.);
        b->visBody()->setColor(0.9, 0.6, 0.3, 1.);
		*/
        /*
        b = w->createBodyBox(Vec3(20, 20, 2), 0.);
        b->setPos(0., 0., -10);
        b->activate();
        */
    }

    void init()
    {
        sim::Sim::init();
        for (size_t i = 0; i < 300; i++){
            visWorld()->step();
            std::cerr << i << "\r";
            usleep(10000);
        }
        std::cerr << std::endl;

        timeRealRestart();
    }

  protected:
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
        ((JointHinge *)j)->setParamLimitLoHi(-M_PI / 2., 0.);

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

        ((JointHinge2 *)j)->setParamLimitLoHi(-M_PI / 4., M_PI / 4.);

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
        sim::ActuatorWheelCylinderX *w[4];
        size_t i;

        chasis = world()->createBodyBox(sim::Vec3(.6, 1., 0.4), 1.);
        for (i = 0; i < 4; i++){
            w[i] = world()->createActuatorWheelCylinderX(0.2, 0.2, 1.);
        }

        chasis->setPos(pos);
        w[0]->setPos(pos.x() + 0.415, pos.y() + 0.4, pos.z() - 0.2);
        w[1]->setPos(pos.x() - 0.415, pos.y() + 0.4, pos.z() - 0.2);
        w[2]->setPos(pos.x() + 0.415, pos.y() - 0.4, pos.z() - 0.2);
        w[3]->setPos(pos.x() - 0.415, pos.y() - 0.4, pos.z() - 0.2);

        for (i = 0; i < 4; i++){
            w[i]->connectToChasis(chasis);
            ((ActuatorWheelCylinderX *)w[i])->joint()->setParamLimitLoHi(-0.0001, 0.0001);
        }

        chasis->activate();
        for (i = 0; i < 4; i++){
            w[i]->activate();
        }

        chasis->visBody()->setText("Robot", 1., osg::Vec4(0.5, 0.6, 0.3, 1.));
    }

    void createRobotMove()
    {
        sim::Vec3 pos(10., -10., -4.9);
        sim::Body *chasis;
        sim::ActuatorWheelCylinderX *w[4];
        size_t i;

        chasis = world()->createBodyBox(sim::Vec3(.6, 1., 0.4), 1.);
        for (i = 0; i < 4; i++){
            w[i] = world()->createActuatorWheelCylinderX(0.2, 0.2, 1.);
        }

        chasis->setPos(pos);
        w[0]->setPos(pos.x() + 0.415, pos.y() + 0.4, pos.z() - 0.2);
        w[1]->setPos(pos.x() - 0.415, pos.y() + 0.4, pos.z() - 0.2);
        w[2]->setPos(pos.x() + 0.415, pos.y() - 0.4, pos.z() - 0.2);
        w[3]->setPos(pos.x() - 0.415, pos.y() - 0.4, pos.z() - 0.2);

        for (i = 0; i < 4; i++){
            w[i]->connectToChasis(chasis);
            ((ActuatorWheelCylinderX *)w[i])->joint()->setParamLimitLoHi(-0.0001, 0.0001);
        }

        ((ActuatorWheelCylinderX *)w[0])->joint()->setParamVel2(5.);
        ((ActuatorWheelCylinderX *)w[0])->joint()->setParamFMax2(10.);
        ((ActuatorWheelCylinderX *)w[1])->joint()->setParamVel2(5.);
        ((ActuatorWheelCylinderX *)w[1])->joint()->setParamFMax2(10.);

        chasis->activate();
        for (i = 0; i < 4; i++){
            w[i]->activate();
        }

        chasis->visBody()->setText("Robot move", 1., osg::Vec4(0.5, 0.6, 0.3, 1.));
    }

	/*
    void createBunny()
    {
        sim::Body *bunny;

        bunny = world()->createBodyTriMesh(bunny_coords, bunny_coords_len,
                                           bunny_ids, bunny_ids_len, 2.);
		bunny->setPos(-2,1,100);
        bunny->visBody()->setColor(0.4, 0.4, 0.4, 1.);
        bunny->setRot(sim::Quat(Vec3(1., 0., 0.), M_PI * 0.5));
        bunny->activate();
    }
	*/
};



void testSim(const int argc, char **argv, const int id) {
	intro;
	
	SimTest1 s;
	s.run();

}


void printTree(osg::Node *node) {
	cerr << "Name=" << node->getName() << "\n";
	osg::Group *group = node->asGroup();
	if (group) {
		for(int i=0;i<group->getNumChildren();i++) {
			cerr << " Children: \n";
			printTree(group->getChild(i));
			cerr << "EOCH\n";
		}
	}

}





class SimTestFormace : public sim::Sim {
  public:
    SimTestFormace	()
        : Sim()
    {
        setTimeStep(sim::Time::fromMs(10));
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
    
        /*
        b = w->createBodyBox(sim::Vec3(20., 20., 1.), 0.);
        b->setPos(0., 0., -10.);
        b->activate();
        */

/*
        b = w->createBodyCube(1., 1.);
        b->visBody()->setColor(osg::Vec4(0., 0., 0., 1.));
        b->setPos(-3., 0.2, 3.);
        b->activate();

        b = w->createBodyCylinderZ(.5, 1., 3.);
        b->visBody()->setTexture("wood.ppm");
        b->visBody()->setColor(1., 0., 0., 1.);
        b->setPos(3., 0., 0.);
        b->activate();

        b = w->createBodyCylinderX(.5, 1., 3.);
        b->setPos(0., 3., 0.);
        b->activate();

        b = w->createBodyCylinderY(.5, 1., 3.);
        b->setPos(0., -3., 0.);
        b->activate();

*/
        createRobot();
		//createArdrone();
		createJezek();
		createPlane();

		printTree(_visworld->sceneRoot());

    }

    void init()
    {
        sim::Sim::init();
        for (size_t i = 0; i < 300; i++){
            visWorld()->step();
            std::cerr << i << "\r";
            usleep(10000);
        }
        std::cerr << std::endl;

        timeRealRestart();
    }

  protected:
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
        ((JointHinge *)j)->setParamLimitLoHi(-M_PI / 2., 0.);

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

        ((JointHinge2 *)j)->setParamLimitLoHi(-M_PI / 4., M_PI / 4.);

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
        sim::Vec3 pos(3. + 2., 3., 2.5);
        sim::Body *chasis;
        sim::ActuatorWheelCylinderX *w[4];
        size_t i;

        chasis = world()->createBodyBox(sim::Vec3(.6, 1., 0.4), 1.);
        for (i = 0; i < 4; i++){
            w[i] = world()->createActuatorWheelCylinderX(0.2, 0.2, 1.);
        }

        chasis->setPos(pos);
        w[0]->setPos(pos.x() + 0.415, pos.y() + 0.4, pos.z() - 0.2);
        w[1]->setPos(pos.x() - 0.415, pos.y() + 0.4, pos.z() - 0.2);
        w[2]->setPos(pos.x() + 0.415, pos.y() - 0.4, pos.z() - 0.2);
        w[3]->setPos(pos.x() - 0.415, pos.y() - 0.4, pos.z() - 0.2);

        for (i = 0; i < 4; i++){
            w[i]->connectToChasis(chasis);
            ((ActuatorWheelCylinderX *)w[i])->joint()->setParamLimitLoHi(-0.0001, 0.0001);
        }

        chasis->activate();
        for (i = 0; i < 4; i++){
            w[i]->activate();
        }

        chasis->visBody()->setText("Robot", 1., osg::Vec4(0.5, 0.6, 0.3, 1.));
    }

    void createRobotMove()
    {
        sim::Vec3 pos(10., -10., -4.9);
        sim::Body *chasis;
        sim::ActuatorWheelCylinderX *w[4];
        size_t i;

        chasis = world()->createBodyBox(sim::Vec3(.6, 1., 0.4), 1.);
        for (i = 0; i < 4; i++){
            w[i] = world()->createActuatorWheelCylinderX(0.2, 0.2, 1.);
        }

        chasis->setPos(pos);
        w[0]->setPos(pos.x() + 0.415, pos.y() + 0.4, pos.z() - 0.2);
        w[1]->setPos(pos.x() - 0.415, pos.y() + 0.4, pos.z() - 0.2);
        w[2]->setPos(pos.x() + 0.415, pos.y() - 0.4, pos.z() - 0.2);
        w[3]->setPos(pos.x() - 0.415, pos.y() - 0.4, pos.z() - 0.2);

        for (i = 0; i < 4; i++){
            w[i]->connectToChasis(chasis);
            ((ActuatorWheelCylinderX *)w[i])->joint()->setParamLimitLoHi(-0.0001, 0.0001);
        }

        ((ActuatorWheelCylinderX *)w[0])->joint()->setParamVel2(5.);
        ((ActuatorWheelCylinderX *)w[0])->joint()->setParamFMax2(10.);
        ((ActuatorWheelCylinderX *)w[1])->joint()->setParamVel2(5.);
        ((ActuatorWheelCylinderX *)w[1])->joint()->setParamFMax2(10.);

        chasis->activate();
        for (i = 0; i < 4; i++){
            w[i]->activate();
        }

        chasis->visBody()->setText("Robot move", 1., osg::Vec4(0.5, 0.6, 0.3, 1.));
    }

	/*
    void createBunny()
    {
        sim::Body *bunny;

        bunny = world()->createBodyTriMesh(bunny_coords, bunny_coords_len,
                                           bunny_ids, bunny_ids_len, 2.);
		bunny->setPos(-2,1,100);
        bunny->visBody()->setColor(0.4, 0.4, 0.4, 1.);
        bunny->setRot(sim::Quat(Vec3(1., 0., 0.), M_PI * 0.5));
        bunny->activate();
    }
	*/

	/*
    void createArdrone()
    {
        sim::Body *ardrone;

        ardrone = world()->createBodyTriMesh(ardrone_verts,ardrone_verts_len,ardrone_ids,ardrone_ids_len,0.2);
		ardrone->setPos(0,1,7);
        ardrone->visBody()->setColor(0.4, 1, 0.4, 1.);
//        bunny->setRot(sim::Quat(Vec3(1., 0., 0.), M_PI * 0.5));
        ardrone->activate();
    }
	*/

	void createJezek()
    {
        sim::Body *obj;

        obj = world()->createBodyTriMesh(jezek_verts,jezek_verts_len,jezek_ids,jezek_ids_len,0.2);
		obj->setPos(2,4,2);
        obj->visBody()->setColor(0.4, 1, 0.4, 1.);
        obj->activate();
    }
	
	void createPlane()
    {
        sim::Body *obj;

        obj = world()->createBodyTriMesh(plane10_verts,plane10_verts_len,plane10_ids,plane10_ids_len,0);
		obj->setPos(0,0,0);
        obj->visBody()->setColor(0.4, 1, 0.4, 1.);
        obj->activate();
    }
	

};


void testFormace(const int argc, char **argv, const int id) {
	intro;
	
	SimTestFormace s;
	s.run();

}









int main(int argc, char **argv) {

	const int k = argc < 2?-1:atoi(argv[1]);
	if (k > 0) {
		argc--;
		argv++;
	} else {
		std::cerr << "Choose on of the following functions:\n";
	}
	// call various routines here
	
	testSim(argc,argv,k);
	testFormace(argc,argv,k);
	return 0;
}

#undef intro
