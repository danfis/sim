#include <iostream>
#include <unistd.h>
#include <fstream>
#include <sstream>
#include <vector>

#include "sim/ode/world.hpp"
#include "sim/sim.hpp"
#include "msg.hpp"
//#include "meshes/ardrone.h"
#include "meshes/jezek.h"
#include "meshes/surface.h"
#include "meshes/plane.h"
#include "sim/comp/povray.hpp"
#include "sim/comp/snake.hpp"

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


int determineShapeClass(osg::Shape *s) {
	if (!s) {
		return -1;
	}
	{
		osg::Box *b = new osg::Box();
		if (s->isSameKindAs(b)) {
			return 0;
		}
	}
	{
		osg::Capsule *b = new osg::Capsule();
		if (s->isSameKindAs(b)) {
			return 1;
		}
	}
	{
		osg::Cone *b = new osg::Cone();
		if (s->isSameKindAs(b)) {
			return 2;
		}
	}
	{
		osg::Cylinder *b = new osg::Cylinder();
		if (s->isSameKindAs(b)) {
			return 3;
		}
	}
	{
		osg::HeightField *b = new osg::HeightField();
		if (s->isSameKindAs(b)) {
			return 4;
		}
	}
	{
		osg::Sphere *b = new osg::Sphere();
		if (s->isSameKindAs(b)) {
			return 5;
		}
	}
	{
		osg::TriangleMesh *b = new osg::TriangleMesh();
		if (s->isSameKindAs(b)) {
			return 6;
		}
	}
	return -1;
}

void drawBox(osg::Shape *s, const int index) {
	char name[200];
	sprintf(name,"box.%06d.inc",index);
	ofstream ofs(name);
	ofs << "// box \n";
	ofs << "#define box" << index << " = \n";

	osg::Box *b = (osg::Box *)s;

	osg::Vec3 center = b->getCenter();
	osg::Vec3 length = b->getHalfLengths();

	ofs	<< "box { ";
	ofs << "<" << center[0]-length[0] << "," << center[1]-length[1] << "," << center[2]-length[2] << ">, ";
	ofs	<< "<" << center[0]+length[0] << "," << center[1]+length[1] << "," << center[2]+length[2] << ">  ";
	ofs << "color { rgb <1,0,1> }";
	ofs << "\n}\n";
	ofs.close();
}

void drawCylinder(osg::Shape *s, const int index) {
	char name[200];
	osg::Cylinder *c = (osg::Cylinder *)s;
	double radius = c->getRadius();
	double height = c->getHeight();

	sprintf(name,"cyl.%06d.inc",index);
	ofstream ofs(name);
	ofs << "// cylinder \n";
	ofs << "#define cyl" << index << " = \n";

	osg::Vec3 center = c->getCenter();
	ofs << "cylinder { ";
	ofs << "<" << center[0] << "," << center[1] << "," << center[2]-height/2.0 << ">, ";
	ofs << "<" << center[0] << "," << center[1] << "," << center[2]+height/2.0 << ">, ";
	ofs << radius << " ";
	ofs << "color { rgb <1,0,0> }\n";
	ofs << "}\n";
	ofs.close();

}

void printTree(osg::Node *node) {
	osg::Geode *geode = node->asGeode();

	static int index = 0;
	if (geode) {
		cerr << "Node is geodde\n";
		cerr << "Geode num drawables = " << geode->getNumDrawables() << "\n";
		for(int i=0;i<(int)geode->getNumDrawables();i++) {
			osg::Drawable *d = geode->getDrawable(i);
			osg::Shape *s = d->getShape();
			int sc = determineShapeClass(s);
			switch (sc) {
				case 0: { drawBox(s,index++); break; }
				case 3: { drawCylinder(s,index++); break; }
			}	
		}
	}

	cerr << "Name=" << node->getName() << "\n";
	osg::Group *group = node->asGroup();
	if (group) {
		for(int i=0;i<(int)group->getNumChildren();i++) {
			cerr << " Children: \n";
			printTree(group->getChild(i));
			cerr << "EOCH\n";
		}
	}

}


class SimTestFormace : public sim::Sim {
	vector<sim::Body *> snakeBodies;
  public:
    SimTestFormace	()
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
    
/*        
        b = w->createBodyBox(sim::Vec3(2., 2., 1.), 0.4);
        b->visBody()->setColor(osg::Vec4(1,0.8,0.4, 1.));
        b->setPos(3, 4., 5);
        b->setRot(sim::Quat(Vec3(1., 1., 0.), M_PI * 0.3));
        b->activate();

		b = w->createBodyBox(sim::Vec3(3., 2., 1.), 1.8);
        b->visBody()->setColor(osg::Vec4(1.0, 0.63, 0.32, 1.));
        b->setPos(-3, 4., 5);
        b->activate();


        b = w->createBodyCube(1., 1.);
        b->visBody()->setColor(osg::Vec4(0., 0., 0., 1.));
        b->setPos(-3., 0.2, 3.);
        b->activate();

	
        b = w->createBodyCylinderZ(.5, 1., 3.);
        b->visBody()->setTexture("wood.ppm");
        b->visBody()->setColor(1., 0., 0., 1.);
        b->setPos(3., 0., 2);
        b->activate();

        b = w->createBodyCylinderX(.5, 1., 3.);
        b->setPos(0., 3., 1.5);
        b->activate();

        b = w->createBodyCylinderY(.5, 1., 3.);
        b->setPos(3., 0., 10);
        b->visBody()->setColor(1., 1, 0., 0.5);
        b->activate();
*/

        //createRobot(bodies);
		//createArdrone();
	//	createJezek();
		createPlane();
//		createSurface();
//		createRobotCarlike();


		createSnake();

//		sim::comp::Povray *pc = new sim::comp::Povray("povray/");
//		addComponent(pc);
//		regPostStep(pc);


		sim::comp::Snake *sc = new sim::comp::Snake(snakeBodies);
		addComponent(sc);
		regMessage(sc,sim::MessageKeyPressed::Type);

    }

    void init()
    {
        sim::Sim::init();
        for (size_t i = 0; i < 150; i++){
            visWorld()->step();
            std::cerr << i << "\r";
            usleep(5000);
        }
        std::cerr << std::endl;

        timeRealRestart();
    }

  protected:

	void createSnake() {

		const double posx = 0;
		const double posy = 0;
		const double posz = 1.1;
		const double width = 1;
		const double gap = width*1.1;
		const double mass = 0.5;

        sim::Body *b1 = world()->createBodyCube(width,mass);
		b1->setPos(sim::Vec3(posx,posy,posz));
        sim::Body *b2 = world()->createBodyCube(width,mass);
		b2->setPos(sim::Vec3(posx+1*gap,posy,posz));
        sim::Body *b3 = world()->createBodyCube(width,mass);
		b3->setPos(sim::Vec3(posx+2*gap,posy,posz));
        sim::Body *b4 = world()->createBodyCube(width,mass);
		b4->setPos(sim::Vec3(posx+3*gap,posy,posz));
        sim::Body *b5 = world()->createBodyCube(width,mass);
		b5->setPos(sim::Vec3(posx+4*gap,posy,posz));

		sim::Joint *j1 = world()->createJointHinge2(b1,b2,b1->pos(),sim::Vec3(0,1,0),sim::Vec3(1,0,0));
		sim::Joint *j2 = world()->createJointHinge2(b2,b3,b2->pos(),sim::Vec3(0,1,0),sim::Vec3(1,0,0));
		sim::Joint *j3 = world()->createJointHinge2(b3,b4,b3->pos(),sim::Vec3(0,1,0),sim::Vec3(1,0,0));
		sim::Joint *j4 = world()->createJointHinge2(b4,b5,b4->pos(),sim::Vec3(0,1,0),sim::Vec3(1,0,0));

		j1->setParamLimitLoHi(0,90*M_PI/180.0);
		j2->setParamLimitLoHi(0,90*M_PI/180.0);
		j3->setParamLimitLoHi(0,90*M_PI/180.0);
		j4->setParamLimitLoHi(0,90*M_PI/180.0);

		j1->setParamLimitLoHi(-45*M_PI/180.0,45*M_PI/180.0);
		j2->setParamLimitLoHi(-45*M_PI/180.0,45*M_PI/180.0);
		j3->setParamLimitLoHi(-45*M_PI/180.0,45*M_PI/180.0);
		j4->setParamLimitLoHi(-45*M_PI/180.0,45*M_PI/180.0);


		b1->activate();
		b2->activate();
		b3->activate();
		b4->activate();
		b5->activate();
		j1->activate();
		j2->activate();
		j3->activate();
		j4->activate();

		snakeBodies.push_back(b1);
		snakeBodies.push_back(b2);
		snakeBodies.push_back(b3);
		snakeBodies.push_back(b4);

		j3->setParamVel(1);
		j3->setParamFMax(10);

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
   

	void createRobotCarlike()
    {
		const double robotWidth = 2;
		const double robotLength = 3;
		const double robotHeight = 0.6;
		const double wheelRadius = 0.8;
		const double wheelWidth = 0.2;
		sim::Vec3 pos(5,2,2.6);

        sim::Body *chasis, *w1,*w2,*w3,*w4;

        chasis = world()->createBodyBox(sim::Vec3(robotLength, robotWidth, robotHeight), 1.);
        chasis->setPos(pos);
        chasis->visBody()->setColor(0.4, 0.1, 1., 0.3);
		chasis->collSetDontCollideId(1);

		// front wheels
		w1 = world()->createBodyCylinderY(wheelRadius,wheelWidth,0.5);
        w1->setPos(sim::Vec3(pos[0]+robotLength/2,pos[1]-robotWidth/2-wheelWidth/1.8,pos[2]));
        w1->visBody()->setColor(1., 0., 0., 0.8);
		w1->collSetDontCollideId(1);

		w2 = world()->createBodyCylinderY(wheelRadius,wheelWidth,0.5);
        w2->setPos(sim::Vec3(pos[0]+robotLength/2,pos[1]+robotWidth/2+wheelWidth/1.8,pos[2]));
        w2->visBody()->setColor(1., 0., 0., 0.8);
		w2->collSetDontCollideId(1);
		
		// rear wheel
		w3 = world()->createBodyCylinderY(wheelRadius,wheelWidth,0.5);
        w3->setPos(sim::Vec3(pos[0]-robotLength/2,pos[1]-robotWidth/2-wheelWidth/1.8,pos[2]));
        w3->visBody()->setColor(1., 0., 0., 0.8);

		w4 = world()->createBodyCylinderY(wheelRadius,wheelWidth,0.5);
        w4->setPos(sim::Vec3(pos[0]-robotLength/2,pos[1]+robotWidth/2+wheelWidth/1.8,pos[2]));
        w4->visBody()->setColor(1., 0., 0., 0.8);


        chasis->setPos(pos);

		// joint between chassis and wheels
		sim::Joint *j1 = world()->createJointHinge2(chasis,w1,
				sim::Vec3(pos[0]+robotLength/2,pos[1]-robotWidth/2-wheelWidth/1.8,pos[2]),
				sim::Vec3(0,0,1),
				sim::Vec3(0,1,0));

		sim::Joint *j2 = world()->createJointHinge2(chasis,w2,
				sim::Vec3(pos[0]+robotLength/2,pos[1]+robotWidth/2+wheelWidth/1.8,pos[2]),
				sim::Vec3(0,0,1),
				sim::Vec3(0,1,0));

		sim::Joint *j3 = world()->createJointHinge2(chasis,w3,
				sim::Vec3(pos[0]-robotLength/2,pos[1]-robotWidth/2-wheelWidth/1.8,pos[2]),
				sim::Vec3(0,0,1),
				sim::Vec3(0,1,0));

		sim::Joint *j4 = world()->createJointHinge2(chasis,w4,
				sim::Vec3(pos[0]-robotLength/2,pos[1]+robotWidth/2+wheelWidth/1.8,pos[2]),
				sim::Vec3(0,0,1),
				sim::Vec3(0,1,0));

		const double angle = 15*M_PI/180.0;
//		j1->setParamLimitLoHi(-angle,angle);
//		j2->setParamLimitLoHi(-angle,angle);
		j3->setParamLimitLoHi(-0.001,0.001);
		j4->setParamLimitLoHi(-0.001,0.001);
		
		j1->setParamLimitLoHi(-0.001,0.001);
		j2->setParamLimitLoHi(-0.001,0.001);

        chasis->activate();

		w1->activate();
		w2->activate();
		w3->activate();
		w4->activate();

		j1->activate();
		j2->activate();
		j3->activate();
		j4->activate();

        chasis->visBody()->setText("Robot", 1., osg::Vec4(0.5, 0.6, 0.3, 1.));

		// front wheel
		j1->setParamVel2(-5);
		j1->setParamFMax2(50);
		j2->setParamVel2(-5);
		j2->setParamFMax2(50);

		j1->setParamVel(0);
//		j1->setParamFMax(25);
		j2->setParamVel(0);
//		j2->setParamFMax(25);

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

		bodies->push_back(bunny->visBody());
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
	
	void createSurface()
    {
        sim::Body *obj;

        obj = world()->createBodyTriMesh(surface_verts,surface_verts_len,surface_ids,surface_ids_len,0.);
		obj->setPos(0,0,-2);
        obj->visBody()->setColor(0.4, 1, 0.4, 1.);
        obj->activate();
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
