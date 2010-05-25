#include <unistd.h>
#include <iostream>
#include <fstream>
#include <sstream>

#include "sim/ode/world.hpp"
#include "sim/sim.hpp"
#include "msg.hpp"

#include "bunny.hpp"

using namespace sim::ode;
using sim::Vec3;
using sim::Quat;
using namespace std;


/** find a point p in given point array. if such a points does not exists, add it to the end of the array
  * and return its index */
int getIndex(vector<Vec3> &pts, const Vec3 &p) {
	double mind = -1;
	int minidx = -1;
	double d;
	const double distanceThreshold = 0.01;
	for(int i=0;i<(int)pts.size();i++) {
		const double dx = pts[i].x() - p.x();
		const double dy = pts[i].y() - p.y();
		const double dz = pts[i].z() - p.z();
		d = dx*dx + dy*dy + dz*dz;
		if (d < mind || mind == -1) {
			mind = d;
			minidx = i;
		}
	}

	if (minidx == -1 || mind > distanceThreshold*distanceThreshold) {
		pts.push_back(p);
		minidx = pts.size()-1;
	}
	return minidx;
}

template<typename T>
std::vector<T> lineToNum(const std::string &line) {

	std::vector<T> res;
	std::stringstream iss(line);
	double a;
	while(iss >> a) {
		res.push_back(a);
	}
	return res;
}



vector<sim::Vec3> loadTriangles(const char *filename, int **indices,  Vec3 **vertices, int &indicesSize, int &verticesSize, const double addX=0, const double addY=0, const double addZ=0) {


	std::ifstream ifs(filename);
	string line;
	vector<Vec3> points;
	vector< vector<int> > iindices;

	int discarded = 0;
	const double triangleAreaThreshold = -0.0005;
	while(ifs) {
		std::getline(ifs,line);
		vector<double> vd(lineToNum<double>(line));
		if (vd.size() == 9) {
			Vec3 p1(vd[0]+addX,vd[1]+addY,vd[2]+addZ);
			Vec3 p2(vd[3]+addX,vd[4]+addY,vd[5]+addZ);
			Vec3 p3(vd[6]+addX,vd[7]+addY,vd[8]+addZ);
//			if (triangleArea(p1,p2,p3) > triangleAreaThreshold) {
			if (1 ) {
				int idx1 = getIndex(points, p1);
				int idx2 = getIndex(points, p2);
				int idx3 = getIndex(points, p3);
				vector<int> in;
				in.push_back(idx1);
				in.push_back(idx2);
				in.push_back(idx3);
				iindices.push_back(in);
				in.clear();
			} else {
				discarded++;
			}
		}
		vd.clear();
	}
	(*indices) = new int[iindices.size()*3];
	(*vertices) = new Vec3[points.size()];
	indicesSize = iindices.size()*3;
	verticesSize = points.size();

	for(int i=0;i<(int)points.size();i++) {
		(*vertices)[i][0] = points[i].x();
		(*vertices)[i][1] = points[i].y();
		(*vertices)[i][2] = points[i].z();
	}

	int idx = 0;
	for(int i=0;i<(int)iindices.size();i++) {
		(*indices)[idx++] = iindices[i][0];
		(*indices)[idx++] = iindices[i][1];
		(*indices)[idx++] = iindices[i][2];
	}
	cerr << "Loaded " << points.size() << " points\n";
	cerr << "Loaded " << iindices.size() << " triangles, discarded: "<< discarded << "\n";
	iindices.clear();
	return points;
}






class S : public sim::Sim {
  public:
    S()
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
        w->setStepType(World::STEP_TYPE_QUICK);
        w->setAutoDisable(0.01, 0.01, 5, 0.);
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
        unsigned int ind[] = { 0, 3, 1,
                               3, 4, 1,
                               1, 4, 2,
                               4, 5, 2,
                               3, 6, 4,
                               6, 7, 4,
                               4, 7, 5,
                               7, 8, 5 };
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
};


int main(int argc, char *argv[])
{
    S s;

    s.run();

    return 0;
}
