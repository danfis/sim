#include <unistd.h>
#include <iostream>

#include "sim/bullet/world.hpp"
#include "sim/sim.hpp"
#include "msg.hpp"

using namespace sim::bullet;
using sim::Vec3;


/** find a point p in given point array. if such a points does not exists, add it to the end of the array
  * and return its index */
int getIndex(vector<Vec3> &pts, const Vec3 &p) {
	double mind = -1;
	int minidx = -1;
	double d;
	const double distanceThreshold = 0.01;
	for(int i=0;i<(int)pts.size();i++) {
		const double dx = pts[i].x - p.x;
		const double dy = pts[i].y - p.y;
		const double dz = pts[i].z - p.z;
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



vector<sim::Vec3> loadTriangles(const char *filename, int **indices,  Vec3 **vertices, int &indicesSize, int &verticesSize, const double addX=0, const double addY=0, const double addZ=0) {


	ifstream ifs(filename);
	string line;
	vector<Vec3> points;
	vector< vector<int> > iindices;
	struct rusage t1,t2;

	getTime(&t1);
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
		(*vertices)[i][0] = points[i].x;
		(*vertices)[i][1] = points[i].y;
		(*vertices)[i][2] = points[i].z;
	}

	int idx = 0;
	for(int i=0;i<(int)iindices.size();i++) {
		(*indices)[idx++] = iindices[i][0];
		(*indices)[idx++] = iindices[i][1];
		(*indices)[idx++] = iindices[i][2];
	}
	getTime(&t2);
	cerr << "Loaded " << points.size() << " points\n";
	cerr << "Loaded " << iindices.size() << " triangles, discarded: "<< discarded << "\n";
	cerr << "Loaded in " << getTime(t1,t2) << "\n";
	iindices.clear();
	return points;
}



class S : public sim::Sim {
  public:
    S()
        : Sim()
    {
        World *w = new World();
        sim::Body *b;

        setWorld(w);
    
        /*
        b = w->createBodyBox(sim::Vec3(20., 20., 1.), 0.);
        b->setPos(0., 0., -10.);
        b->activate();
        */

        b = w->createBodyCube(1., 1.);
        b->visBody()->setColor(osg::Vec4(0.5, 0.5, 0.5, 0.5));
        b->setPos(-6.5, 6.5, -5.);
        b->activate();

        b = w->createBodySphere(1.3, 2.);
        b->visBody()->setColor(osg::Vec4(0.5, 0.1, 0.8, 0.3));
        b->setPos(-3., 0., 0.);
        b->activate();

        b = w->createBodyCube(1., 1.);
        b->visBody()->setColor(osg::Vec4(0., 0., 0., 1.));
        b->setPos(-3., 0.2, 3.);
        b->activate();

        b = w->createBodyCylinderZ(.5, 1., 3.);
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
        createRobot();

		int *indices = NULL;
		Vec3 *vertices = NULL;
		int indicesSize = 0;
		int verticesSize = 0;
		vector<SPoint> points(loadTriangles(rawFile,&indices, &vertices, indicesSize, verticesSize));
	
/*
        Vec3 verts[] = { Vec3(-10., 10., -0.5),
                         Vec3(0., 10., -0.8),
                         Vec3(10., 10., -0.5),
                         Vec3(-10., 0., 0.5),
                         Vec3(0., 0., 0.),
                         Vec3(10., 0., 0.7),
                         Vec3(-10., -10., 0.4),
                         Vec3(0., -10., 1.),
                         Vec3(10., -10., 0.8) };
        unsigned int ind[] = { 0, 1, 4,
                               1, 2, 5,
                               0, 4, 3,
                               1, 5, 4,
                               3, 4, 7,
                               4, 5, 8,
                               3, 7, 6,
                               4, 8, 7 };
        b = w->createBodyTriMesh(verts, 9, ind, 24);
*/
        b = w->createBodyTriMesh(vertices,verticesSize,indices,indicesSize);
        b->setPos(0., 0., -10.3);
        b->activate();
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
        sim::Vec3 pos(-6., -6., -9.);
        sim::Body *cu = world()->createBodyCube(1., 1.);
        sim::Body *s = world()->createBodySphere(0.5, 3.);
        sim::Joint *j;

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
        sim::Vec3 pos(-6., 6., -9.);
        sim::Body *cy = world()->createBodyCylinderZ(0.5, 0.5, 1.);
        sim::Body *cu = world()->createBodyCube(1., 1.);
        sim::Joint *j;

        cu->setPos(pos);
        cy->setPos(pos + sim::Vec3(0., 0., 2.));

        j = world()->createJointHinge(cu, cy, pos + sim::Vec3(0., 0., 1.), sim::Vec3(0., 1., 0.));

        cu->activate();
        cy->activate();
        j->activate();

        cu->visBody()->setText("Hinge", 1., osg::Vec4(0.7, 0.6, 0.3, 1.));
    }

    void createRobot()
    {
        sim::Vec3 pos(3., 3., -6.);
        sim::Body *chasis;
        sim::ActuatorWheelCylinderX *w[4];
        size_t i;

        chasis = world()->createBodyBox(sim::Vec3(.6, 1., 0.4), 1.);
        for (i = 0; i < 4; i++){
            w[i] = world()->createActuatorWheelCylinderX(0.2, 0.2, 1.);
        }

        chasis->setPos(pos);
        w[0]->setPos(pos.x() + 0.405, pos.y() + 0.4, pos.z() - 0.2);
        w[1]->setPos(pos.x() - 0.405, pos.y() + 0.4, pos.z() - 0.2);
        w[2]->setPos(pos.x() + 0.405, pos.y() - 0.4, pos.z() - 0.2);
        w[3]->setPos(pos.x() - 0.405, pos.y() - 0.4, pos.z() - 0.2);

        for (i = 0; i < 4; i++){
            w[i]->connectToChasis(chasis);
        }

        chasis->activate();
        for (i = 0; i < 4; i++){
            w[i]->activate();
        }

        chasis->visBody()->setText("Robot", 1., osg::Vec4(0.5, 0.6, 0.3, 1.));
    }
};


int main(int argc, char *argv[])
{
    S s;

    s.run();

    return 0;
}

