
#include <fstream>
#include <sstream>
#include <iostream>
#include <math.h>
#include <vector>
#include "sim/sim.hpp"
#include "rawUtils.hpp"

using namespace sim::ode;
using sim::Vec3;
using namespace std;


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


int main(int argc, char **argv) {
    if (argc < 4) {
        cerr << "usage " << argv[0] << " <rawFile> <headerFile> <variableName>\n";
        cerr << "rawFile        file in RAW format with mesh\n";
        cerr << "headerFile     output file in c/c++ header file with definition of the object vertices\n";
        cerr << "variableName   name of the output variable\n";
        exit(0);
    }

    const char *inFile = argv[1];
    const char *hFile = argv[2];
    const char *varName = argv[3];


}



