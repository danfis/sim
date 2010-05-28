#include <iostream>
#include <vector>
#include <fstream>
#include <math.h>
#include <stdlib.h>


using namespace std;


const double amplitude = 3;
const double frequencyMin = -2*10*M_PI; // in rad
const double frequencyMax = 2*10*M_PI;
const double phaseMin = -2*M_PI; // in rad
const double phaseMax= 2*M_PI;


class Particle {
    public:
        std::vector<double> data;
        std::vector<double> localBest;
        std::vector<double> velocity;
        double fit;
        double localBestFit;
        Particle(const std::vector<double> &d, const double fitness = 0)
            :data(d),localBest(d),velocity(d),fit(fitness),localBestFit(fit) {}
        Particle(const Particle &p)
            :data(p.data),localBest(p.localBest),velocity(p.velocity),fit(p.fit),localBestFit(p.localBestFit) {}
		void normalize();
};


static void limit(double &val, const double max, const double min) {
	if (val > max) {
		val = max;
	}
	if (val < min) {
		val = min;
	}
}

void Particle::normalize(){
	for(int i=0;i<data.size()/3;i++) {
		limit(data[3*i+0],-amplitude,amplitude);
		limit(data[3*i+1],frequencyMin,frequencyMax);
		limit(data[3*i+2],phaseMin,phaseMax);
	}
}


static double getRandom(const double from, const double to) {
	return (double)(1.0*rand() / (1.0*RAND_MAX))*(to-from) + from;
}


void saveParam(const char *filename, const Particle &p) {
    ofstream ofs(filename);
    for(int i=0;i<p.data.size()/3;i++) {
        ofs << p.data[3*i+0] << " "<< p.data[3*i+1] << " " << p.data[3*i+2] << "\n";
    }
    ofs.close();
 
}

double evaluate(const char *paramFile, const Particle &p, const int evaluateIters) {
    
    saveParam(paramFile,p);

	double sumFit = 0;	
	for(int it = 0; it < evaluateIters; it++) {
		char name[200];
		sprintf(name,"./demo_movement_tunning %s",paramFile);
		system(name);

		const double posx = 8;
		const double posy = 0;
		const double posz = 0.5;

		double fit = -1;
		sprintf(name,"%s.result",paramFile);
		ifstream ifs(name);
		if (ifs) {
			double x,y,z;
			ifs >> x >> y >> z;
			fit = sqrt((posx-x)*(posx-x) + (posy-y)*(posy-y)+(posz-z)*(posz-z));
			cerr << "Fitness of iter " << it << " is " << fit << "\n";
			sumFit += fit;
		} else {
			cerr << "Cannot evaluate candidate!\n";
			exit(0);
		}
	}
	cerr << "Result fitness after " << evaluateIters << " iters is " << sumFit/evaluateIters << "\n";
	return sumFit / evaluateIters;
}  

void printParticle(ofstream &ofs, const Particle &p) {
    ofs << "Velocity: ";
    for(int i=0;i<p.velocity.size();i++) {
        ofs << p.velocity[i] << " ";
    }
    ofs << "\nData: ";
    for(int i=0;i<p.data.size();i++) {
        ofs << p.data[i] << " ";
    }
    ofs << "\nLocalBest: ";
    for(int i=0;i<p.localBest.size();i++) {
        ofs << p.localBest[i] << " ";
    }
    ofs << "\n";
}


int main(int argc, char **argv) {

	if (argc < 2) {
		cerr << "usage: " << argv[0] << " <paramFile> \n";
		cerr << "paramFile   ..   name of file that will be used for passing parameters to demo_movement_tunning \n";
		exit(0);
	}

	const char *paramFile = argv[1];

    const int populationSize = 40;
    const int generationCount = 40;
	const int evaluateIters = 10;

    const int numJoints = 4;

	char name[200];
	sprintf(name,"%s.log",paramFile);
    ofstream ofl(name);
    ofl <<"Staring .. \n";

    vector<Particle> population;
    for(int i=0;i<populationSize;i++) {
        vector<double> data;
        for(int j=0;j<numJoints;j++) {
            data.push_back(getRandom(-amplitude,amplitude));
            data.push_back(getRandom(frequencyMin,frequencyMax));
            data.push_back(getRandom(phaseMin,phaseMax));
            data.push_back(getRandom(-amplitude,amplitude));
            data.push_back(getRandom(frequencyMin,frequencyMax));
            data.push_back(getRandom(phaseMin,phaseMax));
        }
        population.push_back(Particle(data,-1));
        for(int j=0;j<population.back().velocity.size();j++) {
            population.back().velocity[j] = 0;
        }
    }


    Particle global(std::vector<double>(numJoints*2*3,0),-1);
    global.fit = -1;

    for(int iter = 0; iter < generationCount;iter++) {
        ofl << "generation " << iter << "\n";


        for(int i=0;i<(int)population.size();i++) {
            population[i].fit = evaluate(paramFile,population[i],evaluateIters);
            ofl << "p[" << i << "].fit=" << population[i].fit << "\n";
            ofl.flush();
        }



        for(int i=0;i<(int)population.size();i++) {
            if (population[i].fit < global.fit || global.fit == -1) {
                global.data = population[i].data;
                global.fit = population[i].fit;
                global.localBestFit = population[i].localBestFit;
                global.localBest = population[i].localBest;
                global.velocity = population[i].velocity;
				char name[200];
				sprintf(name,"%s.global",paramFile);
                saveParam(name,global);
            }
            if (population[i].fit < population[i].localBestFit || population[i].localBestFit == -1) {
                population[i].localBest = population[i].data;
                population[i].localBestFit = population[i].fit;
            } 
            printParticle(ofl,population[i]);
        }

        cerr << "Best particle is " << global.fit << "\n";
        ofl << "Best particle is " << global.fit << ": ";
        for(int i=0;i<global.data.size();i++) {
            ofl << global.data[i] << " ";
        }
        ofl << "\n";
        ofl.flush();

        for(int i=0;i<(int)population.size();i++) {
            for(int j=0;j<population[i].data.size();j++) {
                population[i].velocity[j] = 0.2*getRandom(0.0,1.0)*population[i].velocity[j]+
                    2*getRandom(0.0,1.0)*(population[i].localBest[j]-population[i].data[j])+
                    2*getRandom(0.0,1.0)*(global.data[j] - population[i].data[j]);
                population[i].data[j] += population[i].velocity[j];
            }
        }



    }


}


