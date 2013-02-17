/***
 * sim
 * ---------------------------------
 * Copyright (c)2010 Vojta Vonasek <vonasek@labe.felk.cvut.cz>
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
#include <vector>
#include <fstream>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <sys/stat.h>

using namespace std;


/** this program provides searching for parameters of movement of a simple organism
  *
  * lest suppose we have an organism with N joints. We will
  * control a velocity of i-th joint by function: f_i(x) = A_i * sin (f_i*t + phase_i)
  * where amplitude A_i, frewquency f_i and phase phase_i are different for each joint.
  *
  * Now we want to find such a set of A_i,f_i,phase_i, i=1..N such that move the robot
  * towards a predefined goal state. 
  *
  * We will employ a simple Particle Swarm Optimization technique to find such parameters.
  * To measure a fitness we will use a simulator to run a short simulation,e.g for 20 second
  * After that the distance between organism center and desired goal determines the value of
  * the fitness function.
  */



// main logger
ofstream ofl;
	

/** each particle represents A_i, f_i and phase_i
  * for all joints.
  * This parameters are stored in 'data'. Also each particle has to know its best 
  * particle, which is in 'localBest'.
  * The volocities of parameters in parameter space are store in 'velocity'
  */
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
//		void normalize();
};



static void limit(double &val, const double min, const double max) {
	if (val > max) {
		val = max;
	}
	if (val < min) {
		val = min;
	}
}

/*
void Particle::normalize(const double amplitude, const double frequencyMin, const double frequencyMax, const double phaseMin, ocnst double phaseMax){
	for(int i=0;i<(int)data.size()/3;i++) {
		limit(data[3*i+0],-amplitude,amplitude);
		limit(data[3*i+1],frequencyMin,frequencyMax);
		limit(data[3*i+2],phaseMin,phaseMax);
	}
}
*/

static double getRandom(const double from, const double to) {
	return (double)(1.0*rand() / (1.0*RAND_MAX))*(to-from) + from;
}


void saveParam(const char *filename, const Particle &p) {
    ofstream ofs(filename);
    for(int i=0;i<(int)p.data.size()/3;i++) {
        ofs << p.data[3*i+0] << " "<< p.data[3*i+1] << " " << p.data[3*i+2] << "\n";
    }
    ofs.close();
 
}

static double evaluate_demo_movement_tunning(const char *binPath, const char *paramFile, const Particle &p, const int evaluateIters) {
    
    saveParam(paramFile,p);

	double sumFit = 0;	
	for(int it = 0; it < evaluateIters; it++) {
		char name[200];
		sprintf(name,"%s %s",binPath,paramFile);
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

static void printParticle(ofstream &ofs, const Particle &p) {
    ofs << "Velocity: ";
    for(int i=0;i<(int)p.velocity.size();i++) {
        ofs << p.velocity[i] << " ";
    }
    ofs << "\nData: ";
    for(int i=0;i<(int)p.data.size();i++) {
        ofs << p.data[i] << " ";
    }
    ofs << "\nLocalBest: ";
    for(int i=0;i<(int)p.localBest.size();i++) {
        ofs << p.localBest[i] << " ";
    }
    ofs << "\n";
}


/** this demo is for simple box-snake: see demo_movement_tunning.pso */
void pso_demo_movement_tunning(int argc, char **argv) {

    if (argc < 3) {
        cerr << "usage: " << argv[0] << " <demo_movement_tunning> <paramFile> \n";
        cerr << "demo_movement_tunning   .. path to a bin file\n";
        cerr << "paramFile               ..   name of file that will be used for passing parameters to demo_movement_tunning \n";
        exit(0);
    }

    const char *binPath = argv[1];
    const char *paramFile = argv[2];


    const int populationSize = 20;
    const int generationCount = 40;
    const int evaluateIters = 6;
    const int numJoints = 4;
    const double amplitude = 3;
    const double frequencyMin = -2*0.5*M_PI; // in rad
    const double frequencyMax = 2*0.5*M_PI;
    const double phaseMin = -2*M_PI; // in rad
    const double phaseMax= 2*M_PI;





    char name[200];
    sprintf(name,"%s.log",paramFile);
    ofstream ofl(name);
    ofl <<"Staring .. \n";

    vector<Particle> population;
    for(int i=0;i<(int)populationSize;i++) {
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
        for(int j=0;j<(int)population.back().velocity.size();j++) {
            population.back().velocity[j] = 0;
        }
    }


    Particle global(std::vector<double>(numJoints*2*3,0),-1);
    global.fit = -1;

    for(int iter = 0; iter < generationCount;iter++) {
        ofl << "generation " << iter << "\n";


        for(int i=0;i<(int)population.size();i++) {
            population[i].fit = evaluate_demo_movement_tunning(binPath,paramFile,population[i],evaluateIters);
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
        for(int i=0;i<(int)global.data.size();i++) {
            ofl << global.data[i] << " ";
        }
        ofl << "\n";
        ofl.flush();

        for(int i=0;i<(int)population.size();i++) {
            for(int j=0;j<(int)population[i].data.size();j++) {
                population[i].velocity[j] = 0.2*getRandom(0.0,1.0)*population[i].velocity[j]+
                    2*getRandom(0.0,1.0)*(population[i].localBest[j]-population[i].data[j])+
                    2*getRandom(0.0,1.0)*(global.data[j] - population[i].data[j]);
                population[i].data[j] += population[i].velocity[j];
            }
        }



    }


}

#define WDEBUG(x) { cerr << x << "\n"; ofl << x << "\n"; ofl.flush(); }


/** evaluate particle (which defines amplitude, frequency and phase for each robot arm. If the last parameter is not null,
  * the binath will be processed with --withpovray DIR, which render simulation to the given directory */
static double evaluate_demo_psofitness_sssa(const char *binPath, const char *paramFile, const Particle &p, const int evaluateIters, const char *povrayDir) {
    
    saveParam(paramFile,p);

	double sumFit = 0;	
	for(int it = 0; it < evaluateIters; it++) {
		char name[2000];

        if (povrayDir) {
            mkdir(povrayDir,S_IRWXU | S_IRWXG | S_IRWXO);
    		sprintf(name,"%s --input %s --nopause --withpovray %s --time 10",binPath,paramFile,povrayDir);
        } else {
    		sprintf(name,"%s --input %s --nopause --nogui --time 10",binPath,paramFile);
        }
        WDEBUG("Evaluating '" << name << "', iteration="<< it);
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
			WDEBUG("Fitness of iter " << it << " is " << fit);
			sumFit += fit;
		} else {
            // this can occur e.g. if ODE halts during simulation. In such a case, we consider fitness as maximal bad, i.e
            // fitness is set to max_int;
            fit = 1e10;
            sumFit += fit;
			WDEBUG("Cannot evaluate candidate!, fitness set to " << fit);
		}
	}
	cerr << "Result fitness after " << evaluateIters << " iters is " << sumFit/evaluateIters << "\n";
	return sumFit / evaluateIters;
}  

string print(const std::vector<double> &data) {
    stringstream ss;
    for(int i=0;i<(int)data.size();i++) {
        ss << data[i] << " ";
    }
    return ss.str();
}

/** this demo is for simple snake from sssa robots. see demo_psofitness_sssa.cpp */
void pso_demo_psofitness(int argc, char **argv) {

    if (argc < 4) {
        cerr << "usage: " << argv[0] << " <demo_psofitness_sssa> <paramFile> <numOfRobots>\n";
        cerr << "demo_psofitness_sssa    .. path to a bin file\n";
        cerr << "paramFile               .. name of file that will be used for passing parameters to demo_movement_tunning \n";
        cerr << "numOfRobots             .. number of robots used in demo_psofitness_sssa\n";
        exit(0);
    }

    const char *binPath = argv[1];
    const char *paramFile = argv[2];
    const int numOfRobots = atoi(argv[3]);

    const int populationSize = 30;
    const int generationCount = 200;
    const int evaluateIters = 4;

    const double minAmplitude = 0.2;
    const double maxAmplitude = 0.8;
    const double frequencyMin = 0.5;
    const double frequencyMax = 5;
    const double phaseMin = -2*M_PI;
    const double phaseMax = 2*M_PI;

    char name[200];
    sprintf(name,"%s.log",paramFile);
    ofl.open(name);
    ofl <<"Staring .. \n";

    sprintf(name,"%s.dat",paramFile);
    ofstream ofsg(name);
    ofsg << "#properties of particles during PSO algorithm\n";
    ofsg << "#N=" << populationSize << "; population size\n";
    ofsg << "#iteration bestFitness bestGlobalFitness wortsFitness Fitness1 fitness2 ... fitnessN\n";



    vector<Particle> population;
    for(int i=0;i<(int)populationSize;i++) {
        vector<double> data;
        for(int j=0;j<numOfRobots;j++) {
            data.push_back(getRandom(minAmplitude,maxAmplitude));
            data.push_back(getRandom(frequencyMin,frequencyMax));
            data.push_back(getRandom(phaseMin,phaseMax));
        }
        population.push_back(Particle(data,-1));
        for(int j=0;j<(int)population.back().velocity.size();j++) {
            population.back().velocity[j] = 0;
        }
    }


    Particle global(std::vector<double>(numOfRobots*3,0),-1); // each particle has: amplitude, frequency and phase
    global.fit = -1;

    for(int iter = 0; iter < generationCount;iter++) {
        WDEBUG("---------------------------------------------");
        WDEBUG("Generation " << iter);


        char name[2000];
        sprintf(name,"%s.generation-%04d.population",paramFile,iter);
        ofstream ofs2(name);
        ofs2 << "#fitness1 particle1 ..\n";

        for(int i=0;i<(int)population.size();i++) {
            population[i].fit = evaluate_demo_psofitness_sssa(binPath,paramFile,population[i],evaluateIters,NULL);
            WDEBUG("Population[" << i << "].fit=" << population[i].fit);
            ofs2 << population[i].fit << " ";
            for(int j=0;j<(int)population[i].data.size()/3;j++) {
                ofs2 << population[i].data[3*j+0] << " " << population[i].data[3*j+1] << " " << population[i].data[3*j+2] << " ";
            }
            ofs2 << "\n";
        }
        ofs2.close();


        double worstFitness = -1;
        double bestActualFitness = 1e10;
        int bestParticleOfPupulationIdx = -1;
        bool newGlobal = false;
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
                newGlobal = true;
            }
            if (population[i].fit < population[i].localBestFit || population[i].localBestFit == -1) {
                population[i].localBest = population[i].data;
                population[i].localBestFit = population[i].fit;
            } 
            if (population[i].fit > worstFitness || i == 0) {
                worstFitness = population[i].fit;
            }
            if (population[i].fit < bestActualFitness || i == 0) {
                bestActualFitness = population[i].fit;
                bestParticleOfPupulationIdx = i;
            }
            printParticle(ofl,population[i]);
        }

        ofsg << iter << " "<< bestActualFitness << " " << global.fit << " " << worstFitness << " ";
        for(int i=0;i<(int)population.size();i++) {
            ofsg << population[i].fit << " ";
        }
        ofsg << "\n";
        ofsg.flush();

        WDEBUG("Best particle at generation " << iter << ".fit = " << global.fit);
        WDEBUG("Best particle at generation " << iter << ": " << print(global.data));
        if (newGlobal) {
            WDEBUG("Rendering best particle of generation " << iter );
            char name[2000];
            sprintf(name,"%s-povray-generation-%d/",paramFile,iter);
            evaluate_demo_psofitness_sssa(binPath,paramFile,global,1,name);
//            sprintf(name,"%s-povray-generation-%d/bestparticle",paramFile,iter);
//            saveParam(name,population[bestParticleOfPupulationIdx]);
        }

        for(int i=0;i<(int)population.size();i++) {
            for(int j=0;j<(int)population[i].data.size();j++) {
                population[i].velocity[j] = 0.1*getRandom(0.0,1.0)*population[i].velocity[j]+
                    2*getRandom(0.0,1.0)*(population[i].localBest[j]-population[i].data[j])+
                    2*getRandom(0.0,1.0)*(global.data[j] - population[i].data[j]);
                population[i].data[j] += population[i].velocity[j];
                for(int k=0;k<(int)population[i].data.size()/3;k++) {
                    limit(population[i].data[3*k+0],minAmplitude,maxAmplitude);
//                    limit(population[i].data[3*k+1],frequencyMin,frequencyMax);
//                    limit(population[i].data[3*k+2],phaseMin,phaseMax);
                }
            }
        }
    }
    ofsg.close();
    ofl.close();
}


int main(int argc, char **argv) {
//    pso_demo_movement_tunning(argc,argv);
    pso_demo_psofitness(argc,argv);
}


