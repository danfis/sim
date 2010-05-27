#include <iostream>
#include <vector>
#include <fstream>
#include <math.h>



using namespace std;

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
};

static double getRandom(const double from, const double to) {
	return (double)(1.0*rand() / (1.0*RAND_MAX))*(to-from) + from;
}


double evaluate(const Particle &p) {
    ofstream ofs("param.txt");
    for(int i=0;i<p.data.size()/3;i++) {
        ofs << p.data[3*i+0] << " "<< p.data[3*i+1] << " " << p.data[3*i+2] << "\n";
    }
    ofs.close();
    char name[200];
    sprintf(name,"./demo_movement_tunning param.txt");
    system(name);

    double fit = -1;
    ifstream ifs("result.txt");
    if (ifs) {
        double x,y,z;
        ifs >> x >> y >> z;
        fit = sqrt((7-x)*(7-x) + (0-y)*(0-y)+(z-0.5)*(z-0.5));
        cerr << "Fitness is " << fit << "\n";
    } else {
        cerr << "Cannot evaluate candidate!\n";
        exit(0);
    }
    return fit;
}   

int main(int argc, char **argv) {

    const int populationSize = 30;
    const int generationCount = 10;

    const int numJoints = 4;

    ofstream ofl("pso.log");
    ofl <<"Staring .. \n";

    vector<Particle> population;
    for(int i=0;i<populationSize;i++) {
        vector<double> data;
        for(int j=0;j<numJoints;j++) {
            data.push_back(getRandom(0,3));
            data.push_back(getRandom(0,2*M_PI*5));
            data.push_back(getRandom(-2*M_PI,2*M_PI));
            data.push_back(getRandom(0,3));
            data.push_back(getRandom(0,2*M_PI*5));
            data.push_back(getRandom(-2*M_PI,2*M_PI));
        }
        population.push_back(Particle(data,-1));
    }


    Particle global(std::vector<double>(4*2*3,0),-1);

    for(int iter = 0; iter < generationCount;iter++) {
        ofl << "generation " << iter << "\n";


        for(int i=0;i<(int)population.size();i++) {
            population[i].fit = evaluate(population[i]);
            ofl << "p[" << i << "].fit=" << population[i].fit << "\n";
            ofl.flush();
        }



        for(int i=0;i<(int)population.size();i++) {
            if (population[i].fit < global.fit || global.fit == -1) {
                global.data = population[i].data;
                global.fit = population[i].fit;
            }
            if (population[i].fit < population[i].localBestFit || population[i].localBestFit == -1) {
                population[i].localBest = population[i].data;
                population[i].localBestFit = population[i].fit;
            } 
        }

        cerr << "Best particle is " << global.fit << "\n";
        ofl << "Best particle is " << global.fit << ": ";
        for(int i=0;i<global.data.size();i++) {
            ofl << global.data[i] << " ";
        }
        ofl << "\n";
        ofl.flush();

        for(int i=0;i<(int)population.size();i++) {
            for(int j=0;j<population[i].data.size();i++) {
                population[j].velocity[j] = getRandom(0.0,1.0)*population[j].velocity[j]+
                    2*getRandom(0.0,1.0)*(population[i].localBest[j]-population[i].data[j])+
                    2*getRandom(0.0,1.0)*(global.data[j] - population[i].data[j]);
                population[j].data[j] += population[i].velocity[j];
            }
        }



    }


}


