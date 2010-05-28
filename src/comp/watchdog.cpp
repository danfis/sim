#include "watchdog.hpp"
#include <sim/msg.hpp>
#include <osgGA/GUIEventAdapter>
#include <sim/time.hpp>
#include <math.h>

namespace sim {

namespace comp {

Watchdog::Watchdog(sim::Body *body, const double timeout, const char *paramFile){
    _body = body;
    _timeout = timeout;
	_paramFile = paramFile;
}

Watchdog::~Watchdog(){
    _body = NULL;
}

void Watchdog::init(sim::Sim *sim) {
	_sim = sim;
    _sim->regPostStep(this);

}

void Watchdog::finish(){
}

void Watchdog::cbPostStep() {

    sim::Time t = _sim->timeReal();
    const double ts = t.inSF();
    if (ts > _timeout ) {
        DBG("Watchdog:  time: " << ts << " reaches timeout: " << _timeout);
		char name[200];
		sprintf(name,"%s.result",_paramFile);
        std::ofstream ofs(name);
        sim::Vec3 pos(_body->pos());
        ofs << pos[0] << " " << pos[1] << " " << pos[2] << "\n";
        ofs.close();
        exit(0); // stopping the simulartor is not implemented yet, so we use this really ugly hack
    }
   
}

void Watchdog::processMessage(const Message &msg) {
    
}


}

}
