#include "watchdog.hpp"
#include <sim/msg.hpp>
#include <osgGA/GUIEventAdapter>
#include <sim/time.hpp>
#include <math.h>

namespace sim {

namespace comp {

Watchdog::Watchdog(sim::Body *body, const double timeout){
    _body = body;
    _timeout = timeout;
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
    }
   
}

void Watchdog::processMessage(const Message &msg) {
    
}


}

}
