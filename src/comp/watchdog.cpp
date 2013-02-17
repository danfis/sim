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

#include "watchdog.hpp"
#include <sim/msg.hpp>
#include <osgGA/GUIEventAdapter>
#include <sim/time.hpp>
#include <math.h>

namespace sim {

namespace comp {

Watchdog::Watchdog(sim::Body *body, const double timeout, const char *paramFile, const bool simulatedTime){
    _body = body;
    _timeout = timeout;
	_paramFile = paramFile;
    _simulatedTime = simulatedTime;
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

    sim::Time t;
    if (_simulatedTime) {
        t = _sim->timeSimulated();
    } else {
        t = _sim->timeReal();
    }

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
