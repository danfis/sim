/***
 * sim
 * ---------------------------------
 * Copyright (c)2010 Vojta Vonasek <vonasek@labe.felk.cvut.cz>
 *                   Daniel Fiser <danfis@danfis.cz>
 *
 *  This file is part of sim.
 *
 *  sim is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation; either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  sim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "povray.hpp"
#include "sim/common.hpp"

namespace sim {

namespace comp {

Povray::Povray(const char *prefix, const double frameTime)
    : sim::Component(), _frame(0), _prefix(prefix),_frameTime(frameTime),_lastTime(0)
{
}

Povray::~Povray()
{
}

void Povray::init(sim::Sim *sim){
	char name[200];

	_sim = sim;

	sprintf(name, "%scamera.inc", _prefix);
	std::ofstream ofs(name);
	ofs << "// camera definition \n";
	ofs << "#include \"colors.inc\"\n";
	ofs << "camera {\n";
	ofs << "\tlocation <6,-6,4>\n";
	ofs << "\tsky <0,0,1>\n";
	ofs << "\tlook_at <1,0,3>\n";
	ofs << "}\n";
	ofs << "light_source { <20,-5,20> color White }\n";
	ofs << "light_source { <-20,-5,20> color White }\n";
	ofs << "light_source { <0,-15,10> color White }\n";
	ofs.close();

    const std::list<VisBody *> &bodies = _sim->visWorld()->bodies();
    int i = 0;
    for_each(std::list<VisBody *>::const_iterator, bodies){
        if (*it){
			sprintf(name, "%sobject_%06d.inc", _prefix, i);
			std::ofstream ofs(name);
			sprintf(name,"object_%06d",i);
			ofs << "#declare "<<name<<"=";
            (*it)->exportToPovray(ofs, VisBody::POVRAY_GEOM);
			ofs.close();
            i++;
        }
    }

    _sim->regPostStep(this);
}

void Povray::finish(){
}

void Povray::cbPostStep(){

	char name[200];
    const std::list<VisBody *> &bodies = _sim->visWorld()->bodies();

    sim::Time t = _sim->timeSimulated();

    if (t.inSF()-_lastTime > _frameTime) { 

        sprintf(name, "%sframe_%06d.pov", _prefix, _frame);
        std::ofstream ofs(name);
        ofs << "//simulated time: " << t.inSF() << "\n";
        ofs << "#include \"camera.inc\"\n";

        int i = 0;
        for_each(std::list<VisBody *>::const_iterator, bodies){
            if (*it) {
                sprintf(name,"object_%06d.inc",i);
                ofs << "#include \"" << name << "\"\n";
                i++;
            }
        }

        i = 0;
        for_each(std::list<VisBody *>::const_iterator, bodies){
            if (*it) {
                sprintf(name,"object_%06d",i);
                ofs << "object {" << name << "\n";
                (*it)->exportToPovray(ofs, VisBody::POVRAY_TRANSFORM);
                ofs << "}\n";
                i++;
            }
        }
        ofs.close();

        _frame++;
        _lastTime = t.inSF();
    }
}

}

}
