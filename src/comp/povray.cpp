#include "povray.hpp"
#include "sim/common.hpp"

namespace sim {

namespace comp {

Povray::Povray(const char *prefix)
    : sim::Component(), _frame(0), _prefix(prefix)
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

	sprintf(name, "%sframe_%06d.pov", _prefix, _frame);
	std::ofstream ofs(name);
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
}

}

}
