
#include "comppovray.hpp"

namespace sim {

PovrayComponent::PovrayComponent(std::vector<sim::VisBody *> *bodies){
	_bodies = bodies;
	frame = 0;
}

PovrayComponent::~PovrayComponent(){
	_bodies = NULL;
}

void PovrayComponent::init(sim::Sim *sim){
	_sim = sim;
	char name[200];
	sprintf(name,"camera.inc");
	std::ofstream ofs(name);
	ofs << "// camera definition \n";
	ofs << "#include \"colors.inc\"\n";
	ofs << "camera {\n";
	ofs << "\tlocation <5,-5,3>\n";
	ofs << "\tsky <0,0,1>\n";
	ofs << "\tlook_at <0,0,3>\n";
	ofs << "}\n";
	ofs << "light_source { <20,-5,20> color White }\n";
	ofs << "light_source { <-20,-5,20> color White }\n";
	ofs << "light_source { <0,-15,10> color White }\n";
	ofs.close();
}

void PovrayComponent::finish(){
}

void PovrayComponent::cbPostStep(){

	char name[200];
	sprintf(name,"frame_%06d.pov",frame);
	std::ofstream ofs(name);
	ofs << "#include \"camera.inc\"\n";
	for(int i=0;i<(int)(*_bodies).size();i++) {
		if ((*_bodies)[i]) {
			ofs << "object {\n";
			((*_bodies)[i])->exportToPovray(ofs,2);
			ofs << "}\n";
		}
	}
	ofs.close();

	frame++;

}

}

