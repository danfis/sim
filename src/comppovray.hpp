#ifndef _SIM_POVRAY_COMPONENT_H_
#define _SIM_POVRAY_COMPONENT_H_

#include "sim/ode/world.hpp"
#include "sim/sim.hpp"
#include "msg.hpp"
#include <vector>

namespace sim {

class PovrayComponent : public sim::Component {
	sim::Sim *_sim;
	std::vector<sim::VisBody *> *_bodies;
	int frame;
	public:
	PovrayComponent(std::vector<sim::VisBody *> *bodies);
	~PovrayComponent();

	void init(sim::Sim *sim);
	void finish();
	void cbPostStep();
};

}

#endif


