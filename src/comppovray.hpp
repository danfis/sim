#ifndef _SIM_POVRAY_COMPONENT_H_
#define _SIM_POVRAY_COMPONENT_H_

#include "sim/ode/world.hpp"
#include "sim/sim.hpp"
#include "msg.hpp"
#include <vector>
#include <string>

/**
  * Povray component
  *
  * the task of this component is to draw objects into povray files which can be processed
  * by 'povray' to obtain high quality picture. The picture can be futhrer joined to video.
  *
  * The component has to be registered into simulator, e.g. in constructor:
  *
  * sim::PovrayComponent *pc = new sim::PovrayComponent(&bodies);
  * addComponent(pc);
  * regPostStep(pc)
  *
  *
  * where 'bodies' is std::vector<sim::VisBody *> - list of pointer to visbody objects
  * the component has registered 'poststep' callback so the scene will be printed to povray after each simulation
  * step.
  *
  * Exporting bodies geometry+position to povray is done by 'exportToPovray' method in visBody* classes
  *
  */


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


