#ifndef _SIM_COMP_POVRAY_HPP_
#define _SIM_COMP_POVRAY_HPP_

#include <sim/sim.hpp>
#include <vector>
#include <string>


namespace sim {

namespace comp {

    
 /**
  * Povray component
  *
  * the task of this component is to draw objects into povray files which can be processed
  * by 'povray' to obtain high quality picture. The picture can be further joined to video.
  *
  * The component has to be registered into simulator, e.g. in constructor:
  *
  * sim::PovrayComponent *pc = new sim::PovrayComponent();
  * addComponent(pc);
  *
  *
  * the component registers 'poststep' callback (cbPostStep()) so the scene will be printed 
  * to povray after each simulation step.
  *
  * Exporting bodies geometry+position to povray is done by 'exportToPovray' method in visBody* classes
  *
  * The result pictures can be processed by povray (http://www.povray.org/) by command:
  * povray -W1024 -H768 frame_xxxx.pov
  * or script demos/script/processPovray.pl can be used for processing batch of .pov files.
  *
  * the result pictures (in png format) can be used to produce a video using script:
  * demos/script/makeVidMen2.sh
  * this script load all png files and run mencoder to make a video in two pass process
  */
class Povray : public sim::Component {
	sim::Sim *_sim;
	int _frame;
    const char *_prefix;

	public:
	Povray(const char *prefix = "");
	~Povray();

	void init(sim::Sim *sim);
	void finish();
	void cbPostStep();
};

} /* namespace comp */

} /* namespace sim */

#endif /* _SIM_COMP_POVRAY_HPP_ */
