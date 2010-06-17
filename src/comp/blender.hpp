#ifndef _SIM_COMP_BLENDER_HPP_
#define _SIM_COMP_BLENDER_HPP_

#include <sim/sim.hpp>
#include <sim/time.hpp>
#include <vector>
#include <string>


namespace sim {

namespace comp {

    
class Blender : public sim::Component {
	sim::Sim *_sim;
	int _frame;
    const char *_prefix;
    double _frameTime;
    sim::Time _lastFrameTime;

	public:
    /** prefix specifies directory for output puictures. if the directory does not exist, no pictures wil
      * be created. FrameTime specifies time of one frame. this can be used .e.g for porducing movie, where one frame
      * is assumed to be 1/24 s long. if frameTime==-1, all simulation frames are outputted */
	Blender(const char *prefix = "", const double frameTime = 1.0/24.0);
	~Blender();

	void init(sim::Sim *sim);
	void finish();
	void cbPostStep();
};

} /* namespace comp */

} /* namespace sim */

#endif /* _SIM_COMP_POVRAY_HPP_ */
