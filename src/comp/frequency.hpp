#ifndef _SIM_COMP_FREQUENCY_HPP_
#define _SIM_COMP_FREQUENCY_HPP_

#include <sim/sim.hpp>
#include <vector>
#include <string>
#include <sim/body.hpp>
#include <sim/joint.hpp>

namespace sim {

namespace comp {


/** Frequency component
  *
  * it can be used for controlling a hinge2 joint by applying a sinus function as its velocity
  * if form velocity(t) = A*sin(freq*t + phase)
  * where frequency 'freq', amplitude 'A' and 'phase' are given in constructor.
  *
  * As the hinge2 joint has two axis, the last parameter 'type' specifies whil axis will be controlled:
  * type=0 means that axis1
  * type=1 means that axis2 
  */
class Frequency : public sim::Component {
	sim::Sim *_sim;
	sim::Joint * _joint;
    double _amplitude;
    double _frequency;
    double _phase;
    int _type;

	public:
	Frequency(sim::Joint *joint, const double amplitude, const double frequency, const double phase, const int type);
	~Frequency();


	void init(sim::Sim *sim);
	void finish();
	void cbPreStep();
	void processMessage(const Message &msg);
};

} /* namespace comp */

} /* namespace sim */

#endif /* _SIM_COMP_SNAKE_HPP_ */
