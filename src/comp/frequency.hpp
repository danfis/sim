#ifndef _SIM_COMP_FREQUENCY_HPP_
#define _SIM_COMP_FREQUENCY_HPP_

#include <sim/sim.hpp>
#include <vector>
#include <string>
#include <sim/body.hpp>
#include <sim/joint.hpp>

namespace sim {

namespace comp {

class Frequency : public sim::Component {
	sim::Sim *_sim;
	sim::Joint * _joint;
    double _amplitude;
    double _frequency;
    double _phase;
    int _type;

	public:
	Frequency(sim::Joint *jointm, const double amplitude, const double frequency, const double phase, const int type);
	~Frequency();


	void init(sim::Sim *sim);
	void finish();
	void cbPreStep();
	void processMessage(const Message &msg);
};

} /* namespace comp */

} /* namespace sim */

#endif /* _SIM_COMP_SNAKE_HPP_ */
