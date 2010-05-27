#ifndef _SIM_COMP_WATCHDOG_HPP_
#define _SIM_COMP_WATCHDOG_HPP_

#include <sim/sim.hpp>
#include <vector>
#include <string>
#include <sim/body.hpp>
#include <sim/joint.hpp>

namespace sim {

namespace comp {

class Watchdog : public sim::Component {
	sim::Sim *_sim;
	sim::Body *_body;
    double _timeout;
	public:
	Watchdog(sim::Body *body, const double timeout);
	~Watchdog();


	void init(sim::Sim *sim);
	void finish();
	void cbPostStep();
	void processMessage(const Message &msg);
};

} /* namespace comp */

} /* namespace sim */

#endif /* _SIM_COMP_SNAKE_HPP_ */
