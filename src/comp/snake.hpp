#ifndef _SIM_COMP_SNAKE_HPP_
#define _SIM_COMP_SNAKE_HPP_

#include <sim/sim.hpp>
#include <vector>
#include <string>
#include <sim/body.hpp>

namespace sim {

namespace comp {

class Snake : public sim::Component {
	sim::Sim *_sim;
	std::vector<sim::Body *> _snakeBodies;

	public:
	Snake(const std::vector<sim::Body *> &snakeBodies);
	~Snake();


	void init(sim::Sim *sim);
	void finish();
	void cbPreStep();
	void processMessage(const Message &msg);
};

} /* namespace comp */

} /* namespace sim */

#endif /* _SIM_COMP_SNAKE_HPP_ */
