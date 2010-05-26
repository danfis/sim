#ifndef _SIM_COMP_SNAKE_HPP_
#define _SIM_COMP_SNAKE_HPP_

#include <sim/sim.hpp>
#include <vector>
#include <string>
#include <sim/body.hpp>
#include <sim/joint.hpp>

namespace sim {

namespace comp {

class Snake : public sim::Component {
	sim::Sim *_sim;
	std::vector<sim::Joint *> _snakeBodies;
	std::vector<double> vel1,vel2;
//	sim::Vec4 oldColor;
	int selectedRobot;

	public:
	Snake(const std::vector<sim::Joint *> &snakeBodies);
	~Snake();


	void init(sim::Sim *sim);
	void finish();
	void cbPreStep();
	void processMessage(const Message &msg);
};

} /* namespace comp */

} /* namespace sim */

#endif /* _SIM_COMP_SNAKE_HPP_ */
