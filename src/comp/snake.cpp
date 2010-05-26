#include "snake.hpp"
#include <sim/msg.hpp>

namespace sim {

namespace comp {


Snake::Snake(const std::vector<sim::Joint *> &snakeBodies) {
	_snakeBodies = snakeBodies;
	selectedRobot = 0;
//	oldColor = snakeBodies[0]->getColor();
}

Snake::~Snake(){
	_snakeBodies.clear();
}

void Snake::init(sim::Sim *sim) {
	_sim = sim;
	for(int i=0;i<(int)_snakeBodies.size();i++) {
		vel1.push_back(0);
		vel2.push_back(0);
		_snakeBodies[i]->setParamVel(0);			
		_snakeBodies[i]->setParamVel2(0);			
	}
}

void Snake::finish(){
}

void Snake::cbPreStep() {

}

void Snake::processMessage(const Message &msg) {

	const int step = 1;

	if (msg.type() == MessageKeyPressed::Type) {
		const MessageKeyPressed &m = (const MessageKeyPressed &)msg;
		if (m.key() == 110) {
			// 'n'
			selectedRobot=++selectedRobot % _snakeBodies.size();
			DBG("Body " << selectedRobot << " is selected");
		} else if (m.key() == 109) {
			// 'm'
			selectedRobot--;
			if (selectedRobot < 0) {
				selectedRobot = (int)_snakeBodies.size()-1;
			}
			DBG("Body " << selectedRobot << " is selected");

		} else if (m.key() == 65362) {
			// up
			vel1[selectedRobot]+= step;
		} else if (m.key() == 65364) {
			vel1[selectedRobot]-= step;
			// down
		} else if (m.key() == 65361) {
			// left
			vel2[selectedRobot]+= step;
		} else if (m.key() == 65363) {
			// right 
			vel2[selectedRobot]-= step;
		} else {
			DBG("keyboard mesage:" << m.key());
		}

		for(int i=0;i<(int)_snakeBodies.size();i++) {
			_snakeBodies[i]->setParamVel(vel1[i]);			
			_snakeBodies[i]->setParamVel2(vel2[i]);		
			DBG("Joint " << i << ", vel1=" << vel1[i] << ", vel2=" << vel2[i]);	
		}


	}	
}


}

}
